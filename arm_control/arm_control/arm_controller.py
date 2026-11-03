#!/usr/bin/env python3
"""Dual-space arm controller.

Two exclusive modes, switched by service:
  * joint : listens on /arm_control/joint_cmd       (sensor_msgs/JointState)
  * tcp   : listens on /arm_control/tcp_waypoint    (geometry_msgs/PoseStamped, queued)
                       /arm_control/tcp_waypoints   (geometry_msgs/PoseArray,   queued in order)
A command that belongs to the *other* mode is ignored (and reported in /arm_control/status).

It sits on top of the ros2_control joint_trajectory_controller (FollowJointTrajectory action).
TCP moves are straight lines in Cartesian space solved with a built-in damped-least-squares IK.
"""
import json
from collections import deque

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

from .kinematics import Chain, mat_to_pose, pose_to_mat
from .planning import PlanError, plan_cartesian, plan_joint

MODES = ("joint", "tcp")


def _dur(t):
    sec = int(t)
    return Duration(sec=sec, nanosec=int(round((t - sec) * 1e9)))


class ArmController(Node):
    def __init__(self):
        super().__init__("arm_controller")

        # ---------------------------------------------------------------- parameters
        p = self.declare_parameter
        p("base_frame", "base_link")
        p("tip_link", "flange")
        p("tcp_offset", [0.0, 0.0, 0.105])
        p("trajectory_action", "/joint_trajectory_controller/follow_joint_trajectory")
        p("joint_states_topic", "/joint_states")
        p("robot_description_topic", "/robot_description")
        p("initial_mode", "joint")
        p("cancel_on_mode_switch", True)
        p("joint_speed_scale", 0.3)
        p("tcp_linear_speed", 0.10)
        p("tcp_angular_speed", 0.5)
        p("cartesian_step", 0.01)
        p("angular_step", 0.05)
        p("home_joints", [-2.306, -1.062, -1.355, 1.715, -1.033, 1.686, 2.639])
        p("marker_pose_topic", "/perception/marker_pose")
        p("marker_timeout", 2.0)
        p("marker_approach_offset", [0.0, 0.0, 0.12])
        p("marker_approach_quat", [1.0, 0.0, 0.0, 0.0])

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.base_frame = gp("base_frame")
        self.tip_link = gp("tip_link")
        self.tcp_offset = list(gp("tcp_offset"))

        mode = gp("initial_mode")
        if mode not in MODES:
            self.get_logger().warn(f"initial_mode '{mode}' invalid, using 'joint'")
            mode = "joint"
        self.mode = mode

        # ---------------------------------------------------------------- state
        self.chain = None
        self.q = None                      # current joints, chain order
        self.queue = deque()               # pending TCP waypoints (4x4 in base frame)
        self.executing = False
        self.goal_handle = None
        self.gen = 0                       # increments on every new/cancelled goal (ignores stale callbacks)
        self.seq = 0                       # increments on every command outcome (for clients)
        self.state = "waiting_for_robot_description"
        self.last_result = "none"
        self.marker = None                 # (PoseStamped, arrival Time)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------------------------------------------------------- interfaces
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)

        # --- LISTENING (inputs)
        self.create_subscription(String, gp("robot_description_topic"), self._on_urdf, latched)
        self.create_subscription(JointState, gp("joint_states_topic"), self._on_joint_states,
                                 qos_profile_sensor_data)
        self.create_subscription(JointState, "/arm_control/joint_cmd", self._on_joint_cmd, 10)
        self.create_subscription(PoseStamped, "/arm_control/tcp_waypoint", self._on_tcp_waypoint, 10)
        self.create_subscription(PoseArray, "/arm_control/tcp_waypoints", self._on_tcp_waypoints, 10)
        self.create_subscription(PoseStamped, gp("marker_pose_topic"), self._on_marker, 10)

        # --- PUBLISHING (outputs)
        self.pub_mode = self.create_publisher(String, "/arm_control/mode", latched)
        self.pub_status = self.create_publisher(String, "/arm_control/status", 10)
        self.pub_tcp = self.create_publisher(PoseStamped, "/arm_control/current_tcp_pose", 10)

        # --- SERVICES
        self.create_service(Trigger, "/arm_control/set_joint_mode", lambda rq, rs: self._srv_set_mode("joint", rs))
        self.create_service(Trigger, "/arm_control/set_tcp_mode", lambda rq, rs: self._srv_set_mode("tcp", rs))
        self.create_service(Trigger, "/arm_control/get_mode", self._srv_get_mode)
        self.create_service(Trigger, "/arm_control/stop", self._srv_stop)
        self.create_service(Trigger, "/arm_control/go_home", self._srv_go_home)        # joint mode only
        self.create_service(Trigger, "/arm_control/goto_marker", self._srv_goto_marker)  # tcp mode only

        # --- downstream: ros2_control joint_trajectory_controller
        self.client = ActionClient(self, FollowJointTrajectory, gp("trajectory_action"))

        self.create_timer(0.05, self._publish_tcp_pose)
        self.create_timer(0.2, self._publish_status)
        self._publish_mode()
        self.get_logger().info(f"arm_controller up, mode='{self.mode}'. Waiting for robot_description...")

    # ====================================================================== helpers
    def _publish_mode(self):
        self.pub_mode.publish(String(data=self.mode))

    def _publish_status(self):
        self.pub_status.publish(String(data=json.dumps(dict(
            mode=self.mode, state=self.state, queue=len(self.queue),
            seq=self.seq, last_result=self.last_result))))

    def _outcome(self, result, level="info"):
        """Record the outcome of a command (clients watch `seq`)."""
        self.last_result = result
        self.seq += 1
        getattr(self.get_logger(), level)(result)
        self._publish_status()

    def _ready(self):
        if self.chain is None:
            self._outcome("rejected: robot_description not received yet", "warn")
            return False
        if self.q is None:
            self._outcome("rejected: no /joint_states yet", "warn")
            return False
        return True

    def _gate(self, required, what):
        if self.mode != required:
            self._outcome(f"rejected: {what} ignored, controller is in '{self.mode}' mode "
                          f"(needs '{required}' mode)", "warn")
            return False
        return True

    def _pose_to_base(self, frame_id, pose: Pose):
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        if np.linalg.norm(q) < 1e-6:
            raise ValueError("pose has a zero quaternion (set orientation.w = 1.0 at least)")
        T = pose_to_mat([pose.position.x, pose.position.y, pose.position.z], q)
        frame = frame_id or self.base_frame
        if frame != self.base_frame:
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, frame, Time())
            except Exception as e:  # noqa: BLE001
                raise ValueError(f"cannot transform '{frame}' -> '{self.base_frame}': {e}")
            t, r = tf.transform.translation, tf.transform.rotation
            T = pose_to_mat([t.x, t.y, t.z], [r.x, r.y, r.z, r.w]) @ T
        return T

    # ====================================================================== inputs
    def _on_urdf(self, msg):
        if self.chain is not None:
            return
        try:
            self.chain = Chain(msg.data, self.base_frame, self.tip_link, self.tcp_offset)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"could not build kinematic chain: {e}")
            return
        self.state = "idle"
        self.get_logger().info(f"kinematic chain ready: {self.chain.joint_names} "
                               f"({self.base_frame} -> {self.tip_link}, tcp_offset={self.tcp_offset})")

    def _on_joint_states(self, msg):
        if self.chain is None:
            return
        try:
            idx = [msg.name.index(n) for n in self.chain.joint_names]
        except ValueError:
            return
        self.q = np.array([msg.position[i] for i in idx])

    def _on_marker(self, msg):
        self.marker = (msg, self.get_clock().now())

    def _on_joint_cmd(self, msg: JointState):
        if not self._gate("joint", "joint_cmd"):
            return
        if not self._ready():
            return
        goal = self.q.copy()
        if not msg.name:
            if len(msg.position) != self.chain.n:
                return self._outcome(f"rejected: joint_cmd needs {self.chain.n} positions "
                                     f"(or names), got {len(msg.position)}", "error")
            goal = np.array(msg.position, float)
        else:
            for n, v in zip(msg.name, msg.position):
                if n not in self.chain.joint_names:
                    return self._outcome(f"rejected: unknown joint '{n}'", "error")
                goal[self.chain.joint_names.index(n)] = v
        try:
            pos, vel, t = plan_joint(self.chain, self.q, goal, self.get_parameter("joint_speed_scale").value)
        except PlanError as e:
            return self._outcome(f"rejected: {e}", "error")
        self._send(pos, vel, t, "joint move")

    def _enqueue(self, T, label):
        self.queue.append(T)
        p = T[:3, 3]
        self.get_logger().info(f"queued TCP waypoint {label}: [{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}] "
                               f"(queue={len(self.queue)})")
        self._try_start_next()

    def _on_tcp_waypoint(self, msg: PoseStamped):
        if not self._gate("tcp", "tcp_waypoint"):
            return
        if not self._ready():
            return
        try:
            T = self._pose_to_base(msg.header.frame_id, msg.pose)
        except ValueError as e:
            return self._outcome(f"rejected: {e}", "error")
        self._enqueue(T, "")

    def _on_tcp_waypoints(self, msg: PoseArray):
        if not self._gate("tcp", "tcp_waypoints"):
            return
        if not self._ready():
            return
        try:
            Ts = [self._pose_to_base(msg.header.frame_id, pz) for pz in msg.poses]
        except ValueError as e:
            return self._outcome(f"rejected: {e}", "error")
        for i, T in enumerate(Ts):
            self.queue.append(T)
        self.get_logger().info(f"queued {len(Ts)} TCP waypoints (queue={len(self.queue)})")
        self._try_start_next()

    # ====================================================================== motion
    def _try_start_next(self):
        if self.executing or not self.queue or self.mode != "tcp":
            return
        T = self.queue.popleft()
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        try:
            pos, vel, t = plan_cartesian(
                self.chain, self.q, T,
                lin_speed=gp("tcp_linear_speed"), ang_speed=gp("tcp_angular_speed"),
                lin_step=gp("cartesian_step"), ang_step=gp("angular_step"),
                speed_scale=gp("joint_speed_scale"))
        except PlanError as e:
            self.queue.clear()
            return self._outcome(f"error: waypoint rejected ({e}); queue cleared", "error")
        self._send(pos, vel, t, "tcp waypoint")

    def _send(self, pos, vel, t, label):
        if not self.client.server_is_ready():
            self.queue.clear()
            return self._outcome("error: trajectory action server not available "
                                 "(is joint_trajectory_controller active?)", "error")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.chain.joint_names)
        for i in range(len(t)):
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in pos[i]]
            pt.velocities = [float(x) for x in vel[i]]
            pt.time_from_start = _dur(float(t[i]))
            goal.trajectory.points.append(pt)
        self.gen += 1
        gen = self.gen
        self.executing = True
        self.state = "moving"
        self.get_logger().info(f"{label}: {len(t)} points, {t[-1]:.2f} s")
        self.client.send_goal_async(goal).add_done_callback(lambda f, g=gen: self._on_goal_response(f, g))

    def _on_goal_response(self, fut, gen):
        gh = fut.result()
        if gh is None or not gh.accepted:
            if gen == self.gen:
                self.executing = False
                self.state = "idle"
                self.queue.clear()
                self._outcome("error: trajectory goal rejected by controller", "error")
            return
        if gen != self.gen:                 # superseded / cancelled while in flight
            gh.cancel_goal_async()
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(lambda f, g=gen: self._on_result(f, g))

    def _on_result(self, fut, gen):
        if gen != self.gen:
            return
        res = fut.result()
        self.executing = False
        self.goal_handle = None
        self.state = "idle"
        if res.status == GoalStatus.STATUS_SUCCEEDED and res.result.error_code == 0:
            self._outcome("ok")
            self._try_start_next()
        elif res.status == GoalStatus.STATUS_CANCELED:
            self.queue.clear()
            self._outcome("cancelled")
        else:
            self.queue.clear()
            self._outcome(f"error: controller returned code {res.result.error_code} "
                          f"'{res.result.error_string}'; queue cleared", "error")

    def _cancel_all(self, reason):
        self.queue.clear()
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        was_busy = self.executing
        self.gen += 1
        self.executing = False
        self.goal_handle = None
        self.state = "idle" if self.chain is not None else self.state
        if was_busy:
            self._outcome(f"cancelled: {reason}")

    # ====================================================================== services
    def _srv_set_mode(self, mode, res):
        if mode == self.mode:
            res.success, res.message = True, f"already in '{mode}' mode"
            return res
        busy = self.executing or len(self.queue) > 0
        if busy and not self.get_parameter("cancel_on_mode_switch").value:
            res.success, res.message = False, "busy: motion in progress (stop first or set cancel_on_mode_switch)"
            return res
        if busy:
            self._cancel_all("mode switch")
        self.mode = mode
        self._publish_mode()
        self.get_logger().info(f"mode -> '{mode}'")
        res.success, res.message = True, f"mode is now '{mode}'"
        return res

    def _srv_get_mode(self, _rq, res):
        res.success, res.message = True, self.mode
        return res

    def _srv_stop(self, _rq, res):
        self._cancel_all("stop requested")
        res.success, res.message = True, "stopped, queue cleared"
        return res

    def _srv_go_home(self, _rq, res):
        if self.mode != "joint":
            res.success, res.message = False, "go_home is a joint-space move: switch to 'joint' mode first"
            return res
        if not self._ready():
            res.success, res.message = False, self.last_result
            return res
        try:
            home = np.array(self.get_parameter("home_joints").value, float)
            pos, vel, t = plan_joint(self.chain, self.q, home, self.get_parameter("joint_speed_scale").value)
        except PlanError as e:
            res.success, res.message = False, str(e)
            return res
        self._send(pos, vel, t, "go_home")
        res.success, res.message = True, "moving home"
        return res

    def _srv_goto_marker(self, _rq, res):
        if self.mode != "tcp":
            res.success, res.message = False, "goto_marker is a TCP move: switch to 'tcp' mode first"
            return res
        if not self._ready():
            res.success, res.message = False, self.last_result
            return res
        if self.marker is None:
            res.success, res.message = False, "no marker pose received yet"
            return res
        msg, arrived = self.marker
        age = (self.get_clock().now() - arrived).nanoseconds * 1e-9
        if age > self.get_parameter("marker_timeout").value:
            res.success, res.message = False, f"marker pose is stale ({age:.1f}s old)"
            return res
        try:
            Tm = self._pose_to_base(msg.header.frame_id, msg.pose)
        except ValueError as e:
            res.success, res.message = False, str(e)
            return res
        off = np.array(self.get_parameter("marker_approach_offset").value, float)
        quat = self.get_parameter("marker_approach_quat").value
        T = pose_to_mat(Tm[:3, 3] + off, quat)      # offset is expressed in the base frame
        self._enqueue(T, "(marker)")
        p = T[:3, 3]
        res.success, res.message = True, f"going to marker approach point [{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}]"
        return res

    # ====================================================================== outputs
    def _publish_tcp_pose(self):
        if self.chain is None or self.q is None:
            return
        pos, quat = mat_to_pose(self.chain.fk(self.q))
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.base_frame
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, pos)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = map(float, quat)
        self.pub_tcp.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
