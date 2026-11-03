"""Small helper to drive /arm_control from your own scripts (and switch modes between moves).

    from arm_control.client import ArmClient
    arm = ArmClient()
    arm.set_mode("joint");  arm.go_home()
    arm.set_mode("tcp");    arm.move_tcp([0.5, -0.1, 0.18])
"""
import json
import time

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


class ArmClient:
    def __init__(self, node: Node = None):
        self._own = node is None
        if self._own:
            if not rclpy.ok():
                rclpy.init()
            node = Node("arm_client")
        self.node = node
        self.status = dict(mode="?", state="?", queue=0, seq=0, last_result="none")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(String, "/arm_control/status", self._on_status, 10)
        self.mode = None
        node.create_subscription(String, "/arm_control/mode", self._on_mode, latched)
        self.pub_joint = node.create_publisher(JointState, "/arm_control/joint_cmd", 10)
        self.pub_wp = node.create_publisher(PoseStamped, "/arm_control/tcp_waypoint", 10)
        self.pub_wps = node.create_publisher(PoseArray, "/arm_control/tcp_waypoints", 10)
        self._srv = {n: node.create_client(Trigger, f"/arm_control/{n}") for n in
                     ("set_joint_mode", "set_tcp_mode", "get_mode", "stop", "go_home", "goto_marker")}
        self._spin(1.0)

    # ------------------------------------------------------------ plumbing
    def _on_status(self, m):
        self.status = json.loads(m.data)

    def _on_mode(self, m):
        self.mode = m.data

    def _spin(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _wait_subscribers(self, pub, timeout=3.0):
        end = time.time() + timeout
        while pub.get_subscription_count() == 0 and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _call(self, name, timeout=5.0):
        cli = self._srv[name]
        if not cli.wait_for_service(timeout_sec=timeout):
            return False, f"service /arm_control/{name} not available (is arm_controller running?)"
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        r = fut.result()
        return (r.success, r.message) if r else (False, "service call timed out")

    def wait_idle(self, seq0, timeout=60.0):
        """Wait until the command issued after `seq0` finished. Returns the controller's result string."""
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            s = self.status
            if s["seq"] > seq0 and s["state"] == "idle" and s["queue"] == 0:
                return s["last_result"]
        return "timeout"

    def _seq(self):
        self._spin(0.3)
        return self.status["seq"]

    # ------------------------------------------------------------ API
    def set_mode(self, mode):
        assert mode in ("joint", "tcp")
        ok, msg = self._call(f"set_{mode}_mode")
        self._spin(0.3)
        return ok, msg

    def stop(self):
        return self._call("stop")

    def go_home(self, wait=True):
        seq0 = self._seq()
        ok, msg = self._call("go_home")
        return self.wait_idle(seq0) if ok and wait else msg

    def move_joints(self, positions, names=None, wait=True):
        seq0 = self._seq()
        self._wait_subscribers(self.pub_joint)
        m = JointState()
        m.name = list(names) if names else []
        m.position = [float(x) for x in positions]
        self.pub_joint.publish(m)
        return self.wait_idle(seq0) if wait else "sent"

    @staticmethod
    def _pose(xyz, quat):
        p = Pose()
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        return p

    def move_tcp(self, xyz, quat=(1.0, 0.0, 0.0, 0.0), frame="base_link", wait=True):
        """quat is (x, y, z, w). Default (1,0,0,0) = tool pointing straight down."""
        seq0 = self._seq()
        self._wait_subscribers(self.pub_wp)
        m = PoseStamped()
        m.header.frame_id = frame
        m.pose = self._pose(xyz, quat)
        self.pub_wp.publish(m)
        return self.wait_idle(seq0) if wait else "sent"

    def move_tcp_path(self, waypoints, frame="base_link", wait=True):
        """waypoints: list of (xyz, quat)."""
        seq0 = self._seq()
        self._wait_subscribers(self.pub_wps)
        m = PoseArray()
        m.header.frame_id = frame
        m.poses = [self._pose(x, q) for x, q in waypoints]
        self.pub_wps.publish(m)
        return self.wait_idle(seq0) if wait else "sent"

    def goto_marker(self, wait=True):
        seq0 = self._seq()
        ok, msg = self._call("goto_marker")
        return self.wait_idle(seq0) if ok and wait else msg

    def close(self):
        if self._own:
            self.node.destroy_node()
            rclpy.shutdown()
