# arm_control

One node (`arm_controller`) that moves the arm in two exclusive spaces and is switched by service.

## Topics it LISTENS to
| Topic | Type | Mode | Meaning |
|---|---|---|---|
| `/arm_control/joint_cmd` | sensor_msgs/JointState | joint | target joints (7 positions, or names+positions for a subset) |
| `/arm_control/tcp_waypoint` | geometry_msgs/PoseStamped | tcp | one waypoint, appended to the queue |
| `/arm_control/tcp_waypoints` | geometry_msgs/PoseArray | tcp | several waypoints, appended in order |
| `/perception/marker_pose` | geometry_msgs/PoseStamped | any (stored) | latest marker pose, used by `goto_marker` |
| `/joint_states`, `/robot_description` | | | state + kinematics (from the simulation) |

A command meant for the other mode is ignored and reported in `/arm_control/status`.

## Topics it PUBLISHES
`/arm_control/mode` (String, latched), `/arm_control/status` (JSON String), `/arm_control/current_tcp_pose` (PoseStamped)

## Services (std_srvs/Trigger)
`/arm_control/set_joint_mode`, `/arm_control/set_tcp_mode`, `/arm_control/get_mode`, `/arm_control/stop`,
`/arm_control/go_home` (joint mode only), `/arm_control/goto_marker` (tcp mode only)

## Output
FollowJointTrajectory goals to `/joint_trajectory_controller/follow_joint_trajectory`.
