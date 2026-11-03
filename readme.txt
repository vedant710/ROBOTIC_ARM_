# ROBOTIC_ARM_

A 7-DoF robotic arm simulated in **ROS 2 + Gazebo**, built for pick-and-place
and multi-task manipulation with a perception interface. It is meant as a
**learning platform**: each layer (kinematics, trajectory planning, control,
perception, simulation) is separate, so you can swap in your own algorithm and
see how the arm behaves.

> **Status:** simulation, kinematics, planning and the mode-switched controller
> work today. Perception is currently an ArUco-marker interface
> (`/perception/marker_pose`), and AI-based decision making is on the roadmap.

## What's inside

| Package | Purpose |
|---|---|
| `arm_control` | Custom controller node. Moves the arm in **joint space** or **TCP (Cartesian) space**, switched at runtime by a service. Includes pure-Python (numpy) kinematics and trajectory planning, plus a small `ArmClient` helper and a demo script. |
| `arm_simulation` | Gazebo world with a table and three ArUco-marked boxes (red, green, blue) for pick-and-place experiments, plus launch and RViz files. |
| `bcr_arm_description` | URDF/xacro, meshes, RViz config and depth-camera description of the BCR arm (from Black Coffee Robotics, Apache-2.0). |
| `bcr_arm_gazebo` | Gazebo launch files, extra object models (coke can, cardboard box, etc.) and CLI test scripts. |
| `bcr_arm_moveit_config` | MoveIt 2 configuration for the arm, for comparing against the custom planner. |

## Features

- Joint-space and Cartesian (TCP) waypoint control, exclusive modes switched by service
- Smooth, eased trajectories with joint velocity limiting
- Numpy-only kinematics (no ROS imports), so it is easy to unit-test and study
- `goto_marker` service: move above a marker pose published by perception
- Simulated overhead camera and ArUco boxes for vision experiments
- MoveIt 2 configuration included
- `ArmClient` Python helper for scripting your own tasks

## Concepts you can explore

- Forward and inverse kinematics on a 7-DoF arm
- Trajectory generation (joint-space and straight-line Cartesian)
- Mode-based controller design with ROS 2 topics, services and actions
- Marker-based perception to pick-and-place
- Sim-to-real workflow with `ros2_control`

## Requirements

- Ubuntu with ROS 2 (<confirm your distro, e.g. Jazzy>)
- Gazebo with `ros_gz_sim` and `ros_gz_bridge`
- `ros2_control`, `controller_manager`, `joint_trajectory_controller`
- MoveIt 2 (optional)
- Python 3, numpy

## Build

```bash
mkdir -p ~/arm_ws/src && cd ~/arm_ws/src
git clone https://github.com/vedant710/ROBOTIC_ARM_.git
cd ~/arm_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
# 1. Arm on a table with ArUco boxes in Gazebo
ros2 launch arm_simulation arm_table.launch.py

# 2. Controller node
ros2 launch arm_control arm_controller.launch.py

# 3. Demo: joint move, switch to TCP mode, move over the boxes
ros2 run arm_control example_client
```

Use it from your own script:

```python
from arm_control.client import ArmClient

arm = ArmClient()
arm.set_mode("joint"); arm.go_home()
arm.set_mode("tcp");   arm.move_tcp([0.5, -0.1, 0.18])
```

## Controller interface (`arm_control`)

| Topic (in) | Type | Mode |
|---|---|---|
| `/arm_control/joint_cmd` | `sensor_msgs/JointState` | joint |
| `/arm_control/tcp_waypoint` | `geometry_msgs/PoseStamped` | tcp |
| `/arm_control/tcp_waypoints` | `geometry_msgs/PoseArray` | tcp |
| `/perception/marker_pose` | `geometry_msgs/PoseStamped` | any |

Services (`std_srvs/Trigger`): `set_joint_mode`, `set_tcp_mode`, `get_mode`,
`stop`, `go_home` (joint mode), `goto_marker` (tcp mode), all under
`/arm_control/`.
Publishes `/arm_control/mode`, `/arm_control/status`, `/arm_control/current_tcp_pose`.
Output goes to `/joint_trajectory_controller/follow_joint_trajectory`.
Tunable parameters (speed scaling, TCP offset, home pose, marker approach
offset) are in `arm_control/config/arm_controller.yaml`.

## Repository structure

```
ROBOTIC_ARM_/
├── arm_control/            # controller, kinematics, planning, client
├── arm_simulation/         # table + ArUco box Gazebo world
├── bcr_arm_description/    # URDF, meshes, sensors
├── bcr_arm_gazebo/         # Gazebo launch, models, test scripts
└── bcr_arm_moveit_config/  # MoveIt 2 config
```

## Roadmap

- [ ] Perception node publishing `/perception/marker_pose` from the camera
- [ ] Gripper and full pick-and-place sequence
- [ ] AI-based object detection and task selection
- [ ] Multi-task behaviours (sorting, stacking)
- [ ] Hardware deployment

## Credits

The arm model and Gazebo/MoveIt packages (`bcr_arm_*`) come from
[Black Coffee Robotics](https://github.com/blackcoffeerobotics) under the
Apache License 2.0. `arm_control` and `arm_simulation` are original work for
this project.

## License

Apache License 2.0. See [LICENSE](LICENSE).

## Author

Vedant
