#!/usr/bin/env python3
"""Spawn the BCR arm on a table in a custom Gazebo world (table + 3 ArUco boxes)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_share = get_package_share_directory("arm_simulation")
    desc_share = get_package_share_directory("bcr_arm_description")

    # So Gazebo can resolve model://table, model://aruco_box_* and the arm meshes
    set_models_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(sim_share, "models")
    )
    set_desc_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.dirname(desc_share)
    )

    args = [
        DeclareLaunchArgument("robot_name", default_value="bcr_arm"),
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "world_path",
            default_value=PathJoinSubstitution(
                [FindPackageShare("arm_simulation"), "worlds", "pick_place_table.world"]
            ),
        ),
        DeclareLaunchArgument(
            "ros2_controllers_path",
            default_value=PathJoinSubstitution(
                [FindPackageShare("bcr_arm_moveit_config"), "config", "ros2_controllers.yaml"]
            ),
        ),
        # Arm base pose in the world. Table top is at z = 0.70
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.70"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
    ]

    robot_name = LaunchConfiguration("robot_name")
    use_camera = LaunchConfiguration("use_camera")
    use_sim_time = LaunchConfiguration("use_sim_time")
    world_path = LaunchConfiguration("world_path")
    ros2_controllers_path = LaunchConfiguration("ros2_controllers_path")

    robot_description_cmd = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("bcr_arm_description"), "urdf", "robots", "bcr_arm.urdf.xacro"]
            ),
            " use_gazebo:=true",
            " use_camera:=", use_camera,
            " ros2_controllers_path:=", ros2_controllers_path,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_cmd, value_type=str)}

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": ["-r -v 4 ", world_path]}.items(),
    )

    # Spawn the arm on top of the table
    spawn_arm = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world", "default",
            "-string", robot_description_cmd,
            "-name", robot_name,
            "-allow_renaming", "true",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
            "-Y", LaunchConfiguration("yaw"),
        ],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Needs <topic>camera/image_raw</topic> inside the <sensor> in depth_camera.urdf.xacro
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        output="screen",
        condition=IfCondition(use_camera),
    )

    # Overhead camera (defined in the world file) -> ROS
    overhead_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/overhead_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/overhead_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        output="screen",
    )

    # TF: ROS frame "world" is the arm base (origin of the arm), which sits at z = 0.70 in Gazebo.
    # Camera is at gazebo (0.35, 0, 1.90) -> (0.35, 0, 1.20) relative to the arm base.
    # Rotation turns the optical frame (z forward, x right, y down) to look straight down.
    overhead_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0.35", "--y", "0", "--z", "1.2",
            "--qx", "0.7071068", "--qy", "-0.7071068", "--qz", "0", "--qw", "0",
            "--frame-id", "world",
            "--child-frame-id", "overhead_camera_optical_frame",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    jtc = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn_arm, on_exit=[jsb])
    )
    jtc_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb, on_exit=[jtc])
    )

    return LaunchDescription(
        [set_models_path, set_desc_path]
        + args
        + [rsp, gz_sim, clock_bridge, camera_bridge, overhead_bridge, overhead_tf, spawn_arm, jsb_after_spawn, jtc_after_jsb]
    )