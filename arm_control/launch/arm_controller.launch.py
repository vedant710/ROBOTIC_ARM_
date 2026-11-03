import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("arm_control"), "config", "arm_controller.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("initial_mode", default_value="joint"),
        Node(
            package="arm_control",
            executable="arm_controller",
            name="arm_controller",
            output="screen",
            parameters=[params, {"use_sim_time": LaunchConfiguration("use_sim_time"),
                                 "initial_mode": LaunchConfiguration("initial_mode")}],
        ),
    ])
