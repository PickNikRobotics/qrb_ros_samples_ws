"""Run Qualcomm's pick-and-place sample on MoveIt Pro Core.

Only the application node is started. It expects ros2_control with rm_group_controller and hand_controller, and
publishes nothing for MoveIt: there is no move_group. robot_description is read from this package's URDF and SRDF.
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("qrb_pick_place_core")) / "config"
    joint_limits = yaml.safe_load((config / "joint_limits.yaml").read_text())["joint_limits"]
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="qrb_pick_place_core",
                executable="qrb_ros_arm_pick_place",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "robot_description": (config / "rml63.urdf").read_text(),
                        "robot_description_semantic": (config / "rml_63.srdf").read_text(),
                        "joint_limits": joint_limits,
                    }
                ],
            ),
        ]
    )
