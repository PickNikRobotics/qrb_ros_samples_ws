"""Start only the MoveIt Pro Simulator, ros2_control, and robot_state_publisher for rml63_sim.

No move_group, Objective server, or other Runtime nodes. Used to validate the MoveIt Pro Core port of
Qualcomm's pick-and-place node, which plays the role of an application on the device.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

CONTROLLERS = ["joint_state_broadcaster", "rm_group_controller", "hand_controller"]


def generate_launch_description():
    share = Path(get_package_share_directory("rml63_sim"))
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    "xacro ",
                    str(share / "description" / "rml63.urdf.xacro"),
                    " mujoco_viewer:=",
                    LaunchConfiguration("mujoco_viewer"),
                ]
            ),
            value_type=str,
        )
    }
    controllers_yaml = str(share / "config" / "control" / "rml63.ros2_control.yaml")

    nodes = [
        DeclareLaunchArgument("mujoco_viewer", default_value="false"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[controllers_yaml],
            output="screen",
        ),
    ]
    nodes += [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager-timeout", "60"],
            output="screen",
        )
        for name in CONTROLLERS
    ]
    return LaunchDescription(nodes)
