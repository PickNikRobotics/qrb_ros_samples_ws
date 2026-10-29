"""Start only the MoveIt Pro Simulator, ros2_control, and robot_state_publisher for qrb_remote_assistant_sim.

No Runtime nodes: the layer-2 gate runs Qualcomm's remote assistant pipeline (remote_assistant_pipeline.launch.py) against the
simulator and controllers alone.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

CONTROLLERS = ["joint_state_broadcaster", "platform_velocity_controller"]


def generate_launch_description():
    share = Path(get_package_share_directory("qrb_remote_assistant_sim"))
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", str(share / "description" / "qrb_robot_base_mini.urdf.xacro")]), value_type=str
        )
    }
    controllers_yaml = str(share / "config" / "control" / "qrb_robot_base_mini.ros2_control.yaml")
    nodes = [
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[robot_description]),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[controllers_yaml],
            remappings=[
                ("/platform_velocity_controller/cmd_vel", "/cmd_vel"),
                ("/platform_velocity_controller/odom", "/odom"),
            ],
            output="screen",
        ),
    ]
    nodes += [
        Node(package="controller_manager", executable="spawner", arguments=[name, "--controller-manager-timeout", "60"])
        for name in CONTROLLERS
    ]
    nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / "remote_assistant_pipeline.launch.py")),
            launch_arguments={"control_period": LaunchConfiguration("control_period")}.items(),
        )
    )
    return LaunchDescription([DeclareLaunchArgument("control_period", default_value="0.05")] + nodes)
