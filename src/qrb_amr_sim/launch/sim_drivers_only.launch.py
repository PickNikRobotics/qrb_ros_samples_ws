"""Start only the MoveIt Pro Simulator, ros2_control, and robot_state_publisher for qrb_amr_sim.

No Runtime nodes. The twist_stamper relay lets Qualcomm's simple_motion node (Twist on /qrb_robot_base/cmd_vel) drive
diff_drive_controller (TwistStamped on /platform_velocity_controller/cmd_vel) unchanged.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

CONTROLLERS = ["joint_state_broadcaster", "platform_velocity_controller"]


def generate_launch_description():
    share = Path(get_package_share_directory("qrb_amr_sim"))
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", str(share / "description" / "qrb_robot_base.urdf.xacro")]), value_type=str
        )
    }
    controllers_yaml = str(share / "config" / "control" / "qrb_robot_base.ros2_control.yaml")
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
        Node(package="qrb_amr_sim", executable="twist_stamper.py", output="screen"),
    ]
    nodes += [
        Node(package="controller_manager", executable="spawner", arguments=[name, "--controller-manager-timeout", "60"])
        for name in CONTROLLERS
    ]
    return LaunchDescription(nodes)
