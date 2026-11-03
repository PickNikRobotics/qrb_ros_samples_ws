"""Nodes that start with the Simulator for qrb_amr_sim (config.yaml hardware.additional_driver_launch_file).

twist_stamper.py lets Qualcomm's unmodified simple_motion node, which publishes Twist on /qrb_robot_base/cmd_vel,
drive diff_drive_controller, which takes TwistStamped only. The Desktop App's teleoperation publishes TwistStamped
on /cmd_vel directly and does not need it.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(package="qrb_amr_sim", executable="twist_stamper.py", output="screen")])
