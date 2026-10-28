#!/usr/bin/env python3
"""Relay geometry_msgs/Twist to geometry_msgs/TwistStamped for diff_drive_controller.

Qualcomm's simple_motion node publishes Twist on /qrb_robot_base/cmd_vel, the topic Gazebo's diff-drive plugin read.
ROS 2 Jazzy's diff_drive_controller accepts only TwistStamped, and drops a command older than its cmd_vel_timeout, so
each message is stamped on arrival. The last command is re-sent at republish_rate until a new one arrives, which keeps
the original node's press-once-and-drive behavior (Gazebo latched the last command); set republish_rate to 0 to send
each command once.
"""

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class TwistStamper(Node):
    def __init__(self):
        super().__init__("twist_stamper")
        self.declare_parameter("input_topic", "/qrb_robot_base/cmd_vel")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("frame_id", "base_footprint")
        self.declare_parameter("republish_rate", 10.0)
        self._frame_id = self.get_parameter("frame_id").value
        self._last = None
        self._publisher = self.create_publisher(TwistStamped, self.get_parameter("output_topic").value, 10)
        self.create_subscription(Twist, self.get_parameter("input_topic").value, self._on_twist, 10)
        rate = self.get_parameter("republish_rate").value
        if rate > 0.0:
            self.create_timer(1.0 / rate, self._republish)

    def _on_twist(self, msg):
        self._last = msg
        self._publish(msg)

    def _republish(self):
        if self._last is not None:
            self._publish(self._last)

    def _publish(self, twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self._frame_id
        stamped.twist = twist
        self._publisher.publish(stamped)


def main():
    rclpy.init()
    node = TwistStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
