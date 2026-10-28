"""Teleop gate: drive the QRB base the way the Desktop App's Pose tab does, and measure it.

Runs TeleoperateBase exactly as the Request Teleoperation override does in Pose mode (teleop_mode 2), publishes VelocityForceCommand on
/pose_jog/base at 20 Hz (the jog pad's stream) for a forward and a turn command, then stops, and reports the base's
motion. Exits non-zero if the base does not follow within 15% or does not stop.
"""

import math
import sys
import threading
import time

import rclpy
from moveit_pro_controllers_msgs.msg import VelocityForceCommand
from moveit_studio_sdk_msgs.action import DoObjectiveSequence
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

# Request Teleoperation needs a connected Desktop App (DoTeleoperateAction's goal is rejected with "no UI tab is
# subscribed"), so this gate runs its base branch alone: TeleoperateBase with the same ports the override uses.
OBJECTIVE_XML = """
<root BTCPP_format="4" main_tree_to_execute="Teleop Gate">
  <BehaviorTree ID="Teleop Gate">
    <Action ID="TeleoperateBase" planning_group="base" cmd_vel_topic="/cmd_vel" frame_id="base_footprint"
      max_linear_velocity="0.5" max_angular_velocity="1.0" />
  </BehaviorTree>
</root>"""


def main():
    rclpy.init()
    node = rclpy.create_node("teleop_check")
    buffer = Buffer()
    TransformListener(buffer, node)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    publisher = node.create_publisher(VelocityForceCommand, "/pose_jog/base", 10)

    client = ActionClient(node, DoObjectiveSequence, "/do_objective")
    assert client.wait_for_server(timeout_sec=10), "no /do_objective"
    goal_future = client.send_goal_async(DoObjectiveSequence.Goal(objective_xml_string=OBJECTIVE_XML))
    while not goal_future.done():
        time.sleep(0.05)
    handle = goal_future.result()
    assert handle.accepted, "teleoperation Objective rejected"
    time.sleep(3.0)

    def pose():
        t = buffer.lookup_transform("odom", "base_footprint", Time()).transform
        q = t.rotation
        return t.translation.x, t.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def drive(vx, wz, seconds):
        msg = VelocityForceCommand()
        msg.velocity_controlled_axes.x = True
        msg.velocity_controlled_axes.rz = True
        msg.twist.linear.x = vx
        msg.twist.angular.z = wz
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            msg.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(msg)
            time.sleep(0.05)

    failures = []
    drive(0.4, 0.0, 0.5)
    x0, y0, _ = pose()
    drive(0.4, 0.0, 2.0)
    x1, y1, _ = pose()
    speed = math.hypot(x1 - x0, y1 - y0) / 2.0
    print(f"forward: {speed:.3f} m/s for 0.4 commanded")
    if abs(speed - 0.4) > 0.15 * 0.4:
        failures.append(f"forward {speed:.3f}")

    time.sleep(1.0)
    drive(0.0, 0.8, 0.5)
    _, _, a0 = pose()
    turned, last = 0.0, a0
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        drive(0.0, 0.8, 0.1)
        _, _, a = pose()
        turned += (a - last + math.pi) % (2 * math.pi) - math.pi
        last = a
    print(f"turn: {turned / 2.0:.3f} rad/s for 0.8 commanded")
    if abs(turned / 2.0 - 0.8) > 0.15 * 0.8:
        failures.append(f"turn {turned / 2.0:.3f}")

    time.sleep(1.0)
    xs, ys, _ = pose()
    time.sleep(1.0)
    xe, ye, _ = pose()
    drift = math.hypot(xe - xs, ye - ys)
    print(f"after release: moved {drift:.4f} m in 1 s")
    if drift > 0.01:
        failures.append("base kept moving after commands stopped")

    handle.cancel_goal_async()
    time.sleep(1.0)
    rclpy.shutdown()
    if failures:
        print("FAIL:", "; ".join(failures))
        sys.exit(1)
    print("PASS")


if __name__ == "__main__":
    main()
