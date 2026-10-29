"""Measure the QRB base's actual motion from simulation ground truth while commands are sent.

Reads base_link_ground_truth, the frame the MoveIt Pro Simulator publishes under mj_world at the base body's true pose.
mj_world -> base_footprint would resolve through the controller's odometry, which agrees with the command by
construction. Reports distance and yaw change over a window.

Usage: python3 base_tracker.py <seconds>
"""

import math
import sys
import time

import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def pose(buffer, node):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            t = buffer.lookup_transform("mj_world", "base_link_ground_truth", Time()).transform
        except Exception:
            continue
        q = t.rotation
        return t.translation.x, t.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    raise RuntimeError("no mj_world -> base_link_ground_truth TF")


def main(seconds):
    rclpy.init()
    node = rclpy.create_node("base_tracker")
    buffer = Buffer()
    TransformListener(buffer, node)
    x0, y0, yaw0 = pose(buffer, node)
    turned, last = 0.0, yaw0
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        _, _, yaw = pose(buffer, node)
        turned += (yaw - last + math.pi) % (2 * math.pi) - math.pi
        last = yaw
    x1, y1, _ = pose(buffer, node)
    dist = math.hypot(x1 - x0, y1 - y0)
    print(f"moved {dist:.3f} m ({dist / seconds:.3f} m/s), turned {turned:+.3f} rad ({turned / seconds:+.3f} rad/s) "
          f"over {seconds:.1f} s")
    rclpy.shutdown()


if __name__ == "__main__":
    main(float(sys.argv[1]))
