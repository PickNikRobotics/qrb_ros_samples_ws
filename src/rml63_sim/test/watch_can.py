"""Print coke1's pose and the gripper joints at 5 Hz for N seconds (debug aid for grasp reliability)."""

import sys
import time

import rclpy
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

HAND = ["joint01", "joint11", "joint02", "joint22"]


def main(seconds):
    rclpy.init()
    node = rclpy.create_node("watch_can")
    buffer = Buffer()
    TransformListener(buffer, node)
    hand = {}
    node.create_subscription(JointState, "/joint_states", lambda m: hand.update(zip(m.name, m.position)), 10)
    end = time.monotonic() + seconds
    next_print = 0.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.monotonic() < next_print:
            continue
        next_print = time.monotonic() + 0.2
        try:
            t = buffer.lookup_transform("mj_world", "coke1_site", rclpy.time.Time()).transform
        except Exception:
            continue
        q = t.rotation
        up = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        fingers = " ".join(f"{hand.get(j, float('nan')):+.3f}" for j in HAND)
        p = t.translation
        print(f"{time.monotonic():.1f} can {p.x:.3f} {p.y:.3f} {p.z:.3f} up {up:+.2f} fingers {fingers}", flush=True)
    rclpy.shutdown()


if __name__ == "__main__":
    main(float(sys.argv[1]))
