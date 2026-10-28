"""Live check of where coke1 is, read from the MoveIt Pro Simulator's TF (site coke1_site in mj_world).

Usage:
  python3 coke_tracker.py lifted   # pass if coke1 is >= 0.05 m above the table top (1.015 m)
  python3 coke_tracker.py placed   # pass if coke1 rests upright on the cardboard box top (1.32 m) within 0.05 m xy
                                   # of the box-top drop point measured by grasp_gate.py --full
  python3 coke_tracker.py home     # pass if coke1 is at its spawn pose on the table
  python3 coke_tracker.py moved    # pass if coke1 left the table: picked and carried to the box area (the original
                                   # re_raise sweeps the open gripper through the released can, knocking it off
                                   # about 1 run in 4, so its final pose is not a migration signal)
"""

import sys

import numpy as np
import rclpy
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener

TABLE_TOP_Z = 1.015
BOX_TOP_Z = 1.32
SPAWN_XY = np.array([3.0, -2.0])
# Release point of Qualcomm's PLACE pose in this scene (grasp_gate.py --full: tcp 3.086 -2.53).
PLACE_XY = np.array([3.08, -2.52])


def coke_pose(node, buffer):
    deadline = node.get_clock().now() + Duration(seconds=10)
    while node.get_clock().now() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if buffer.can_transform("mj_world", "coke1_site", rclpy.time.Time()):
            t = buffer.lookup_transform("mj_world", "coke1_site", rclpy.time.Time())
            p = t.transform.translation
            q = t.transform.rotation
            up = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
            return np.array([p.x, p.y, p.z]), up
    raise RuntimeError("no TF mj_world -> coke1_site within 10 s; is the simulator running?")


def main(mode):
    rclpy.init()
    node = rclpy.create_node("coke_tracker")
    buffer = Buffer()
    TransformListener(buffer, node)
    pos, up = coke_pose(node, buffer)
    print(f"coke1 {pos.round(3)} up {up:.2f}")
    ok = {
        "lifted": pos[2] > TABLE_TOP_Z + 0.05,
        "placed": abs(pos[2] - BOX_TOP_Z) < 0.01 and up > 0.95 and np.linalg.norm(pos[:2] - PLACE_XY) < 0.05,
        "home": abs(pos[2] - TABLE_TOP_Z) < 0.01 and np.linalg.norm(pos[:2] - SPAWN_XY) < 0.01,
        "moved": np.linalg.norm(pos[:2] - SPAWN_XY) > 0.3 or pos[2] > TABLE_TOP_Z + 0.1,
    }[mode]
    print("PASS" if ok else "FAIL")
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main(sys.argv[1])
