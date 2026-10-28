"""Offline MuJoCo check: scene loads, the arm holds its default keyframe, and objects stay put.

Usage: python3 settle_check.py <scene.xml>
Exits non-zero on failure.
"""

import sys

import mujoco
import numpy as np

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
STEPS = 2000


def main(scene):
    m = mujoco.MjModel.from_xml_path(scene)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key("default").id)
    mujoco.mj_forward(m, d)
    start_arm = np.array([d.joint(j).qpos[0] for j in ARM_JOINTS])
    start_coke = d.body("coke1").xpos.copy()
    for _ in range(STEPS):
        mujoco.mj_step(m, d)
    arm = np.array([d.joint(j).qpos[0] for j in ARM_JOINTS])
    coke = d.body("coke1").xpos
    print("coke1", coke, "arm drift", np.abs(arm - start_arm).max())
    failures = []
    if not np.all(np.isfinite(d.qpos)):
        failures.append("non-finite qpos (simulation exploded)")
    if np.abs(arm - start_arm).max() > 0.01:
        failures.append(f"arm drifted {np.abs(arm - start_arm).max():.4f} rad")
    if not (1.0 < coke[2] < 1.1) or np.linalg.norm(coke[:2] - start_coke[:2]) > 0.01:
        failures.append(f"coke1 moved to {coke}")
    if failures:
        print("FAIL:", "; ".join(failures))
        sys.exit(1)
    print("PASS")


if __name__ == "__main__":
    main(sys.argv[1])
