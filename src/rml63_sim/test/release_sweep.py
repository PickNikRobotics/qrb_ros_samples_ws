"""Debug aid: does the place -> re_raise motion sweep the open gripper through the released can?"""

import sys

import mujoco
import numpy as np

DEG = np.pi / 180
ARM = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
PLACE = np.array([-31, -42, -59, 0, 0, 0]) * DEG
RE_RAISE = np.array([-22, -35, -52, 0, 0, 0]) * DEG
CAN_BASE = np.array([3.066, -2.508, 1.314])
CAN_RADIUS, CAN_HEIGHT = 0.0299, 0.145


def main(scene):
    m = mujoco.MjModel.from_xml_path(scene)
    d = mujoco.MjData(m)
    hand_bodies = {m.body(n).id for n in ["Link00", "Link01", "Link11", "Link02", "Link22", "link7", "link8"]}
    geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] in hand_bodies and m.geom_contype[g] != 0]
    for t in np.linspace(0, 1, 11):
        q = PLACE + (RE_RAISE - PLACE) * t
        mujoco.mj_resetDataKeyframe(m, d, 0)
        for name, v in zip(ARM, q):
            d.joint(name).qpos[0] = v
        d.joint("joint01").qpos[0] = 0.0
        mujoco.mj_forward(m, d)
        # Closest approach of any gripper collision geom to the can's side, inside the can's height band.
        worst = np.inf
        for g in geoms:
            p = d.geom_xpos[g]
            if CAN_BASE[2] <= p[2] <= CAN_BASE[2] + CAN_HEIGHT + 0.03:
                worst = min(worst, np.linalg.norm(p[:2] - CAN_BASE[:2]) - CAN_RADIUS - m.geom_rbound[g])
        print(f"t={t:.1f} tcp {d.site('tcp').xpos.round(3)} min clearance to can side {worst:+.3f} m")


if __name__ == "__main__":
    main(sys.argv[1])
