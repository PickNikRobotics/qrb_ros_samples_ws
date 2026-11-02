"""Offline MuJoCo grasp check for Qualcomm's pick-and-place joint targets.

Drives the position actuators through the sample's steps (ready, open, pick, close, raise[, place, open, re-raise])
and checks the can. Joint values are copied verbatim from qrb_ros_arm_pick_place.cpp and rml_63.srdf.

Usage: python3 grasp_check.py <scene.xml> [--full]
  default: pick -> close -> raise; pass if coke1 rises >= 0.05 m.
  --full:  complete sequence; pass if coke1 ends upright on top of the cardboard box, within 0.05 m (xy) of the release point.
"""

import argparse
import sys

import mujoco
import numpy as np

ARM = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
HAND = ["joint01", "joint11", "joint02", "joint22"]
DEG = np.pi / 180.0

READY = [0.0, 0.0, -2.5, 0.0, 1.0, 0.0]
PICK = [0.0, -46.0 * DEG, -114.0 * DEG, 0.0, 74.0 * DEG, 0.0]
RAISE = [-22.0 * DEG, -37.0 * DEG, -48.0 * DEG, 0.0, 0.0, 0.0]
PLACE = [-31.0 * DEG, -42.0 * DEG, -59.0 * DEG, 0.0, 0.0, 0.0]
RE_RAISE = [-22.0 * DEG, -35.0 * DEG, -52.0 * DEG, 0.0, 0.0, 0.0]
OPEN = [0.0, -0.25, 0.0, 0.25]
CLOSE = [0.2, 0.0, -0.2, 0.0]
# coke1 body origin is the can base; a held can sits within one can height of the tcp site.
MAX_HELD_OFFSET = 0.15
# Paired fingers must stay mirrored while the gripper opens over the box; asymmetry flicks the can over on release.
MAX_FINGER_ASYMMETRY = 0.02
RELEASE_REPEATS = 8
BOX_HALF_HEIGHT = 0.15


class Sim:
    def __init__(self, scene):
        self.m = mujoco.MjModel.from_xml_path(scene)
        self.d = mujoco.MjData(self.m)
        mujoco.mj_resetDataKeyframe(self.m, self.d, self.m.key("default").id)
        self.act = {self.m.joint(self.m.actuator(i).trnid[0]).name: i for i in range(self.m.nu)}

    def joints(self, names):
        return np.array([self.d.joint(n).qpos[0] for n in names])

    def move(self, names, goal, duration=2.0, settle=0.5):
        """Linear joint interpolation, standing in for a JointTrajectoryController."""
        start = self.joints(names)
        steps = int(duration / self.m.opt.timestep)
        for k in range(1, steps + 1):
            q = start + (np.array(goal) - start) * k / steps
            for n, v in zip(names, q):
                # Coupled gripper: only joint01 has an actuator; the other fingers follow it.
                if n in self.act:
                    self.d.ctrl[self.act[n]] = v
            mujoco.mj_step(self.m, self.d)
        for _ in range(int(settle / self.m.opt.timestep)):
            mujoco.mj_step(self.m, self.d)

    def coke(self):
        return self.d.body("coke1").xpos.copy()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scene")
    parser.add_argument("--full", action="store_true")
    parser.add_argument("--release", action="store_true", help="repeat pick/place and require upright landings")
    args = parser.parse_args()

    if args.release:
        release_gate(args.scene)
        return

    sim = Sim(args.scene)
    sim.move(ARM, READY)
    sim.move(HAND, OPEN, duration=1.0)
    sim.move(ARM, PICK)
    before = sim.coke()
    sim.move(HAND, CLOSE, duration=1.0, settle=1.0)
    sim.move(ARM, RAISE)
    lifted = sim.coke()
    offset = np.linalg.norm(lifted - sim.d.site("tcp").xpos)
    print(f"coke1 at pick {before.round(3)}, after raise {lifted.round(3)}, can-tcp {offset:.3f} m, hand {sim.joints(HAND).round(3)}")
    if lifted[2] - before[2] < 0.05:
        print(f"FAIL: can rose {lifted[2] - before[2]:.3f} m (< 0.05)")
        sys.exit(1)
    # Held, not knocked: the can must ride with the gripper.
    if offset > MAX_HELD_OFFSET:
        print(f"FAIL: can is {offset:.3f} m from the tcp (> {MAX_HELD_OFFSET}); not held")
        sys.exit(1)
    if not args.full:
        print("PASS")
        return

    sim.move(ARM, PLACE)
    release = sim.d.site("tcp").xpos.copy()
    sim.move(HAND, OPEN, duration=1.0, settle=1.5)
    sim.move(ARM, RE_RAISE)
    sim.move(ARM, READY)
    final = sim.coke()
    print(f"release tcp {release.round(3)}, coke1 final {final.round(3)}")
    box = sim.d.body("cardboard_box")
    box_top = box.xpos[2] + BOX_HALF_HEIGHT
    upright = sim.d.body("coke1").xmat.reshape(3, 3)[2, 2]
    if np.linalg.norm(final[:2] - release[:2]) > 0.05:
        print(f"FAIL: can landed {np.linalg.norm(final[:2] - release[:2]):.3f} m from the release point")
        sys.exit(1)
    if abs(final[2] - box_top) > 0.01 or upright < 0.95:
        print(f"FAIL: can not resting upright on the box (z {final[2]:.3f} vs top {box_top:.3f}, up {upright:.2f})")
        sys.exit(1)
    print("PASS")


def release_gate(scene):
    """Pick and place RELEASE_REPEATS times with perturbed gripper timing; every can must land upright."""
    for run in range(RELEASE_REPEATS):
        sim = Sim(scene)
        sim.move(ARM, READY)
        sim.move(HAND, OPEN, duration=1.0)
        sim.move(ARM, PICK)
        sim.move(HAND, CLOSE, duration=1.0, settle=1.0)
        sim.move(ARM, RAISE)
        sim.move(ARM, PLACE)
        worst = 0.0
        # Vary the open duration so the release does not land on one lucky contact timing.
        steps = int((0.6 + 0.1 * run) / sim.m.opt.timestep)
        start = sim.joints(HAND)
        for k in range(1, steps + 1):
            q = start + (np.array(OPEN) - start) * k / steps
            for n, v in zip(HAND, q):
                if n in sim.act:
                    sim.d.ctrl[sim.act[n]] = v
            mujoco.mj_step(sim.m, sim.d)
            j = sim.joints(HAND)
            worst = max(worst, abs(j[0] + j[2]), abs(j[1] + j[3]))
        sim.move(ARM, RE_RAISE)
        up = sim.d.body("coke1").xmat.reshape(3, 3)[2, 2]
        print(f"run {run}: finger asymmetry {worst:.3f} rad, coke1 up {up:.2f}, z {sim.coke()[2]:.3f}")
        if worst > MAX_FINGER_ASYMMETRY or up < 0.95:
            print("FAIL: asymmetric release or can tipped")
            sys.exit(1)
    print("PASS")


if __name__ == "__main__":
    main()
