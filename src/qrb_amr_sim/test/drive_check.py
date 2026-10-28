"""Offline MuJoCo check of the QRB base's drive kinematics, before any controller is involved.

Commands the wheel speeds diff_drive_controller would send for Qualcomm's simple_motion speeds (0.5 m/s forward,
1.0 rad/s turn) and compares the base's measured motion against them. Wheel speeds use the controller's model:
  w_left/right = (v -/+ omega * separation / 2) / radius, separation = 0.33 * 1.1525 (Qualcomm's Gazebo multiplier).

Usage: python3 drive_check.py <scene.xml> [--separation-multiplier M]
Exits non-zero if the base does not settle on its wheels, or measured motion is off by more than 10%.
"""

import argparse
import sys

import mujoco
import numpy as np

RADIUS = 0.0655
SEPARATION = 0.33
FORWARD_V = 0.5
TURN_W = 1.0
SECONDS = 3.0
TOLERANCE = 0.10


def yaw(quat):
    w, x, y, z = quat
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def run(m, d, act, v, w, multiplier, seconds):
    sep = SEPARATION * multiplier
    d.ctrl[act["wheel_left_joint"]] = (v - w * sep / 2) / RADIUS
    d.ctrl[act["wheel_right_joint"]] = (v + w * sep / 2) / RADIUS
    start_pos = d.body("base_footprint").xpos.copy()
    # Accumulate yaw per step: a long turn rotates past pi, which a start/end difference would wrap.
    turned = 0.0
    last = yaw(d.body("base_footprint").xquat)
    for _ in range(int(seconds / m.opt.timestep)):
        mujoco.mj_step(m, d)
        now = yaw(d.body("base_footprint").xquat)
        turned += (now - last + np.pi) % (2 * np.pi) - np.pi
        last = now
    return d.body("base_footprint").xpos - start_pos, turned


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scene")
    parser.add_argument("--separation-multiplier", type=float, default=1.1525)
    args = parser.parse_args()
    m = mujoco.MjModel.from_xml_path(args.scene)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key("default").id)
    act = {m.actuator(i).name: i for i in range(m.nu)}

    for _ in range(int(1.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    z = d.body("base_footprint").xpos[2]
    tilt = 1.0 - d.body("base_footprint").xmat.reshape(3, 3)[2, 2]
    print(f"settled: base_footprint z {z:.4f} m, tilt {tilt:.5f}")
    failures = []
    if abs(z) > 0.01 or tilt > 1e-3:
        failures.append("base did not settle flat on its wheels")

    # Accelerate for 0.5 s before measuring so the steady-state speed is compared, not the ramp.
    run(m, d, act, FORWARD_V, 0.0, args.separation_multiplier, 0.5)
    moved, turned = run(m, d, act, FORWARD_V, 0.0, args.separation_multiplier, SECONDS)
    speed = np.linalg.norm(moved[:2]) / SECONDS
    print(f"forward: {speed:.3f} m/s (commanded {FORWARD_V}), heading drift {np.degrees(turned):+.2f} deg")
    if abs(speed - FORWARD_V) > TOLERANCE * FORWARD_V:
        failures.append(f"forward speed {speed:.3f} m/s")

    run(m, d, act, 0.0, 0.0, args.separation_multiplier, 1.0)
    run(m, d, act, 0.0, TURN_W, args.separation_multiplier, 0.5)
    moved, turned = run(m, d, act, 0.0, TURN_W, args.separation_multiplier, SECONDS)
    rate = turned / SECONDS
    print(f"turn: {rate:.3f} rad/s (commanded {TURN_W}), drift {np.linalg.norm(moved[:2]):.3f} m")
    if abs(rate - TURN_W) > TOLERANCE * TURN_W:
        failures.append(f"turn rate {rate:.3f} rad/s")

    if failures:
        print("FAIL:", "; ".join(failures))
        sys.exit(1)
    print("PASS")


if __name__ == "__main__":
    main()
