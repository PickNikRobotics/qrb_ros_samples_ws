"""Offline check of the qrb_remote_assistant_sim office scene in Python MuJoCo, no ROS.

Settles the base, drives it at Qualcomm's 0.5 m/s and 1.0 rad/s through the wheel actuators, reads the 1800-beam
lidar, and times the simulation. Usage: python3 test/offline_check.py description/scene.xml
"""

import sys
import time

import mujoco
import numpy as np

WHEEL_RADIUS = 0.053
WHEEL_SEPARATION = 0.25 * 1.125


def wheel_ctrl(v, w):
    return [(v - w * WHEEL_SEPARATION / 2) / WHEEL_RADIUS, (v + w * WHEEL_SEPARATION / 2) / WHEEL_RADIUS]


def yaw(d):
    w, x, y, z = d.qpos[3:7]
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def run(m, d, ctrl, seconds):
    d.ctrl[:] = ctrl
    for _ in range(int(seconds / m.opt.timestep)):
        mujoco.mj_step(m, d)


m = mujoco.MjModel.from_xml_path(sys.argv[1])
d = mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m, d, m.key("default").id)
run(m, d, [0, 0], 1.0)
print(f"settled at x={d.qpos[0]:.3f} y={d.qpos[1]:.3f} z={d.qpos[2]:.4f} tilt_quat_xy={np.abs(d.qpos[4:6]).max():.4f}")

scan = d.sensordata[:1800]
print(f"lidar beams={scan.size} min={scan[scan > 0].min():.3f} max={scan.max():.3f} no-hit={(scan < 0).sum()}")
print(f"lidar beam 0 (rear, -179 deg)={scan[0]:.3f} beam 900 (forward)={scan[900]:.3f}")

x0 = d.qpos[0]
run(m, d, wheel_ctrl(0.5, 0), 1.0)
x1, t1 = d.qpos[0], d.time
run(m, d, wheel_ctrl(0.5, 0), 2.0)
print(f"forward v={(d.qpos[0] - x1) / (d.time - t1):.3f} m/s (target 0.5)")
run(m, d, [0, 0], 1.0)
run(m, d, wheel_ctrl(0, 1.0), 1.0)
y0, t0 = np.unwrap([yaw(d)])[0], d.time
samples = []
for _ in range(20):
    run(m, d, wheel_ctrl(0, 1.0), 0.1)
    samples.append(yaw(d))
print(f"turn w={(np.unwrap([y0] + samples)[-1] - y0) / (d.time - t0):.3f} rad/s (target 1.0)")

start, sim0 = time.perf_counter(), d.time
run(m, d, [0, 0], 5.0)
print(f"real-time factor {(d.time - sim0) / (time.perf_counter() - start):.1f}x (physics only, 1800 rays per step)")
