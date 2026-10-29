"""Offline check of Qualcomm's build_map_node route in the qrb_remote_assistant_sim office scene, no ROS.

Drives the node's step list at its speeds (0.8 m/s, 30 deg/s) from the spawn, closing each step on the base's true
pose the way the node closes it on /odom, and reports where each step ends and any contact between the base and the
room. Usage: python3 route_check.py description/scene.xml
"""

import math
import sys

import mujoco
import numpy as np

WHEEL_RADIUS = 0.053
WHEEL_SEPARATION = 0.25 * 1.125
LINEAR, ANGULAR = 0.8, math.radians(30)
# build_map_node.py's steps, unchanged.
STEPS = [("FORWARD", 4.0), ("TURN", -90), ("FORWARD", 7.0), ("TURN", -90), ("TURN", -90), ("FORWARD", 7.0),
         ("TURN", 90), ("FORWARD", 3.3), ("TURN", -90), ("FORWARD", 1.5)]


def wheel_ctrl(v, w):
    return [(v - w * WHEEL_SEPARATION / 2) / WHEEL_RADIUS, (v + w * WHEEL_SEPARATION / 2) / WHEEL_RADIUS]


def yaw(d):
    w, x, y, z = d.qpos[3:7]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def main(path):
    m = mujoco.MjModel.from_xml_path(path)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key("default").id)
    robot = {b for b in range(m.nbody) if m.body_rootid[b] == m.body("base_footprint").id}
    contacts = {}
    for _ in range(int(1.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    for action, param in STEPS:
        start, start_yaw, turned, prev = d.qpos[:2].copy(), yaw(d), 0.0, yaw(d)
        for _ in range(int(30 / m.opt.timestep)):
            if action == "FORWARD":
                if np.linalg.norm(d.qpos[:2] - start) >= param:
                    break
                d.ctrl[:] = wheel_ctrl(LINEAR, 0.0)
            else:
                if abs(turned) >= math.radians(abs(param)):
                    break
                d.ctrl[:] = wheel_ctrl(0.0, math.copysign(ANGULAR, param))
            mujoco.mj_step(m, d)
            now = yaw(d)
            turned += (now - prev + math.pi) % (2 * math.pi) - math.pi
            prev = now
            for c in d.contact[: d.ncon]:
                b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
                other = c.geom2 if b1 in robot and b2 not in robot else c.geom1 if b2 in robot and b1 not in robot else None
                if other is not None and m.geom(other).name != "floor":
                    contacts.setdefault(m.geom(other).name, d.time)
        d.ctrl[:] = 0
        for _ in range(int(0.5 / m.opt.timestep)):
            mujoco.mj_step(m, d)
        print(f"{action:7s} {param:+5}: ends at ({d.qpos[0]:.2f}, {d.qpos[1]:.2f}) yaw {math.degrees(yaw(d)):+.0f} deg")
    print("contacts with the room:", contacts or "none")


if __name__ == "__main__":
    main(sys.argv[1])
