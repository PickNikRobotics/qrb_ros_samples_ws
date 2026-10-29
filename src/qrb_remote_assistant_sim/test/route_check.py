"""Offline check of Qualcomm's build_map_node route in the qrb_remote_assistant_sim office scene, no ROS.

Drives the node's step list at its speeds (0.8 m/s, 30 deg/s) from the spawn and reports where each step ends and any
contact between the base and the room. --close-on odom closes each step on wheel odometry computed with the drive
controller's parameters, as build_map_node does on /odom; --close-on truth closes it on the base's true pose.
Usage: python3 route_check.py description/scene.xml [--close-on odom|truth]
"""

import math
import sys

import mujoco
import numpy as np

WHEEL_RADIUS = 0.053
# diff_drive_controller's model of the base (qrb_robot_base_mini.ros2_control.yaml): 0.25 m times the 1.05 calibration.
WHEEL_SEPARATION = 0.25 * 1.05
LINEAR, ANGULAR = 0.8, math.radians(30)
# build_map_node.py's steps, unchanged.
STEPS = [("FORWARD", 4.0), ("TURN", -90), ("FORWARD", 7.0), ("TURN", -90), ("TURN", -90), ("FORWARD", 7.0),
         ("TURN", 90), ("FORWARD", 3.3), ("TURN", -90), ("FORWARD", 1.5)]


def wheel_ctrl(v, w):
    return [(v - w * WHEEL_SEPARATION / 2) / WHEEL_RADIUS, (v + w * WHEEL_SEPARATION / 2) / WHEEL_RADIUS]


def yaw(d):
    w, x, y, z = d.qpos[3:7]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class WheelOdometry:
    """Integrates the wheel joint angles the way diff_drive_controller does, from its wheel parameters."""

    def __init__(self, m, d):
        self._qpos = [m.jnt_qposadr[m.joint(n).id] for n in ("wheel_left_joint", "wheel_right_joint")]
        self._last = [d.qpos[a] for a in self._qpos]
        self.x, self.y, self.yaw = d.qpos[0], d.qpos[1], yaw(d)

    def update(self, d):
        now = [d.qpos[a] for a in self._qpos]
        left, right = ((n - p) * WHEEL_RADIUS for n, p in zip(now, self._last))
        self._last = now
        distance, turn = (left + right) / 2, (right - left) / WHEEL_SEPARATION
        self.x += distance * math.cos(self.yaw + turn / 2)
        self.y += distance * math.sin(self.yaw + turn / 2)
        self.yaw += turn


def main(path, close_on):
    m = mujoco.MjModel.from_xml_path(path)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key("default").id)
    robot = {b for b in range(m.nbody) if m.body_rootid[b] == m.body("base_footprint").id}
    contacts = {}
    for _ in range(int(1.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    odom = WheelOdometry(m, d)

    def pose():
        return (np.array([odom.x, odom.y]), odom.yaw) if close_on == "odom" else (d.qpos[:2].copy(), yaw(d))

    for action, param in STEPS:
        (start, start_yaw), turned = pose(), 0.0
        prev = start_yaw
        for _ in range(int(30 / m.opt.timestep)):
            if action == "FORWARD":
                if np.linalg.norm(pose()[0] - start) >= param:
                    break
                d.ctrl[:] = wheel_ctrl(LINEAR, 0.0)
            else:
                if abs(turned) >= math.radians(abs(param)):
                    break
                d.ctrl[:] = wheel_ctrl(0.0, math.copysign(ANGULAR, param))
            mujoco.mj_step(m, d)
            odom.update(d)
            now = pose()[1]
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
            odom.update(d)
        print(f"{action:7s} {param:+5}: ends at ({d.qpos[0]:.2f}, {d.qpos[1]:.2f}) yaw {math.degrees(yaw(d)):+.0f} deg")
    print("contacts with the room:", contacts or "none")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[3] if len(sys.argv) > 3 and sys.argv[2] == "--close-on" else "truth")
