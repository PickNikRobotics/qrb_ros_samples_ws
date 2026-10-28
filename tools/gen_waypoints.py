"""Write rml63_sim waypoints from Qualcomm's joint targets.

Arm values: qrb_ros_arm_pick_place.cpp get*JointAngles() (degrees) and rml_63.srdf "ready".
Gripper values: rml_63.srdf "open" / "close".
Usage: python3 -I gen_waypoints.py > rml63_waypoints.yaml
"""

import math

import yaml

ARM = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
HAND = ["joint01", "joint11", "joint02", "joint22"]
DEG = math.pi / 180.0

WAYPOINTS = [
    ("ready", "rm_group_controller", ARM, [0.0, 0.0, -2.5, 0.0, 1.0, 0.0]),
    ("pick", "rm_group_controller", ARM, [0.0, -46 * DEG, -114 * DEG, 0.0, 74 * DEG, 0.0]),
    ("raise", "rm_group_controller", ARM, [-22 * DEG, -37 * DEG, -48 * DEG, 0.0, 0.0, 0.0]),
    ("place", "rm_group_controller", ARM, [-31 * DEG, -42 * DEG, -59 * DEG, 0.0, 0.0, 0.0]),
    ("re_raise", "rm_group_controller", ARM, [-22 * DEG, -35 * DEG, -52 * DEG, 0.0, 0.0, 0.0]),
    ("open", "hand_controller", HAND, [0.0, -0.25, 0.0, 0.25]),
    ("close", "hand_controller", HAND, [0.2, 0.0, -0.2, 0.0]),
]


def entry(name, group, joints, positions):
    stamp = {"sec": 0, "nanosec": 0}
    return {
        "description": "From Qualcomm simulation_sample_pick_and_place",
        "favorite": False,
        "joint_group_names": [group],
        "joint_state": {
            "effort": [],
            "header": {"frame_id": "", "stamp": stamp},
            "name": joints,
            "position": positions,
            "velocity": [],
        },
        "multi_dof_joint_state": {
            "header": {"frame_id": "", "stamp": stamp},
            "joint_names": [],
            "transforms": [],
            "twist": [],
            "wrench": [],
        },
        "name": name,
    }


print(yaml.safe_dump([entry(*w) for w in WAYPOINTS], sort_keys=True), end="")
