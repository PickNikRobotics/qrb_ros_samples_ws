"""Post-process urdf_to_mjcf output for Qualcomm's QRB robot base into description/qrb_robot_base.xml.

Usage: python3 -I postprocess_qrb_amr_mjcf.py <robot_description.xml> <out.xml> [model name, default qrb_robot_base]

Used for both QRB bases (qrb_amr_sim: qrb_robot_base, qrb_amr_nav_sim: qrb_robot_base_mini); they share joint names.

A mobile base is not a fixed arm, so the generated arm defaults are undone: base_footprint gets a free joint, gravity
compensation is removed (the base must rest on its wheels), wheel position actuators become velocity actuators,
and the passive casters become frictionless spheres with no actuator.
"""

import sys
import xml.etree.ElementTree as ET

DRIVE_WHEELS = {"wheel_left_joint", "wheel_right_joint"}
# Velocity actuators: tau = armature / kv must stay below the timestep (2 ms in scene.xml).
WHEEL_KV = "50"
WHEEL_ARMATURE = "0.05"
# wheel_radius 0.0655 m (mini: 0.053 m), so +-30 rad/s covers 1.6 to 2 m/s, over 3x the samples' 0.5 m/s.
WHEEL_CTRLRANGE = "-30 30"


def main(src, dst, model="qrb_robot_base"):
    tree = ET.parse(src)
    root = tree.getroot()
    root.set("model", model)
    root.find("compiler").set("meshdir", "assets")

    for body in root.iter("body"):
        body.attrib.pop("gravcomp", None)
    footprint = root.find("worldbody/body[@name='base_footprint']")
    footprint.insert(0, ET.Element("freejoint", name="base_footprint"))

    actuators = root.find("actuator")
    for act in list(actuators):
        actuators.remove(act)
    for name in sorted(DRIVE_WHEELS):
        ET.SubElement(actuators, "velocity", name=name, joint=name, kv=WHEEL_KV, ctrlrange=WHEEL_CTRLRANGE)

    for joint in root.iter("joint"):
        name = joint.get("name")
        if name in DRIVE_WHEELS:
            joint.set("armature", WHEEL_ARMATURE)
            joint.set("damping", "0.01")
        elif name and name.startswith("caster_wheel_joint"):
            # Passive: a ball joint lets the sphere roll any direction; the URDF's diagonal hinge would drag.
            joint.set("type", "ball")
            joint.attrib.pop("axis", None)

    for geom in root.iter("geom"):
        if geom.get("contype") == "0":
            continue
        name = geom.get("name", "")
        if name.startswith("caster_wheel"):
            geom.set("friction", "0 0 0")
            geom.set("condim", "1")
        elif name.startswith("wheel_"):
            geom.set("friction", "1.2 0.01 0.001")
            geom.set("condim", "6")

    ET.indent(tree)
    tree.write(dst)


if __name__ == "__main__":
    main(*sys.argv[1:4])
