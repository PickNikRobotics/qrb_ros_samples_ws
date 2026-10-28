"""Post-process urdf_to_mjcf output into description/rml63.xml.

Usage: python3 -I postprocess_rml63_mjcf.py <robot_description.xml> <out rml63.xml>

Places the arm at Qualcomm's Gazebo spawn pose, points meshes at assets/,
splits arm and gripper actuator gains, and adds a fingertip site for tests.
"""

import sys
import xml.etree.ElementTree as ET

# Qualcomm README spawn: initial_x:=2.2 initial_y:=-2 initial_z:=1.025 initial_yaw:=3.14159
BASE_POS = "2.2 -2 1.025"
BASE_QUAT = "0 0 0 1"  # yaw = pi
ARM_JOINTS = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"}
GRIPPER_JOINTS = {"joint01", "joint11", "joint02", "joint22"}


def main(src, dst):
    tree = ET.parse(src)
    root = tree.getroot()
    root.set("model", "rml63")
    compiler = root.find("compiler")
    compiler.set("meshdir", "assets")
    compiler.set("autolimits", "true")

    for act in root.find("actuator"):
        joint = act.get("joint")
        if joint in ARM_JOINTS:
            act.set("kp", "2000")
            act.set("dampratio", "1")
        elif joint in GRIPPER_JOINTS:
            act.set("kp", "50")
            act.set("dampratio", "1")
            act.set("forcerange", "-5 5")
            act.set("forcelimited", "true")
        # ctrlrange follows the joint range so commands outside it saturate cleanly.
        act.set("ctrllimited", "false")
        act.attrib.pop("ctrlrange", None)

    base = root.find("worldbody/body[@name='arm_base_link']")
    base.set("pos", BASE_POS)
    base.set("quat", BASE_QUAT)

    link8 = root.find(".//body[@name='link8']")
    ET.SubElement(link8, "site", name="tcp", pos="0 0 0.12", size="0.005", rgba="1 0 0 1")

    for joint in root.iter("joint"):
        if joint.get("name") in GRIPPER_JOINTS:
            joint.set("damping", "0.5")
            joint.set("armature", "0.001")

    for geom in root.iter("geom"):
        if geom.get("contype") != "0":
            geom.set("friction", "1.5 0.02 0.002")

    ET.indent(tree)
    tree.write(dst)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
