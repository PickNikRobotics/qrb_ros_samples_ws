"""Add Qualcomm's base RGB camera to the QRB robot base mini MJCF, for qrb_remote_assistant_sim.

Usage: python3 -I add_base_camera.py <qrb_amr_nav_sim/description/qrb_robot_base_mini.xml> <out.xml>

Camera from qrb_ros_sim_gazebo config/params/rgb_camera_params.yaml: 1280x720 and a horizontal FOV of 1.640609 rad,
so a vertical FOV of 2 * atan(tan(1.640609 / 2) * 720 / 1280) = 62.1974 deg, on base_camera_link. MuJoCo cameras look
along local -Z with +Y up and ROS links are +X forward, +Z up; the optical-frame site is ROS's +Z forward, +Y down.
user="1" makes it RGB only; Qualcomm's sample runs with the depth camera off.
"""

import sys
import xml.etree.ElementTree as ET


def main(src, dst):
    tree = ET.parse(src)
    mount = tree.getroot().find(".//body[@name='base_camera_link']")
    camera = ET.Element(
        "camera", name="camera", mode="fixed", resolution="1280 720", fovy="62.1974", user="1", xyaxes="0 -1 0 0 0 1"
    )
    site = ET.Element("site", name="camera_optical_frame", xyaxes="0 -1 0 0 0 -1", group="4")
    index = list(mount).index(mount.find("body"))
    mount.insert(index, site)
    mount.insert(index, camera)
    ET.indent(tree)
    tree.write(dst)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
