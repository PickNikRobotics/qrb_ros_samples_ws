"""Emit MJCF geoms for the static collision boxes in Qualcomm's warehouse.sdf.

Usage: python3 -I gen_warehouse_mjcf.py <warehouse.sdf> > warehouse.xml
"""

import sys
import xml.etree.ElementTree as ET


def main(path):
    root = ET.parse(path).getroot()
    model = next(m for m in root.iter("model") if m.get("name") == "depot_collision")
    mx, my, mz = (float(v) for v in model.findtext("pose").split()[:3])
    print("<mujoco model=\"qrb_warehouse\">")
    print("  <worldbody>")
    for c in model.iter("collision"):
        x, y, z, _, _, yaw = (float(v) for v in c.findtext("pose").split())
        geom = list(c.find("geometry"))[0]
        pos = f"{x + mx:.5f} {y + my:.5f} {z + mz:.5f}"
        euler = f' euler="0 0 {yaw:.6f}"' if yaw else ""
        name = c.get("name")
        if geom.tag == "box":
            sx, sy, sz = (float(v) / 2 for v in geom.findtext("size").split())
            print(f'    <geom name="{name}" class="warehouse" type="box" pos="{pos}"{euler} size="{sx:.4f} {sy:.4f} {sz:.4f}"/>')
        elif geom.tag == "cylinder":
            r = float(geom.findtext("radius"))
            h = float(geom.findtext("length")) / 2
            print(f'    <geom name="{name}" class="warehouse" type="cylinder" pos="{pos}" size="{r:.4f} {h:.4f}"/>')
    print("  </worldbody>")
    print("</mujoco>")


if __name__ == "__main__":
    main(sys.argv[1])
