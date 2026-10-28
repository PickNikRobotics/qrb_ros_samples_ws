"""Emit MJCF world geoms for the primitive collision shapes (box, cylinder, sphere) of one model in a Gazebo SDF.

Usage: python3 -I gen_sdf_collision_mjcf.py <file.sdf> <model name> <mjcf default class> <mjcf model name> > out.xml
  qrb_amr_sim:     gen_sdf_collision_mjcf.py warehouse.sdf depot_collision warehouse qrb_warehouse > warehouse.xml
  qrb_amr_nav_sim: gen_sdf_collision_mjcf.py "Ionic interior/model.sdf" ionic_interior ionic qrb_ionic > ionic.xml

Mesh and plane collisions are skipped; the scene supplies its own floor plane. SDF poses are x y z roll pitch yaw,
fixed-axis, which is MuJoCo's quaternion of Rz(yaw) * Ry(pitch) * Rx(roll).
"""

import math
import sys
import xml.etree.ElementTree as ET


def rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def parse_pose(text):
    return [float(v) for v in text.split()] if text else [0.0] * 6


def main(path, model_name, geom_class, mjcf_model):
    root = ET.parse(path).getroot()
    model = next(m for m in root.iter("model") if m.get("name") == model_name)
    mx, my, mz = parse_pose(model.findtext("pose"))[:3]
    print(f'<mujoco model="{mjcf_model}">')
    print("  <worldbody>")
    for c in model.iter("collision"):
        x, y, z, roll, pitch, yaw = parse_pose(c.findtext("pose"))
        geom = list(c.find("geometry"))[0]
        pos = f"{x + mx:.5f} {y + my:.5f} {z + mz:.5f}"
        if roll or pitch:
            rot = ' quat="{:.6f} {:.6f} {:.6f} {:.6f}"'.format(*rpy_to_quat(roll, pitch, yaw))
        else:
            rot = f' euler="0 0 {yaw:.6f}"' if yaw else ""
        name = c.get("name")
        if geom.tag == "box":
            sx, sy, sz = (float(v) / 2 for v in geom.findtext("size").split())
            size = f"{sx:.4f} {sy:.4f} {sz:.4f}"
        elif geom.tag == "cylinder":
            size = f'{float(geom.findtext("radius")):.4f} {float(geom.findtext("length")) / 2:.4f}'
        elif geom.tag == "sphere":
            size = f'{float(geom.findtext("radius")):.4f}'
        else:
            continue
        print(f'    <geom name="{name}" class="{geom_class}" type="{geom.tag}" pos="{pos}"{rot} size="{size}"/>')
    print("  </worldbody>")
    print("</mujoco>")


if __name__ == "__main__":
    main(*sys.argv[1:5])
