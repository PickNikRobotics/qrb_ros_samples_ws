"""Emit MJCF world geoms for the primitive collision shapes (box, cylinder, sphere) of one model in a Gazebo SDF.

Usage: python3 -I gen_sdf_collision_mjcf.py <file.sdf> <model name> <mjcf default class> <mjcf model name> > out.xml
  qrb_amr_sim:     gen_sdf_collision_mjcf.py warehouse.sdf depot_collision warehouse qrb_warehouse > warehouse.xml
  qrb_amr_nav_sim: gen_sdf_collision_mjcf.py "Ionic interior/model.sdf" ionic_interior ionic qrb_ionic > ionic.xml

Mesh and plane collisions are skipped; the scene supplies its own floor plane. SDF poses are x y z roll pitch yaw,
fixed-axis, which is MuJoCo's quaternion of Rz(yaw) * Ry(pitch) * Rx(roll). A collision's pose composes with its link's
and the model's, in the plane: the worlds converted so far rotate links and models only about z.
"""

import math
import re
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


def planar(parent, x, y, z, yaw):
    px, py, pz, pyaw = parent
    c, s = math.cos(pyaw), math.sin(pyaw)
    return px + c * x - s * y, py + s * x + c * y, pz + z, pyaw + yaw


def main(path, model_name, geom_class, mjcf_model):
    # Comments are dropped before parsing: some worlds carry non-ASCII attribution text the declared encoding rejects.
    with open(path, encoding="utf-8", errors="replace") as f:
        text = re.sub(r"<!--.*?-->", "", f.read(), flags=re.S)
    root = ET.fromstring(text[text.index("<sdf"):])
    model = next(m for m in root.iter("model") if m.get("name") == model_name)
    mx, my, mz, _, _, myaw = parse_pose(model.findtext("pose"))
    print(f'<mujoco model="{mjcf_model}">')
    print("  <worldbody>")
    for link in model.iter("link"):
        lx, ly, lz, _, _, lyaw = parse_pose(link.findtext("pose"))
        link_frame = planar((mx, my, mz, myaw), lx, ly, lz, lyaw)
        for c in link.findall("collision"):
            emit_collision(c, link_frame, geom_class)
    print("  </worldbody>")
    print("</mujoco>")


def emit_collision(c, link_frame, geom_class):
    cx, cy, cz, roll, pitch, cyaw = parse_pose(c.findtext("pose"))
    x, y, z, yaw = planar(link_frame, cx, cy, cz, cyaw)
    geom = list(c.find("geometry"))[0]
    pos = f"{x:.5f} {y:.5f} {z:.5f}"
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
        return
    print(f'    <geom name="{name}" class="{geom_class}" type="{geom.tag}" pos="{pos}"{rot} size="{size}"/>')


if __name__ == "__main__":
    main(*sys.argv[1:5])
