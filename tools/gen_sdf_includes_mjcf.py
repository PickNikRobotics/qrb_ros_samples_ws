"""Emit MJCF world geoms for the furniture and people a Gazebo world places with Fuel <include>s.

Usage: python3 -I gen_sdf_includes_mjcf.py <world.sdf> <fuel model dir> <mjcf model name> > out.xml
  qrb_remote_assistant_sim: gen_sdf_includes_mjcf.py office.sdf <dir with one unzipped Fuel model per subdirectory,
                            named after the URI's last path segment with spaces as underscores> qrb_office_furniture

Every <include> is flattened to its world pose through its enclosing <model> poses. A model whose visual is an OBJ
becomes a textured mesh geom that is drawn but not collided, plus its own collision mesh (the model's low-poly
*_Col.obj, or its visual when it has none) as an undrawn geom, so lidar, costmaps, and contact see what Gazebo did. A
model whose visual is COLLADA, which MuJoCo cannot load, becomes its own SDF primitive colliders, drawn plain. Nested <model> and
<include> poses here are planar (x y z and yaw), which this assumes and checks; a primitive collider inside a model may
be rotated any way.
"""

import math
import pathlib
import re
import sys
import xml.etree.ElementTree as ET

# Shared with the collision generator beside this file; python3 -I leaves the script's directory off sys.path.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from gen_sdf_collision_mjcf import rpy_to_quat  # noqa: E402


def parse_full_pose(text):
    return [float(v) for v in text.split()] if text and text.strip() else [0.0] * 6


def parse_pose(text):
    values = parse_full_pose(text)
    if abs(values[3]) > 1e-6 or abs(values[4]) > 1e-6:
        raise SystemExit(f"pose {text!r} has roll or pitch; only planar poses are supported")
    return values[0], values[1], values[2], values[5]


def compose(parent, child):
    px, py, pz, pyaw = parent
    cx, cy, cz, cyaw = child
    c, s = math.cos(pyaw), math.sin(pyaw)
    return px + c * cx - s * cy, py + s * cx + c * cy, pz + cz, pyaw + cyaw


def walk(model, parent, out):
    frame = compose(parent, parse_pose(model.findtext("pose")))
    for include in model.findall("include"):
        name = include.findtext("uri").strip().rstrip("/").split("/")[-1].replace(" ", "_")
        out.append((model.get("name"), name, compose(frame, parse_pose(include.findtext("pose")))))
    for child in model.findall("model"):
        walk(child, frame, out)


def mtl_textures(obj_path):
    """Map each usemtl material in an OBJ to its diffuse texture file, or None, largest material (by faces) first."""
    obj_text = obj_path.read_text(errors="ignore")
    mtllib = re.search(r"^mtllib\s+(\S+)", obj_text, re.M)
    textures = {}
    if mtllib:
        current = None
        for line in (obj_path.parent / mtllib.group(1)).read_text(errors="ignore").splitlines():
            parts = line.split()
            if parts[:1] == ["newmtl"]:
                current = parts[1]
                textures[current] = None
            elif parts[:1] == ["map_Kd"] and current:
                textures[current] = parts[-1]
    faces, current = {}, None
    for line in obj_text.splitlines():
        if line.startswith("usemtl"):
            current = line.split()[1]
        elif line.startswith("f "):
            faces[current] = faces.get(current, 0) + 1
    return dict(sorted(textures.items(), key=lambda item: -faces.get(item[0], 0)))


def find_texture(model_dir, mesh_dir, texture):
    """Resolve an MTL map_Kd path: relative to the mesh, else anywhere in the model (Fuel models disagree)."""
    direct = (mesh_dir / texture).resolve()
    if direct.is_file():
        return direct
    found = sorted(model_dir.rglob(pathlib.Path(texture).name))
    return found[0].resolve() if found else None


def model_visual(model_dir):
    """Return ('obj', mesh path, scale, local pose) or ('primitives', [collision elements])."""
    sdf = ET.parse(model_dir / "model.sdf").getroot()
    link = sdf.find("model/link")
    link_pose = parse_pose(link.findtext("pose"))
    model_pose = parse_pose(sdf.find("model").findtext("pose"))
    base = compose(model_pose, link_pose)
    visual = link.find("visual")
    uri = visual.findtext("geometry/mesh/uri").strip()
    if uri.lower().endswith(".obj"):
        scale = visual.findtext("geometry/mesh/scale") or "1 1 1"
        collision_uri = link.findtext("collision/geometry/mesh/uri")
        collision_obj = model_dir / "meshes" / pathlib.Path(collision_uri.strip()).name if collision_uri else None
        local = compose(base, parse_pose(visual.findtext("pose")))
        return "obj", model_dir / "meshes" / pathlib.Path(uri).name, collision_obj, scale, local
    return "primitives", base, [c for c in link.findall("collision") if list(c.find("geometry"))[0].tag in ("box", "cylinder")]


def main(world_path, fuel_dir, mjcf_model):
    text = re.sub(r"<!--.*?-->", "", pathlib.Path(world_path).read_text(), flags=re.S)
    world = ET.fromstring(text[text.index("<sdf"):]).find("world")
    placements = []
    for model in world.findall("model"):
        walk(model, (0.0, 0.0, 0.0, 0.0), placements)

    assets, geoms, meshes_done = [], [], set()
    for index, (group, name, (x, y, z, yaw)) in enumerate(placements):
        model_dir = pathlib.Path(fuel_dir) / name
        if not model_dir.is_dir():
            raise SystemExit(f"Fuel model {name} (in {group}) is not in {fuel_dir}")
        kind, *rest = model_visual(model_dir)
        geom_name = f"{name.lower()}_{index}"
        if kind == "obj":
            obj, collision_obj, scale, local = rest
            wx, wy, wz, wyaw = compose((x, y, z, yaw), local)
            pose = f'pos="{wx:.4f} {wy:.4f} {wz:.4f}" euler="0 0 {wyaw:.5f}"'
            textures = mtl_textures(obj) or {None: None}
            for material, texture in textures.items():
                part = f"{name}_{material}" if material else name
                if part not in meshes_done:
                    meshes_done.add(part)
                    # MuJoCo loads an OBJ as one mesh with one material, so the texture of the material covering the
                    # most faces is applied (a person's clothes, not the book they hold) and the rest are dropped.
                    texture_path = find_texture(model_dir, obj.parent, texture) if texture else None
                    if texture_path and f"{name}_tex" not in meshes_done:
                        meshes_done.add(f"{name}_tex")
                        file = texture_path.relative_to(model_dir.parent).as_posix()
                        assets.append(f'    <texture name="{name}" type="2d" file="{file}" />')
                        assets.append(f'    <material name="{name}" texture="{name}" />')
                    if f"{name}_mesh" not in meshes_done:
                        meshes_done.add(f"{name}_mesh")
                        assets.append(f'    <mesh name="{name}" file="{name}/meshes/{obj.name}" scale="{scale}" />')
            material = f' material="{name}"' if f"{name}_tex" in meshes_done else ' rgba="0.6 0.6 0.6 1"'
            geoms.append(f'    <geom name="{geom_name}" class="furniture" type="mesh" mesh="{name}"{material} {pose} />')
            if collision_obj and collision_obj.is_file():
                if f"{name}_col" not in meshes_done:
                    meshes_done.add(f"{name}_col")
                    file = collision_obj.relative_to(model_dir.parent).as_posix()
                    assets.append(f'    <mesh name="{name}_col" file="{file}" scale="{scale}" />')
                geoms.append(
                    f'    <geom name="{geom_name}_col" class="furniture_collision" type="mesh" mesh="{name}_col" {pose} />'
                )
        else:
            base, collisions = rest
            frame = compose((x, y, z, yaw), base)
            for part, collision in enumerate(collisions):
                lx, ly, lz, roll, pitch, lyaw = parse_full_pose(collision.findtext("pose"))
                cx, cy, cz, cyaw = compose(frame, (lx, ly, lz, lyaw))
                rot = ' quat="{:.6f} {:.6f} {:.6f} {:.6f}"'.format(*rpy_to_quat(roll, pitch, cyaw))
                shape = list(collision.find("geometry"))[0]
                if shape.tag == "box":
                    sx, sy, sz = (float(v) / 2 for v in shape.findtext("size").split())
                    size = f"{sx:.4f} {sy:.4f} {sz:.4f}"
                else:
                    size = f'{float(shape.findtext("radius")):.4f} {float(shape.findtext("length")) / 2:.4f}'
                geoms.append(
                    f'    <geom name="{geom_name}_{part}" class="furniture" type="{shape.tag}" '
                    f'pos="{cx:.4f} {cy:.4f} {cz:.4f}"{rot} size="{size}" rgba="0.55 0.45 0.35 1" />'
                )

    print(f'<mujoco model="{mjcf_model}">')
    print("  <asset>")
    print("\n".join(assets))
    print("  </asset>")
    print("  <worldbody>")
    print("\n".join(geoms))
    print("  </worldbody>")
    print("</mujoco>")


if __name__ == "__main__":
    main(*sys.argv[1:4])
