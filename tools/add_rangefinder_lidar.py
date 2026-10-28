"""Add a planar rangefinder lidar to a MuJoCo robot MJCF, in the form picknik_mujoco_ros publishes as a LaserScan.

Usage: python3 -I add_rangefinder_lidar.py <robot.xml> --body laser --name laser_scan --samples 1800 \
         --angle-min -3.124139 --angle-max 3.14159 --range-min 0.15 --range-max 25 --noise 0.01 \
         --exclude-geom wheelTec_AMR_mini

One site and one <rangefinder> per beam, named <name>-0000 onward, written out rather than with <replicate>, which
MuJoCo 3.6 rejects in a model that has a keyframe. Beam i points along the body's x-y plane at
angle_min + i * increment, with increment = (angle_max - angle_min) / (samples - 1) as Gazebo's gpu_lidar computes it.
picknik_mujoco_ros expresses the scan in <name>_ROS, whose x axis points at beam 0, so the published LaserScan has
angle_min 0 and TF carries the rotation to the body frame.

Every rangefinder casts its ray on every physics step whatever lidar_publish_rate is (MuJoCo 3.6 does not honor the
sensor interval attribute for them), so the beam count sets the simulation's cost. --exclude-geom moves visual-only geoms that enclose the sensor into its body, keeping their pose,
because a rangefinder skips only its own body's geoms; Gazebo's near-clip at range_min hid them instead.
"""

import argparse
import math
import xml.etree.ElementTree as ET


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mjcf")
    parser.add_argument("--body", required=True)
    parser.add_argument("--name", required=True)
    parser.add_argument("--samples", type=int, required=True)
    parser.add_argument("--angle-min", type=float, required=True)
    parser.add_argument("--angle-max", type=float, required=True)
    parser.add_argument("--range-min", type=float, required=True)
    parser.add_argument("--range-max", type=float, required=True)
    parser.add_argument("--noise", type=float, default=0.0)
    parser.add_argument("--exclude-geom", nargs="*", default=[])
    args = parser.parse_args()

    tree = ET.parse(args.mjcf)
    root = tree.getroot()
    body = root.find(f".//body[@name='{args.body}']")
    for name in args.exclude_geom:
        parent = next(b for b in root.iter("body") if b.find(f"geom[@name='{name}']") is not None)
        geom = parent.find(f"geom[@name='{name}']")
        if body not in list(parent) or body.get("quat") or body.get("euler"):
            raise SystemExit(f"{args.body} must be an unrotated child of the body holding {name}")
        offset = [float(v) for v in body.get("pos", "0 0 0").split()]
        pos = [float(v) for v in geom.get("pos", "0 0 0").split()]
        geom.set("pos", " ".join(f"{a - b:.6g}" for a, b in zip(pos, offset)))
        parent.remove(geom)
        body.insert(0, geom)
    sensors = root.find("sensor")
    if sensors is None:
        sensors = ET.SubElement(root, "sensor")
    increment = (args.angle_max - args.angle_min) / (args.samples - 1)
    sweep = increment * (args.samples - 1)
    user = f"{args.noise} 0 {sweep:.6f} {increment:.7f} {args.range_min} {args.range_max}"
    # A rangefinder measures along its site's +z: turn z onto +x, then about the body's z to the beam's angle.
    to_x = (math.cos(math.pi / 4), 0.0, math.sin(math.pi / 4), 0.0)
    width = len(str(args.samples - 1))
    for i in range(args.samples):
        theta = args.angle_min + i * increment
        q = quat_mul((math.cos(theta / 2), 0.0, 0.0, math.sin(theta / 2)), to_x)
        site = f"{args.name}-{i:0{width}d}"
        ET.SubElement(body, "site", name=site, size="0.005", group="4", quat=" ".join(f"{v:.7f}" for v in q))
        ET.SubElement(sensors, "rangefinder", site=site, user=user)
    ET.indent(tree)
    tree.write(args.mjcf)


if __name__ == "__main__":
    main()
