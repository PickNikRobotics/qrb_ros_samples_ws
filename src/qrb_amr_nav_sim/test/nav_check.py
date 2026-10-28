"""Layer-2 check for qrb_amr_nav_sim: with the simulator, controllers, Cartographer, and Nav2 up (no Runtime),
confirm /scan and /map, the map -> odom -> base_footprint chain, then send one /navigate_to_pose goal and report
where the base ended up in TF and in MuJoCo's ground truth.

Usage: python3 nav_check.py <goal x> <goal y>
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class Check(Node):
    def __init__(self):
        super().__init__("qrb_nav_check")
        self.scan = None
        self.map = None
        self.create_subscription(LaserScan, "/scan", lambda m: setattr(self, "scan", m), QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "map", m), QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def pose(self, parent, child):
        t = self.tf.lookup_transform(parent, child, rclpy.time.Time()).transform
        q = t.rotation
        return t.translation.x, t.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    gx, gy = float(sys.argv[1]), float(sys.argv[2])
    rclpy.init()
    n = Check()
    n.spin_for(5)
    if n.scan:
        valid = [r for r in n.scan.ranges if n.scan.range_min <= r <= n.scan.range_max]
        print(f"scan frame={n.scan.header.frame_id} beams={len(n.scan.ranges)} valid={len(valid)} min={min(valid):.2f} max={max(valid):.2f}")
    else:
        print("scan: NONE")
    print(f"map: {'%dx%d @ %.2f' % (n.map.info.width, n.map.info.height, n.map.info.resolution) if n.map else 'NONE'}")
    for parent, child in (("map", "odom"), ("odom", "base_footprint"), ("map", "laser_scan_ROS"), ("mj_world", "map")):
        try:
            print(f"tf {parent}->{child}: x=%.3f y=%.3f yaw=%.3f" % n.pose(parent, child))
        except Exception as e:
            print(f"tf {parent}->{child}: MISSING ({type(e).__name__})")
    if not n.nav.wait_for_server(timeout_sec=30):
        print("navigate_to_pose: server not available")
        return 1
    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = "map"
    goal.pose.pose.position.x, goal.pose.pose.position.y = gx, gy
    goal.pose.pose.orientation.w = 1.0
    send = n.nav.send_goal_async(goal)
    rclpy.spin_until_future_complete(n, send)
    handle = send.result()
    print(f"goal ({gx}, {gy}) accepted={handle.accepted}")
    result = handle.get_result_async()
    start = time.time()
    while not result.done() and time.time() - start < 120:
        rclpy.spin_once(n, timeout_sec=0.1)
    if not result.done():
        handle.cancel_goal_async()
        print("navigate_to_pose: TIMEOUT after 120 s")
    else:
        r = result.result()
        print(f"navigate_to_pose: status={r.status} error_code={r.result.error_code} '{r.result.error_msg}' in {time.time() - start:.1f} s")
    n.spin_for(1)
    try:
        print("base in map (SLAM): x=%.3f y=%.3f yaw=%.3f" % n.pose("map", "base_footprint"))
    except Exception as e:
        print(f"base in map: MISSING ({type(e).__name__})")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
