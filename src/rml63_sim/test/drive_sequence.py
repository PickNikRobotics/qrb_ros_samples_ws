"""Drive Qualcomm's pick-and-place joint targets straight to the controllers, without any planner.

Validates the sim + ros2_control layer independently of MoveIt Pro Core or Objectives.
Usage: python3 drive_sequence.py [pick|full]
"""

import sys

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint

from grasp_gate import ARM, CLOSE, HAND, OPEN, PICK, PLACE, RAISE, READY, RE_RAISE

PICK_STEPS = [("arm", READY), ("hand", OPEN), ("arm", PICK), ("hand", CLOSE), ("arm", RAISE)]
FULL_STEPS = PICK_STEPS + [("arm", PLACE), ("hand", OPEN), ("arm", RE_RAISE), ("arm", READY)]


def send(node, client, joints, goal, seconds):
    msg = FollowJointTrajectory.Goal()
    msg.trajectory.joint_names = joints
    point = JointTrajectoryPoint(positions=list(goal))
    point.time_from_start.sec = seconds
    msg.trajectory.points = [point]
    handle_future = client.send_goal_async(msg)
    rclpy.spin_until_future_complete(node, handle_future)
    handle = handle_future.result()
    if not handle.accepted:
        raise RuntimeError(f"goal rejected for {joints}")
    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result().result
    if result.error_code != 0:
        raise RuntimeError(f"{joints} failed: {result.error_code} {result.error_string}")


def main(mode):
    rclpy.init()
    node = rclpy.create_node("drive_sequence")
    clients = {
        "arm": (ActionClient(node, FollowJointTrajectory, "/rm_group_controller/follow_joint_trajectory"), ARM, 3),
        "hand": (ActionClient(node, FollowJointTrajectory, "/hand_controller/follow_joint_trajectory"), HAND, 1),
    }
    for client, _, _ in clients.values():
        client.wait_for_server(timeout_sec=10)
    for group, goal in PICK_STEPS if mode == "pick" else FULL_STEPS:
        client, joints, seconds = clients[group]
        send(node, client, joints, goal, seconds)
        print(f"{group} -> {[round(v, 3) for v in goal]} done")
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "pick")
