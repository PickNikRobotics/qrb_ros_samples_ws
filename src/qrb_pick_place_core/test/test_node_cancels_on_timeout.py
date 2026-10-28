"""The ported node must cancel an accepted controller goal when it gives up waiting for the result.

A fake rm_group_controller accepts the trajectory and never finishes it. The node is started with a short result
timeout; it must send a cancel for the goal before exiting non-zero, so the arm does not keep moving unsupervised.
"""

import os
import subprocess
import threading
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from moveit_pro_controllers_msgs.action import FollowJointTrajectoryWithAdmittance
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand

DOMAIN = "232"
JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint01", "joint11", "joint02", "joint22"]


def test_cancels_goal_when_result_times_out():
    os.environ["ROS_DOMAIN_ID"] = DOMAIN
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    rclpy.init()
    node = rclpy.create_node("fake_controllers")
    cancelled = threading.Event()

    def execute(goal_handle):
        while not goal_handle.is_cancel_requested:
            time.sleep(0.05)
        cancelled.set()
        goal_handle.canceled()
        return FollowJointTrajectoryWithAdmittance.Result()

    ActionServer(
        node,
        FollowJointTrajectoryWithAdmittance,
        "/rm_group_controller/follow_joint_trajectory",
        execute_callback=execute,
        goal_callback=lambda _: GoalResponse.ACCEPT,
        cancel_callback=lambda _: CancelResponse.ACCEPT,
    )
    ActionServer(node, GripperCommand, "/hand_controller/gripper_cmd", execute_callback=lambda h: h.succeed() or GripperCommand.Result())
    publisher = node.create_publisher(JointState, "/joint_states", 10)
    def publish_state():
        msg = JointState(name=JOINTS, position=[0.0] * len(JOINTS))
        msg.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(msg)

    node.create_timer(0.05, publish_state)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    config = Path(get_package_share_directory("qrb_pick_place_core")) / "config"
    params = Path(os.environ.get("TMPDIR", "/tmp")) / "qrb_cancel_params.yaml"
    params.write_text(
        yaml.safe_dump(
            {
                "qrb_ros_arm_pick_place": {
                    "ros__parameters": {
                        "robot_description": (config / "rml63.urdf").read_text(),
                        "robot_description_semantic": (config / "rml_63.srdf").read_text(),
                        "joint_limits": yaml.safe_load((config / "joint_limits.yaml").read_text())["joint_limits"],
                        "result_timeout_s": 2.0,
                    }
                }
            }
        )
    )
    exe = Path(get_package_prefix("qrb_pick_place_core")) / "lib/qrb_pick_place_core/qrb_ros_arm_pick_place"
    try:
        result = subprocess.run(
            [str(exe), "--ros-args", "--params-file", str(params)], capture_output=True, text=True, timeout=60
        )
    finally:
        executor.shutdown()
        rclpy.shutdown()
    output = result.stdout + result.stderr
    assert result.returncode == 1, f"exit {result.returncode}\n{output[-800:]}"
    assert cancelled.wait(5.0), f"node exited without cancelling the accepted goal\n{output[-800:]}"
