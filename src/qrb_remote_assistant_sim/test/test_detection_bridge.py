"""Tests for detection_bridge: a person count from the QRB Detect Person Objective becomes the
vision_msgs/Detection2DArray Qualcomm's task_manager_node reads on /yolo_detect_result."""

import pathlib
import sys
import threading
import time

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "scripts"))
from detection_bridge import DetectionBridge  # noqa: E402


@pytest.fixture
def system():
    rclpy.init()
    bridge = DetectionBridge(class_id="person")
    probe = Node("probe")
    received = []
    probe.create_subscription(Detection2DArray, "/yolo_detect_result", received.append, 10)
    counts = probe.create_publisher(String, "/qrb_person_count", 10)
    executor = MultiThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(probe)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    deadline = time.monotonic() + 5.0
    while counts.get_subscription_count() == 0 and time.monotonic() < deadline:
        time.sleep(0.05)

    def send(text):
        received.clear()
        counts.publish(String(data=text))
        deadline = time.monotonic() + 3.0
        while not received and time.monotonic() < deadline:
            time.sleep(0.02)
        return received[-1] if received else None

    yield send
    executor.shutdown()
    rclpy.shutdown()
    thread.join(timeout=5.0)


def test_a_count_publishes_one_detection_per_person_with_qualcomms_class_id(system):
    msg = system("2")
    assert msg is not None
    assert len(msg.detections) == 2
    assert all(d.results[0].hypothesis.class_id == "person" for d in msg.detections)


def test_zero_publishes_an_empty_array(system):
    msg = system("0")
    assert msg is not None
    assert msg.detections == []


def test_a_non_number_publishes_nothing(system):
    assert system("not a count") is None
