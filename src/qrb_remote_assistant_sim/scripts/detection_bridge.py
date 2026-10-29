#!/usr/bin/env python3
"""Turn the QRB Detect Person Objective's person count into Qualcomm's detection message.

Qualcomm's task_manager_node waits for a vision_msgs/Detection2DArray on /yolo_detect_result with a result whose
class_id is "person", the label their YOLO pipeline (qrb_ros_yolo_process with coco8.yaml) publishes. MoveIt Pro's
SAM3 Behavior returns masks and a count instead, which the Objective publishes as a std_msgs/String on
/qrb_person_count. This node publishes one detection per counted person, so task_manager_node runs unchanged. On the
IQ-9, Qualcomm's own YOLO pipeline publishes /yolo_detect_result; do not run this node there.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class DetectionBridge(Node):
    def __init__(self, class_id):
        super().__init__("qrb_detection_bridge")
        self._class_id = class_id
        self._publisher = self.create_publisher(Detection2DArray, "/yolo_detect_result", 10)
        self.create_subscription(String, "/qrb_person_count", self._on_count, 10)

    def _on_count(self, msg):
        try:
            count = int(msg.data)
        except ValueError:
            self.get_logger().error(f"Ignoring /qrb_person_count {msg.data!r}: not a count")
            return
        out = Detection2DArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "camera_optical_frame"
        for _ in range(count):
            detection = Detection2D()
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = self._class_id
            hypothesis.hypothesis.score = 1.0
            detection.results.append(hypothesis)
            out.detections.append(detection)
        self._publisher.publish(out)


def main():
    rclpy.init()
    params = Node("qrb_detection_bridge_params")
    class_id = params.declare_parameter("class_id", "person").value
    params.destroy_node()
    node = DetectionBridge(class_id)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
