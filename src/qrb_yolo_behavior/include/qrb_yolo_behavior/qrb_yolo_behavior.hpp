#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <behaviortree_cpp/action_node.h>

#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

namespace qrb_yolo_behavior
{

class QrbYoloBehavior
  : public moveit_pro::behaviors::SharedResourcesNode<
      BT::StatefulActionNode>
{
public:
  QrbYoloBehavior(
    const std::string& name,
    const BT::NodeConfiguration& config,
    const std::shared_ptr<
      moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:
  void resultCallback(
    vision_msgs::msg::Detection2DArray::ConstSharedPtr msg);

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  rclcpp::Subscription<
    vision_msgs::msg::Detection2DArray>::SharedPtr result_sub_;

  vision_msgs::msg::Detection2DArray::ConstSharedPtr latest_result_;

  rclcpp::Time started_at_;

  double timeout_sec_{ 5.0 };

  std::mutex mutex_;
};

}  // namespace qrb_yolo_behavior
