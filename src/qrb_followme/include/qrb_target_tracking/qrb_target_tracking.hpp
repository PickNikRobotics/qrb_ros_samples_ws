#pragma once

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d.hpp>

#include <memory>
#include <string>

namespace qrb_target_tracking
{
class QrbTargetTracking : public BT::SyncActionNode
{
public:
  QrbTargetTracking(
    const std::string & name,
    const BT::NodeConfig & config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext> & shared_resources);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  struct PidState
  {
    double integral{0.0};
    double previous_error{0.0};
    bool initialized{false};
  };

  static double clamp(double value, double low, double high);
  static double runPid(double error, double dt, double kp, double ki, double kd,
                       double max_output, PidState & state);
  bool readDepthMeters(const sensor_msgs::msg::Image & depth,
                       const vision_msgs::msg::Detection2D & detection,
                       int roi_radius, double min_depth, double max_depth,
                       double & depth_m) const;
  void publishStop(const std::string & reason);

  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> shared_resources_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  std::string cmd_vel_topic_;
  rclcpp::Time previous_time_;
  PidState linear_pid_;
  PidState angular_pid_;
};
}  // namespace qrb_target_tracking
