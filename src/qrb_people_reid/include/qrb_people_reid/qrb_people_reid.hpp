#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <qrb_ros_people_reid/srv/compute_similarity.hpp>
#include <qrb_ros_people_reid/srv/extract_feature.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <chrono>
#include <cstddef>
#include <memory>
#include <limits>
#include <string>
#include <vector>

namespace qrb_people_reid
{
class QrbPeopleReid : public BT::StatefulActionNode
{
public:
  QrbPeopleReid(const std::string& name, const BT::NodeConfig& config,
                const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class State { IDLE, WAIT_EXTRACT, WAIT_SIMILARITY };

  bool beginNextCandidate();
  sensor_msgs::msg::Image cropPerson(const sensor_msgs::msg::Image& image,
                                     const vision_msgs::msg::Detection2D& detection) const;
  bool timedOut() const;
  BT::NodeStatus finishNoMatch(const std::string& reason);
  void resetRunState();

  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> shared_resources_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<qrb_ros_people_reid::srv::ExtractFeature>::SharedPtr extract_client_;
  rclcpp::Client<qrb_ros_people_reid::srv::ComputeSimilarity>::SharedPtr similarity_client_;

  rclcpp::Client<qrb_ros_people_reid::srv::ExtractFeature>::SharedFuture extract_future_;
  rclcpp::Client<qrb_ros_people_reid::srv::ComputeSimilarity>::SharedFuture similarity_future_;

  State state_{State::IDLE};
  sensor_msgs::msg::Image image_;
  vision_msgs::msg::Detection2DArray detections_;
  std::vector<float> reference_feature_;
  std::vector<float> candidate_feature_;
  std::size_t candidate_index_{0};
  std::size_t active_candidate_index_{0};
  float best_score_{std::numeric_limits<float>::infinity()};
  std::size_t best_index_{0};
  std::vector<float> best_feature_;
  double match_threshold_{0.35};
  double template_update_threshold_{0.20};
  double timeout_sec_{3.0};
  std::chrono::steady_clock::time_point request_start_;
};
}  // namespace qrb_people_reid
