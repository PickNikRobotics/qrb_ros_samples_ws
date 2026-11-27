// Copyright (c) 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_studio_internal_msgs/srv/set_mujoco_free_body_poses.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class FollowMePeopleController : public rclcpp::Node
{
public:
  using SetFreeBodyPoses =
    moveit_studio_internal_msgs::srv::SetMujocoFreeBodyPoses;

  FollowMePeopleController()
  : Node("followme_people_controller")
  {
    /*
     * Parameters.
     */
    body_name_ = declare_parameter<std::string>(
      "body_name", "followme_target");

    service_name_ = declare_parameter<std::string>(
      "service_name", "/mujoco_system/set_free_body_poses");

    control_mode_topic_ = declare_parameter<std::string>(
      "control_mode_topic", "/followme_person/control_mode");

    /*
     * The SetMujocoFreeBodyPoses service uses geometry_msgs/msg/Pose,
     * which does not contain a frame_id.
     *
     * Keep this parameter for launch-file compatibility and logging.
     * The MuJoCo service interprets poses in its world coordinate frame.
     */
    world_frame_ = declare_parameter<std::string>(
      "world_frame", "mj_world");

    movement_speed_ = declare_parameter<double>(
      "movement_speed", 0.5);

    update_rate_ = declare_parameter<double>(
      "update_rate", 20.0);

    target_z_ = declare_parameter<double>(
      "target_z", 0.0);

    loop_path_ = declare_parameter<bool>(
      "loop_path", true);

    /*
     * Initial orientation.
     *
     * The orientation will be updated dynamically in AUTO mode so that
     * the person faces the direction of movement.
     */
    target_yaw_ = declare_parameter<double>(
      "target_yaw", kPi);

    /*
     * Waypoints:
     *
     * First move from X=6.0 to X=8.0.
     * Then reverse direction and move from X=8.0 to X=2.0.
     *
     * Y remains 6.0 m.
     * Distance between adjacent waypoints is 0.5 m.
     */
    waypoints_ = {
      {7.5, 6.0},
      {7.0, 6.0},
      {6.5, 6.0},
      {6.0, 6.0},
      {5.5, 6.0},
      {5.0, 6.0},
      {4.5, 6.0},
      {4.0, 6.0},
      {3.5, 6.0},
      {3.0, 6.0},
      {2.5, 6.0},
      {2.0, 6.0},
  {2.0, 6.0},
  {2.5, 6.0},
  {3.0, 6.0},
  {3.5, 6.0},
  {4.0, 6.0},
  {4.5, 6.0},
  {5.0, 6.0},
  {5.5, 6.0},
  {6.0, 6.0},
  {6.5, 6.0},
  {7.0, 6.0},
  {7.5, 6.0},
  {8.0, 6.0}
    };


    if (waypoints_.empty()) {
      throw std::runtime_error("waypoints_ cannot be empty");
    }

    /*
     * Initialize the current pose at the first waypoint.
     */
    current_x_ = waypoints_.front().x;
    current_y_ = waypoints_.front().y;

    /*
     * Index 0 is the initial position. The first movement target is
     * waypoint index 1.
     */
    current_waypoint_index_ =
      waypoints_.size() > 1 ? 1 : 0;

    /*
     * Create MuJoCo service client.
     */
    set_pose_client_ =
      create_client<SetFreeBodyPoses>(service_name_);

    /*
     * Receive mode commands:
     *
     *   auto
     *   stop
     *   reset
     */
    control_mode_sub_ =
      create_subscription<std_msgs::msg::String>(
      control_mode_topic_,
      rclcpp::QoS(10),
      std::bind(
        &FollowMePeopleController::controlModeCallback,
        this,
        std::placeholders::_1));

    /*
     * Timer drives waypoint movement.
     */
    const double safe_update_rate =
      std::max(update_rate_, 1.0);

    const auto timer_period =
      std::chrono::duration<double>(
      1.0 / safe_update_rate);

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        timer_period),
      std::bind(
        &FollowMePeopleController::timerCallback,
        this));

    RCLCPP_INFO(
      get_logger(),
      "Follow-me person controller started");

    RCLCPP_INFO(
      get_logger(),
      "Body: '%s'",
      body_name_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "MuJoCo service: '%s'",
      service_name_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Control mode topic: '%s'",
      control_mode_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "MuJoCo world frame: '%s'",
      world_frame_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Movement speed: %.3f m/s",
      movement_speed_);

    RCLCPP_INFO(
      get_logger(),
      "Update rate: %.1f Hz",
      safe_update_rate);

    RCLCPP_INFO(
      get_logger(),
      "Initial position: x=%.3f, y=%.3f, z=%.3f",
      current_x_,
      current_y_,
      target_z_);

    RCLCPP_INFO(
      get_logger(),
      "Initial yaw: %.3f rad",
      target_yaw_);

    RCLCPP_INFO(
      get_logger(),
      "Initial mode: stop");

    RCLCPP_INFO(
      get_logger(),
      "Available commands: auto, stop, reset");
  }

private:
  static constexpr double kPi =
    3.14159265358979323846;

  struct Waypoint
  {
    double x;
    double y;
  };

  enum class ControlMode
  {
    STOP,
    AUTO
  };

  void controlModeCallback(
    const std_msgs::msg::String::SharedPtr msg)
  {
    std::string command = msg->data;

    /*
     * Convert command to lowercase.
     */
    std::transform(
      command.begin(),
      command.end(),
      command.begin(),
      [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
      });

    if (command == "auto") {
      handleAutoCommand();
      return;
    }

    if (command == "stop") {
      handleStopCommand();
      return;
    }

    if (command == "t") {
      handleResetCommand();
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "Unknown control command: '%s'. "
      "Available commands: auto, stop, reset",
      command.c_str());
  }

  void handleAutoCommand()
  {
    /*
     * If the path has already completed, restart it from the first
     * waypoint before entering AUTO mode.
     */
    if (path_completed_) {
      resetPathState();
      publishCurrentPose();
    }

    control_mode_ = ControlMode::AUTO;

    if (waypoints_.size() <= 1) {
      RCLCPP_WARN(
        get_logger(),
        "AUTO mode requested, but the path contains only one waypoint");
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "AUTO mode enabled, next waypoint %zu/%zu",
      current_waypoint_index_ + 1,
      waypoints_.size());
  }

  void handleStopCommand()
  {
    control_mode_ = ControlMode::STOP;

    RCLCPP_INFO(
      get_logger(),
      "STOP mode enabled at position "
      "x=%.3f, y=%.3f, yaw=%.3f",
      current_x_,
      current_y_,
      target_yaw_);
  }

  void handleResetCommand()
  {
    control_mode_ = ControlMode::STOP;

    resetPathState();
    publishCurrentPose();

    RCLCPP_INFO(
      get_logger(),
      "Path reset to first waypoint: "
      "x=%.3f, y=%.3f, z=%.3f",
      current_x_,
      current_y_,
      target_z_);
  }

  void resetPathState()
  {
    current_x_ = waypoints_.front().x;
    current_y_ = waypoints_.front().y;

    /*
     * Index 0 is the initial position, so the first target is index 1.
     */
    current_waypoint_index_ =
      waypoints_.size() > 1 ? 1 : 0;

    path_completed_ = false;

    /*
     * Set the initial orientation toward the first movement target.
     */
    if (waypoints_.size() > 1) {
      const double dx =
        waypoints_[1].x - current_x_;
      const double dy =
        waypoints_[1].y - current_y_;

      if (std::hypot(dx, dy) > 1e-6) {
        target_yaw_ = std::atan2(dy, dx);
      }
    }
  }

  void timerCallback()
  {
    if (control_mode_ != ControlMode::AUTO) {
      return;
    }

    /*
     * Only keep one MuJoCo service request in flight.
     */
    if (service_request_in_progress_) {
      return;
    }

    if (path_completed_) {
      return;
    }

    if (current_waypoint_index_ >= waypoints_.size()) {
      completePath();
      return;
    }

    moveTowardCurrentWaypoint();
  }

  void moveTowardCurrentWaypoint()
  {
    const Waypoint & target =
      waypoints_.at(current_waypoint_index_);

    const double dx =
      target.x - current_x_;

    const double dy =
      target.y - current_y_;

    const double distance =
      std::hypot(dx, dy);

    /*
     * Make the person face the current movement direction.
     *
     * +X direction: yaw = 0
     * -X direction: yaw = pi
     */
    if (distance > 1e-6) {
      target_yaw_ = std::atan2(dy, dx);
    }

    /*
     * Maximum movement for one timer cycle:
     *
     * step = movement_speed / update_rate
     *
     * Example:
     *   movement_speed = 0.5 m/s
     *   update_rate    = 20 Hz
     *   step           = 0.025 m
     */
    const double safe_update_rate =
      std::max(update_rate_, 1.0);

    const double max_step =
      std::max(movement_speed_, 0.0) /
      safe_update_rate;

    if (max_step <= 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Movement speed is zero; person cannot move");
      return;
    }

    /*
     * Treat the waypoint as reached when the remaining distance can
     * be completed in the current update.
     */
    if (distance <= max_step || distance < 1e-6) {
      current_x_ = target.x;
      current_y_ = target.y;

      publishCurrentPose();

      RCLCPP_INFO(
        get_logger(),
        "Reached waypoint %zu/%zu: "
        "x=%.3f, y=%.3f, yaw=%.3f",
        current_waypoint_index_ + 1,
        waypoints_.size(),
        current_x_,
        current_y_,
        target_yaw_);

      ++current_waypoint_index_;

      if (current_waypoint_index_ >= waypoints_.size()) {
        completePath();
      }

      return;
    }

    /*
     * Move one fixed step toward the current waypoint.
     */
    current_x_ +=
      max_step * dx / distance;

    current_y_ +=
      max_step * dy / distance;

    publishCurrentPose();
  }

  void completePath()
  {
    if (loop_path_) {
      RCLCPP_INFO(
        get_logger(),
        "Path completed; restarting because loop_path=true");

      resetPathState();
      publishCurrentPose();
      return;
    }

    path_completed_ = true;
    control_mode_ = ControlMode::STOP;

    RCLCPP_INFO(
      get_logger(),
      "Path completed at x=%.3f, y=%.3f, yaw=%.3f",
      current_x_,
      current_y_,
      target_yaw_);
  }

  void publishCurrentPose()
  {
    if (!set_pose_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Waiting for MuJoCo service '%s'",
        service_name_.c_str());
      return;
    }

    auto request =
      std::make_shared<SetFreeBodyPoses::Request>();

    /*
     * SetMujocoFreeBodyPoses::Request::poses is:
     *
     *   geometry_msgs/msg/Pose[]
     *
     * Therefore target_pose must be geometry_msgs::msg::Pose, not
     * geometry_msgs::msg::PoseStamped.
     */
    geometry_msgs::msg::Pose target_pose;

    target_pose.position.x = current_x_;
    target_pose.position.y = current_y_;
    target_pose.position.z = target_z_;

    /*
     * Quaternion generated from a yaw rotation around the Z axis:
     *
     * qx = 0
     * qy = 0
     * qz = sin(yaw / 2)
     * qw = cos(yaw / 2)
     */
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z =
      std::sin(target_yaw_ * 0.5);
    target_pose.orientation.w =
      std::cos(target_yaw_ * 0.5);

    request->body_names.push_back(body_name_);
    request->poses.push_back(target_pose);

    service_request_in_progress_ = true;

    set_pose_client_->async_send_request(
  request,
  [this](rclcpp::Client<SetFreeBodyPoses>::SharedFuture future)
  {
    service_request_in_progress_ = false;

    try {
      (void)future.get();
    } catch (const std::exception & exception) {
      control_mode_ = ControlMode::STOP;

      RCLCPP_ERROR(
        get_logger(),
        "Failed to move MuJoCo body '%s': %s. "
        "Controller changed to STOP mode.",
        body_name_.c_str(),
        exception.what());
    }
  });
  }

  /*
   * ROS interfaces.
   */
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    control_mode_sub_;

  rclcpp::Client<SetFreeBodyPoses>::SharedPtr
    set_pose_client_;

  rclcpp::TimerBase::SharedPtr timer_;

  /*
   * Configuration.
   */
  std::string body_name_;
  std::string service_name_;
  std::string control_mode_topic_;
  std::string world_frame_;

  double movement_speed_{0.5};
  double update_rate_{20.0};
  double target_z_{0.0};
  double target_yaw_{kPi};

  bool loop_path_{true};

  /*
   * Path state.
   */
  std::vector<Waypoint> waypoints_;

  std::size_t current_waypoint_index_{1};

  double current_x_{6.0};
  double current_y_{6.0};

  ControlMode control_mode_{ControlMode::STOP};

  bool path_completed_{false};
  bool service_request_in_progress_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<FollowMePeopleController>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
