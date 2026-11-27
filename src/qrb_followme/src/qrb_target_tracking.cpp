#include <qrb_target_tracking/qrb_target_tracking.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d.hpp>

namespace
{

constexpr char kDefaultCmdVelTopic[] = "/cmd_vel";
constexpr char kLogFile[] = "/tmp/follow_me_target_tracking.log";

std::mutex g_log_mutex;

enum class LogLevel
{
  DEBUG,
  INFO,
  WARN,
  ERROR
};

const char * logLevelToString(LogLevel level)
{
  switch (level) {
    case LogLevel::DEBUG:
      return "DEBUG";

    case LogLevel::INFO:
      return "INFO";

    case LogLevel::WARN:
      return "WARN";

    case LogLevel::ERROR:
      return "ERROR";

    default:
      return "UNKNOWN";
  }
}

/*
 * This is the only function in this source file that directly opens
 * and writes /tmp/follow_me_target_tracking.log.
 *
 * Other functions must call logMessage() instead of creating their
 * own std::ofstream.
 */
bool writeTrackingLog(
  const rclcpp::Time & stamp,
  LogLevel level,
  const std::string & message)
{
  std::lock_guard<std::mutex> lock(g_log_mutex);

  std::ofstream log_file(
    kLogFile,
    std::ios::out | std::ios::app);

  if (!log_file.is_open()) {
    std::cerr
      << "[ERROR] Failed to open target tracking log file: "
      << kLogFile
      << std::endl;

    return false;
  }

  log_file
    << "["
    << std::fixed
    << std::setprecision(6)
    << stamp.seconds()
    << "] ["
    << logLevelToString(level)
    << "] "
    << message
    << '\n';

  log_file.flush();

  if (!log_file.good()) {
    std::cerr
      << "[ERROR] Failed to write target tracking log file: "
      << kLogFile
      << std::endl;

    return false;
  }

  return true;
}

/*
 * Unified ROS console log and file log interface.
 */
void logMessage(
  const rclcpp::Node::SharedPtr & node,
  LogLevel level,
  const std::string & message)
{
  rclcpp::Time stamp(
    0,
    0,
    RCL_ROS_TIME);

  if (node) {
    stamp = node->now();

    switch (level) {
      case LogLevel::DEBUG:
        RCLCPP_DEBUG(
          node->get_logger(),
          "%s",
          message.c_str());
        break;

      case LogLevel::INFO:
        RCLCPP_INFO(
          node->get_logger(),
          "%s",
          message.c_str());
        break;

      case LogLevel::WARN:
        RCLCPP_WARN(
          node->get_logger(),
          "%s",
          message.c_str());
        break;

      case LogLevel::ERROR:
        RCLCPP_ERROR(
          node->get_logger(),
          "%s",
          message.c_str());
        break;
    }
  } else {
    std::cerr
      << "["
      << logLevelToString(level)
      << "] "
      << message
      << std::endl;
  }

  writeTrackingLog(
    stamp,
    level,
    message);
}

vision_msgs::msg::Detection2D alignDetectionToImage(
  const vision_msgs::msg::Detection2D & input,
  int target_width,
  int target_height,
  int source_width,
  int source_height,
  bool source_letterboxed)
{
  vision_msgs::msg::Detection2D output = input;

  if (target_width <= 0 || target_height <= 0) {
    return output;
  }

  const double input_center_x =
    input.bbox.center.position.x;

  const double input_center_y =
    input.bbox.center.position.y;

  const double input_size_x =
    input.bbox.size_x;

  const double input_size_y =
    input.bbox.size_y;

  /*
   * Case 1:
   * Detection output uses normalized coordinates [0, 1].
   */
  const bool normalized_coordinates =
    input_center_x >= 0.0 &&
    input_center_x <= 1.0 &&
    input_center_y >= 0.0 &&
    input_center_y <= 1.0 &&
    input_size_x > 0.0 &&
    input_size_x <= 1.0 &&
    input_size_y > 0.0 &&
    input_size_y <= 1.0;

  if (normalized_coordinates) {
    output.bbox.center.position.x =
      input_center_x *
      static_cast<double>(target_width);

    output.bbox.center.position.y =
      input_center_y *
      static_cast<double>(target_height);

    output.bbox.size_x =
      input_size_x *
      static_cast<double>(target_width);

    output.bbox.size_y =
      input_size_y *
      static_cast<double>(target_height);
  } else if (source_width <= 0 || source_height <= 0) {
    /*
     * source_width/source_height equal to zero means the detection
     * bbox is already in the RGB/depth image coordinate system.
     */
  } else if (
    source_width == target_width &&
    source_height == target_height)
  {
    /*
     * Detector and target image already have the same resolution.
     */
  } else if (!source_letterboxed) {
    /*
     * Case 2:
     * Detector input was resized directly without letterbox.
     */
    const double scale_x =
      static_cast<double>(target_width) /
      static_cast<double>(source_width);

    const double scale_y =
      static_cast<double>(target_height) /
      static_cast<double>(source_height);

    output.bbox.center.position.x =
      input_center_x * scale_x;

    output.bbox.center.position.y =
      input_center_y * scale_y;

    output.bbox.size_x =
      input_size_x * scale_x;

    output.bbox.size_y =
      input_size_y * scale_y;
  } else {
    /*
     * Case 3:
     * Detector input used YOLO-style letterbox preprocessing.
     *
     * Forward:
     * detector_x = original_x * gain + pad_x
     * detector_y = original_y * gain + pad_y
     *
     * Reverse:
     * original_x = (detector_x - pad_x) / gain
     * original_y = (detector_y - pad_y) / gain
     */
    const double gain_x =
      static_cast<double>(source_width) /
      static_cast<double>(target_width);

    const double gain_y =
      static_cast<double>(source_height) /
      static_cast<double>(target_height);

    const double gain =
      std::min(
      gain_x,
      gain_y);

    if (!std::isfinite(gain) || gain <= 0.0) {
      return output;
    }

    const double resized_width =
      static_cast<double>(target_width) *
      gain;

    const double resized_height =
      static_cast<double>(target_height) *
      gain;

    const double pad_x =
      (
        static_cast<double>(source_width) -
        resized_width
      ) / 2.0;

    const double pad_y =
      (
        static_cast<double>(source_height) -
        resized_height
      ) / 2.0;

    output.bbox.center.position.x =
      (input_center_x - pad_x) /
      gain;

    output.bbox.center.position.y =
      (input_center_y - pad_y) /
      gain;

    output.bbox.size_x =
      input_size_x /
      gain;

    output.bbox.size_y =
      input_size_y /
      gain;
  }

  /*
   * All coordinate conversion paths must pass through the same
   * boundary clamping logic.
   */
  output.bbox.center.position.x =
    std::clamp(
    output.bbox.center.position.x,
    0.0,
    static_cast<double>(target_width - 1));

  output.bbox.center.position.y =
    std::clamp(
    output.bbox.center.position.y,
    0.0,
    static_cast<double>(target_height - 1));

  output.bbox.size_x =
    std::clamp(
    output.bbox.size_x,
    1.0,
    static_cast<double>(target_width));

  output.bbox.size_y =
    std::clamp(
    output.bbox.size_y,
    1.0,
    static_cast<double>(target_height));

  return output;
}

}  // namespace

namespace qrb_target_tracking
{

QrbTargetTracking::QrbTargetTracking(
  const std::string & name,
  const BT::NodeConfig & config,
  const std::shared_ptr<
    moveit_pro::behaviors::BehaviorContext> & shared_resources)
: BT::SyncActionNode(
    name,
    config),
  shared_resources_(
    shared_resources),
  node_(
    shared_resources ?
    shared_resources->node :
    nullptr),
  previous_time_(
    node_ ?
    node_->now() :
    rclcpp::Time(
      0,
      0,
      RCL_ROS_TIME))
{
  if (!shared_resources_) {
    writeTrackingLog(
      rclcpp::Time(
        0,
        0,
        RCL_ROS_TIME),
      LogLevel::ERROR,
      "QrbTargetTracking initialization failed: "
      "shared_resources is null");

    throw std::runtime_error(
      "QrbTargetTracking: shared_resources is null");
  }

  if (!node_) {
    writeTrackingLog(
      rclcpp::Time(
        0,
        0,
        RCL_ROS_TIME),
      LogLevel::ERROR,
      "QrbTargetTracking initialization failed: "
      "ROS node is null");

    throw std::runtime_error(
      "QrbTargetTracking: ROS node is null");
  }

  std::ostringstream message_stream;

  message_stream
    << "QrbTargetTracking initialized"
    << " | BT_node_name=" << name
    << " | log_file=" << kLogFile
    << " | cmd_type=geometry_msgs/msg/TwistStamped";

  logMessage(
    node_,
    LogLevel::INFO,
    message_stream.str());
}

BT::PortsList QrbTargetTracking::providedPorts()
{
  return {
    BT::InputPort<sensor_msgs::msg::Image>(
      "depth_image",
      "Aligned depth image, 16UC1 mm or 32FC1 m"),

    BT::InputPort<sensor_msgs::msg::CameraInfo>(
      "camera_info",
      "RGB/depth camera intrinsics"),

    BT::InputPort<vision_msgs::msg::Detection2D>(
      "target_detection",
      "Re-ID selected target bbox"),

    BT::InputPort<int>(
      "detection_image_width",
      "0",
      "Detector input width; 0 means bbox is already aligned"),

    BT::InputPort<int>(
      "detection_image_height",
      "0",
      "Detector input height; 0 means bbox is already aligned"),

    BT::InputPort<bool>(
      "detection_letterboxed",
      "false",
      "True when detector uses YOLO-style letterbox preprocessing"),

    BT::InputPort<std::string>(
      "cmd_vel_topic",
      "/cmd_vel",
      "ROS 2 TwistStamped topic used to control the AMR"),

    BT::InputPort<double>(
      "target_distance",
      "2.0",
      "Desired target distance in meters"),

    BT::InputPort<double>(
      "distance_tolerance",
      "0.15",
      "Linear deadband in meters"),

    BT::InputPort<double>(
      "angle_tolerance",
      "0.05",
      "Angular deadband in radians"),

    BT::InputPort<double>(
      "min_valid_depth",
      "0.3",
      "Minimum valid depth in meters"),

    BT::InputPort<double>(
      "max_valid_depth",
      "8.0",
      "Maximum valid depth in meters"),

    BT::InputPort<int>(
      "depth_roi_radius",
      "5",
      "Half-size of center depth sampling ROI"),

    BT::InputPort<double>(
      "linear_kp",
      "0.45",
      "Linear PID Kp"),

    BT::InputPort<double>(
      "linear_ki",
      "0.0",
      "Linear PID Ki"),

    BT::InputPort<double>(
      "linear_kd",
      "0.05",
      "Linear PID Kd"),

    BT::InputPort<double>(
      "angular_kp",
      "1.2",
      "Angular PID Kp"),

    BT::InputPort<double>(
      "angular_ki",
      "0.0",
      "Angular PID Ki"),

    BT::InputPort<double>(
      "angular_kd",
      "0.08",
      "Angular PID Kd"),

    BT::InputPort<double>(
      "max_linear_speed",
      "0.45",
      "Maximum linear speed"),

    BT::InputPort<double>(
      "max_angular_speed",
      "0.8",
      "Maximum angular speed"),

    BT::OutputPort<double>(
      "target_depth",
      "Median target depth in meters"),

    BT::OutputPort<double>(
      "target_angle",
      "Horizontal bearing angle in radians"),

    BT::OutputPort<std::string>(
      "message",
      "Diagnostic result")
  };
}

BT::NodeStatus QrbTargetTracking::tick()
{
  if (!node_) {
    writeTrackingLog(
      rclcpp::Time(
        0,
        0,
        RCL_ROS_TIME),
      LogLevel::ERROR,
      "QrbTargetTracking::tick called with null ROS node");

    return BT::NodeStatus::FAILURE;
  }

  // ============================================================
  // 1. Confirm that BehaviorTree actually executes this node
  // ============================================================

  logMessage(
    node_,
    LogLevel::INFO,
    "QrbTargetTracking::tick entered");

  // ============================================================
  // 2. Read configurable parameters
  // ============================================================

  std::string cmd_vel_topic =
    kDefaultCmdVelTopic;

  double target_distance = 2.0;
  double distance_tolerance = 0.15;
  double angle_tolerance = 0.05;

  double min_depth = 0.3;
  double max_depth = 8.0;

  double linear_kp = 0.45;
  double linear_ki = 0.0;
  double linear_kd = 0.05;

  double angular_kp = 1.2;
  double angular_ki = 0.0;
  double angular_kd = 0.08;

  double max_linear = 0.45;
  double max_angular = 0.8;

  int roi_radius = 5;

  int detection_image_width = 0;
  int detection_image_height = 0;
  bool detection_letterboxed = false;

  getInput(
    "cmd_vel_topic",
    cmd_vel_topic);

  getInput(
    "target_distance",
    target_distance);

  getInput(
    "distance_tolerance",
    distance_tolerance);

  getInput(
    "angle_tolerance",
    angle_tolerance);

  getInput(
    "min_valid_depth",
    min_depth);

  getInput(
    "max_valid_depth",
    max_depth);

  getInput(
    "depth_roi_radius",
    roi_radius);

  getInput(
    "detection_image_width",
    detection_image_width);

  getInput(
    "detection_image_height",
    detection_image_height);

  getInput(
    "detection_letterboxed",
    detection_letterboxed);

  getInput(
    "linear_kp",
    linear_kp);

  getInput(
    "linear_ki",
    linear_ki);

  getInput(
    "linear_kd",
    linear_kd);

  getInput(
    "angular_kp",
    angular_kp);

  getInput(
    "angular_ki",
    angular_ki);

  getInput(
    "angular_kd",
    angular_kd);

  getInput(
    "max_linear_speed",
    max_linear);

  getInput(
    "max_angular_speed",
    max_angular);

  if (cmd_vel_topic.empty()) {
    cmd_vel_topic =
      kDefaultCmdVelTopic;
  }

  // ============================================================
  // 3. Create publisher before validating BT inputs
  //
  // This guarantees that missing depth/detection/camera_info can
  // still trigger a STOP command.
  // ============================================================

  if (
    !cmd_vel_pub_ ||
    cmd_vel_topic_ != cmd_vel_topic)
  {
    cmd_vel_pub_.reset();

    cmd_vel_topic_ =
      cmd_vel_topic;

    const auto qos =
      rclcpp::QoS(
      rclcpp::KeepLast(10))
      .reliable()
      .durability_volatile();

    try {
      cmd_vel_pub_ =
        node_->create_publisher<
        geometry_msgs::msg::TwistStamped>(
        cmd_vel_topic_,
        qos);
    } catch (const std::exception & exception) {
      std::ostringstream message_stream;

      message_stream
        << "Failed to create cmd_vel publisher"
        << " | topic=" << cmd_vel_topic_
        << " | message_type="
        << "geometry_msgs/msg/TwistStamped"
        << " | exception=" << exception.what();

      const std::string message =
        message_stream.str();

      logMessage(
        node_,
        LogLevel::ERROR,
        message);

      setOutput(
        "message",
        message);

      return BT::NodeStatus::FAILURE;
    }

    std::ostringstream message_stream;

    message_stream
      << "Created cmd_vel publisher"
      << " | topic=" << cmd_vel_topic_
      << " | message_type="
      << "geometry_msgs/msg/TwistStamped"
      << " | subscription_count="
      << cmd_vel_pub_->get_subscription_count();

    logMessage(
      node_,
      LogLevel::INFO,
      message_stream.str());
  }

  // ============================================================
  // 4. Get required BehaviorTree inputs
  // ============================================================

  const auto depth =
    getInput<sensor_msgs::msg::Image>(
    "depth_image");

  const auto info =
    getInput<sensor_msgs::msg::CameraInfo>(
    "camera_info");

  const auto detection =
    getInput<vision_msgs::msg::Detection2D>(
    "target_detection");

  if (!depth || !info || !detection) {
    std::ostringstream message_stream;

    message_stream
      << "Missing required BehaviorTree input"
      << " | depth_image="
      << (depth ? "available" : "missing")
      << " | camera_info="
      << (info ? "available" : "missing")
      << " | target_detection="
      << (detection ? "available" : "missing");

    publishStop(
      message_stream.str());

    return BT::NodeStatus::FAILURE;
  }

  if (depth->width == 0 || depth->height == 0) {
    std::ostringstream message_stream;

    message_stream
      << "Invalid depth image dimensions"
      << " | width=" << depth->width
      << " | height=" << depth->height
      << " | encoding=" << depth->encoding
      << " | step=" << depth->step
      << " | data_size=" << depth->data.size();

    publishStop(
      message_stream.str());

    return BT::NodeStatus::FAILURE;
  }

  if (
    !std::isfinite(info->k[0]) ||
    info->k[0] <= 0.0)
  {
    std::ostringstream message_stream;

    message_stream
      << "Invalid camera_info"
      << " | fx=" << info->k[0]
      << " | cx=" << info->k[2]
      << " | width=" << info->width
      << " | height=" << info->height;

    publishStop(
      message_stream.str());

    return BT::NodeStatus::FAILURE;
  }

  {
    std::ostringstream message_stream;

    message_stream
      << "Required BT inputs received"
      << " | depth="
      << depth->width
      << "x"
      << depth->height
      << " " << depth->encoding
      << " | depth_step=" << depth->step
      << " | depth_data_size=" << depth->data.size()
      << " | camera_fx=" << info->k[0]
      << " | camera_cx=" << info->k[2]
      << " | detection_center=("
      << detection->bbox.center.position.x
      << ","
      << detection->bbox.center.position.y
      << ")"
      << " | detection_size=("
      << detection->bbox.size_x
      << ","
      << detection->bbox.size_y
      << ")";

    logMessage(
      node_,
      LogLevel::INFO,
      message_stream.str());
  }

  // ============================================================
  // 5. Validate configurable parameters
  // ============================================================

  if (
    !std::isfinite(min_depth) ||
    !std::isfinite(max_depth) ||
    min_depth < 0.0 ||
    max_depth <= min_depth)
  {
    std::ostringstream message_stream;

    message_stream
      << "Invalid depth range configuration"
      << " | min_valid_depth=" << min_depth
      << " | max_valid_depth=" << max_depth;

    publishStop(
      message_stream.str());

    return BT::NodeStatus::FAILURE;
  }

  if (
    !std::isfinite(target_distance) ||
    target_distance < 0.0)
  {
    std::ostringstream message_stream;

    message_stream
      << "Invalid target distance"
      << " | target_distance=" << target_distance;

    publishStop(
      message_stream.str());

    return BT::NodeStatus::FAILURE;
  }

  distance_tolerance =
    std::max(
    0.0,
    std::abs(distance_tolerance));

  angle_tolerance =
    std::max(
    0.0,
    std::abs(angle_tolerance));

  roi_radius =
    std::max(
    1,
    roi_radius);

  max_linear =
    std::abs(max_linear);

  max_angular =
    std::abs(max_angular);

  // ============================================================
  // 6. Align detection bbox to depth image coordinates
  // ============================================================

  const auto aligned_detection =
    alignDetectionToImage(
    detection.value(),
    static_cast<int>(depth->width),
    static_cast<int>(depth->height),
    detection_image_width,
    detection_image_height,
    detection_letterboxed);

  const double aligned_center_x =
    aligned_detection.bbox.center.position.x;

  const double aligned_center_y =
    aligned_detection.bbox.center.position.y;

  const double aligned_size_x =
    aligned_detection.bbox.size_x;

  const double aligned_size_y =
    aligned_detection.bbox.size_y;

  if (
    !std::isfinite(aligned_center_x) ||
    !std::isfinite(aligned_center_y) ||
    !std::isfinite(aligned_size_x) ||
    !std::isfinite(aligned_size_y) ||
    aligned_size_x <= 0.0 ||
    aligned_size_y <= 0.0)
  {
    std::ostringstream message_stream;

    message_stream
      << "Invalid aligned target bbox"
      << " | center=("
      << aligned_center_x
      << ","
      << aligned_center_y
      << ")"
      << " | size=("
      << aligned_size_x
      << ","
      << aligned_size_y
      << ")";

    publishStop(
      message_stream.str());

    return BT::NodeStatus::FAILURE;
  }

  if (
    aligned_center_x < 0.0 ||
    aligned_center_x >=
    static_cast<double>(depth->width) ||
    aligned_center_y < 0.0 ||
    aligned_center_y >=
    static_cast<double>(depth->height))
  {
    std::ostringstream message_stream;

    message_stream
      << "Aligned target bbox center is outside depth image"
      << " | center=("
      << aligned_center_x
      << ","
      << aligned_center_y
      << ")"
      << " | depth_image="
      << depth->width
      << "x"
      << depth->height;

    publishStop(
      message_stream.str());

    return BT::NodeStatus::FAILURE;
  }

  {
    std::ostringstream message_stream;

    message_stream
      << "BBox alignment"
      << " | input_center=("
      << detection->bbox.center.position.x
      << ","
      << detection->bbox.center.position.y
      << ")"
      << " | input_size=("
      << detection->bbox.size_x
      << ","
      << detection->bbox.size_y
      << ")"
      << " | detector_image="
      << detection_image_width
      << "x"
      << detection_image_height
      << " | letterboxed="
      << (
        detection_letterboxed ?
        "true" :
        "false")
      << " | aligned_center=("
      << aligned_center_x
      << ","
      << aligned_center_y
      << ")"
      << " | aligned_size=("
      << aligned_size_x
      << ","
      << aligned_size_y
      << ")"
      << " | depth_image="
      << depth->width
      << "x"
      << depth->height;

    logMessage(
      node_,
      LogLevel::INFO,
      message_stream.str());
  }

  // ============================================================
  // 7. Get target depth with aligned bbox
  // ============================================================

  double depth_m = 0.0;

  if (!readDepthMeters(
      depth.value(),
      aligned_detection,
      roi_radius,
      min_depth,
      max_depth,
      depth_m))
  {
    publishStop(
      "No valid depth samples inside aligned "
      "target bbox center ROI");

    return BT::NodeStatus::FAILURE;
  }

  if (!std::isfinite(depth_m)) {
    publishStop(
      "Calculated target depth is not finite");

    return BT::NodeStatus::FAILURE;
  }

  // ============================================================
  // 8. Calculate target horizontal angle
  // ============================================================

  const double u =
    aligned_detection.bbox.center.position.x;

  const double fx =
    info->k[0];

  const double cx =
    info->k[2];

  const double angle =
    std::atan2(
      u - cx,
      fx);

  if (!std::isfinite(angle)) {
    publishStop(
      "Calculated target angle is not finite");

    return BT::NodeStatus::FAILURE;
  }

  // ============================================================
  // 9. Calculate PID errors
  // ============================================================

  const double distance_error =
    depth_m - target_distance;

  /*
   * Positive image angle means the target is to the right.
   * Keep the original sign convention used by the existing code.
   */
  const double angular_error =
    -angle;

  // ============================================================
  // 10. Calculate dt
  // ============================================================

  const auto now =
    node_->now();

  double dt =
    (now - previous_time_).seconds();

  previous_time_ =
    now;

  if (
    !std::isfinite(dt) ||
    dt <= 0.0 ||
    dt > 1.0)
  {
    dt = 0.05;
  }

  // ============================================================
  // 11. Generate TwistStamped
  // ============================================================

  geometry_msgs::msg::TwistStamped cmd;

  cmd.header.stamp =
    now;

  cmd.header.frame_id =
    "base_link";

  if (
    std::abs(distance_error) <=
    distance_tolerance)
  {
    cmd.twist.linear.x = 0.0;
    linear_pid_ = {};
  } else {
    cmd.twist.linear.x =
      runPid(
      distance_error,
      dt,
      linear_kp,
      linear_ki,
      linear_kd,
      max_linear,
      linear_pid_);
  }

  if (
    std::abs(angle) <=
    angle_tolerance)
  {
    cmd.twist.angular.z = 0.0;
    angular_pid_ = {};
  } else {
    cmd.twist.angular.z =
      runPid(
      angular_error,
      dt,
      angular_kp,
      angular_ki,
      angular_kd,
      max_angular,
      angular_pid_);
  }

  cmd.twist.linear.y = 0.0;
  cmd.twist.linear.z = 0.0;

  cmd.twist.angular.x = 0.0;
  cmd.twist.angular.y = 0.0;

  if (
    !std::isfinite(cmd.twist.linear.x) ||
    !std::isfinite(cmd.twist.angular.z))
  {
    publishStop(
      "PID output is not finite");

    return BT::NodeStatus::FAILURE;
  }

  // ============================================================
  // 12. Publish AMR command
  // ============================================================

  if (!cmd_vel_pub_) {
    publishStop(
      "cmd_vel publisher unexpectedly became null");

    return BT::NodeStatus::FAILURE;
  }

  const std::size_t subscription_count =
    cmd_vel_pub_->get_subscription_count();

  cmd_vel_pub_->publish(cmd);

  // ============================================================
  // 13. Set BehaviorTree outputs
  // ============================================================

  setOutput(
    "target_depth",
    depth_m);

  setOutput(
    "target_angle",
    angle);

  // ============================================================
  // 14. Log tracking result
  // ============================================================

  std::ostringstream message_stream;

  message_stream
    << "Target tracking command published"
    << " | topic=" << cmd_vel_topic_
    << " | message_type="
    << "geometry_msgs/msg/TwistStamped"
    << " | subscription_count="
    << subscription_count
    << " | depth=" << depth_m << " m"
    << " | target_distance="
    << target_distance << " m"
    << " | distance_error="
    << distance_error << " m"
    << " | angle=" << angle << " rad"
    << " | angular_error="
    << angular_error << " rad"
    << " | aligned_u=" << u
    << " | fx=" << fx
    << " | cx=" << cx
    << " | dt=" << dt << " s"
    << " | linear.x="
    << cmd.twist.linear.x << " m/s"
    << " | angular.z="
    << cmd.twist.angular.z << " rad/s";

  if (subscription_count == 0) {
    message_stream
      << " | warning=no matched cmd_vel subscribers";
  }

  const std::string message =
    message_stream.str();

  setOutput(
    "message",
    message);

  logMessage(
    node_,
    LogLevel::INFO,
    message);

  return BT::NodeStatus::SUCCESS;
}

double QrbTargetTracking::clamp(
  double value,
  double low,
  double high)
{
  return std::max(
    low,
    std::min(
      value,
      high));
}

double QrbTargetTracking::runPid(
  double error,
  double dt,
  double kp,
  double ki,
  double kd,
  double max_output,
  PidState & state)
{
  if (
    !std::isfinite(error) ||
    !std::isfinite(dt) ||
    !std::isfinite(kp) ||
    !std::isfinite(ki) ||
    !std::isfinite(kd) ||
    !std::isfinite(max_output) ||
    dt <= 0.0 ||
    max_output < 0.0)
  {
    state = {};
    return 0.0;
  }

  if (!state.initialized) {
    state.previous_error =
      error;

    state.integral =
      0.0;

    state.initialized =
      true;
  }

  state.integral =
    clamp(
    state.integral +
    error * dt,
    -max_output,
    max_output);

  const double derivative =
    (error - state.previous_error) /
    dt;

  state.previous_error =
    error;

  const double output =
    kp * error +
    ki * state.integral +
    kd * derivative;

  if (!std::isfinite(output)) {
    state = {};
    return 0.0;
  }

  return clamp(
    output,
    -max_output,
    max_output);
}

bool QrbTargetTracking::readDepthMeters(
  const sensor_msgs::msg::Image & depth,
  const vision_msgs::msg::Detection2D & detection,
  int roi_radius,
  double min_depth,
  double max_depth,
  double & depth_m) const
{
  depth_m = 0.0;

  cv::Mat image;
  double scale = 1.0;

  std::size_t bytes_per_pixel = 0;

  if (
    depth.encoding == "16UC1" ||
    depth.encoding == "mono16")
  {
    bytes_per_pixel =
      sizeof(std::uint16_t);
  } else if (depth.encoding == "32FC1") {
    bytes_per_pixel =
      sizeof(float);
  } else {
    const std::string message =
      std::string("Unsupported depth encoding: ") +
      depth.encoding;

    logMessage(
      node_,
      LogLevel::ERROR,
      message);

    return false;
  }

  const std::size_t minimum_step =
    static_cast<std::size_t>(depth.width) *
    bytes_per_pixel;

  const std::size_t expected_data_size =
    static_cast<std::size_t>(depth.step) *
    static_cast<std::size_t>(depth.height);

  if (
    depth.step < minimum_step ||
    depth.data.size() < expected_data_size)
  {
    std::ostringstream message_stream;

    message_stream
      << "Invalid depth image memory layout"
      << " | width=" << depth.width
      << " | height=" << depth.height
      << " | encoding=" << depth.encoding
      << " | step=" << depth.step
      << " | minimum_step=" << minimum_step
      << " | data_size=" << depth.data.size()
      << " | expected_data_size="
      << expected_data_size;

    logMessage(
      node_,
      LogLevel::ERROR,
      message_stream.str());

    return false;
  }

  try {
    const auto depth_message =
      std::make_shared<sensor_msgs::msg::Image>(
      depth);

    if (
      depth.encoding == "16UC1" ||
      depth.encoding == "mono16")
    {
      image =
        cv_bridge::toCvShare(
        depth_message,
        "16UC1")->image;

      scale = 0.001;
    } else {
      image =
        cv_bridge::toCvShare(
        depth_message,
        "32FC1")->image;

      scale = 1.0;
    }
  } catch (const cv_bridge::Exception & exception) {
    const std::string message =
      std::string("cv_bridge exception: ") +
      exception.what();

    logMessage(
      node_,
      LogLevel::ERROR,
      message);

    return false;
  } catch (const cv::Exception & exception) {
    const std::string message =
      std::string("OpenCV exception: ") +
      exception.what();

    logMessage(
      node_,
      LogLevel::ERROR,
      message);

    return false;
  } catch (const std::exception & exception) {
    const std::string message =
      std::string(
      "Unexpected depth conversion exception: ") +
      exception.what();

    logMessage(
      node_,
      LogLevel::ERROR,
      message);

    return false;
  }

  if (image.empty()) {
    logMessage(
      node_,
      LogLevel::ERROR,
      "Converted depth image is empty");

    return false;
  }

  if (
    image.cols != static_cast<int>(depth.width) ||
    image.rows != static_cast<int>(depth.height))
  {
    std::ostringstream message_stream;

    message_stream
      << "Converted depth image dimensions do not match ROS image"
      << " | cv_image="
      << image.cols
      << "x"
      << image.rows
      << " | ros_image="
      << depth.width
      << "x"
      << depth.height;

    logMessage(
      node_,
      LogLevel::ERROR,
      message_stream.str());

    return false;
  }

  if (
    image.type() != CV_16UC1 &&
    image.type() != CV_32FC1)
  {
    std::ostringstream message_stream;

    message_stream
      << "Unexpected OpenCV depth image type"
      << " | cv_type=" << image.type()
      << " | expected_16UC1=" << CV_16UC1
      << " | expected_32FC1=" << CV_32FC1;

    logMessage(
      node_,
      LogLevel::ERROR,
      message_stream.str());

    return false;
  }

  const double bbox_center_x =
    detection.bbox.center.position.x;

  const double bbox_center_y =
    detection.bbox.center.position.y;

  if (
    !std::isfinite(bbox_center_x) ||
    !std::isfinite(bbox_center_y))
  {
    logMessage(
      node_,
      LogLevel::WARN,
      "Depth bbox center is not finite");

    return false;
  }

  const int center_x =
    static_cast<int>(
    std::lround(bbox_center_x));

  const int center_y =
    static_cast<int>(
    std::lround(bbox_center_y));

  if (
    center_x < 0 ||
    center_x >= image.cols ||
    center_y < 0 ||
    center_y >= image.rows)
  {
    std::ostringstream message_stream;

    message_stream
      << "Depth bbox center is outside image"
      << " | center=("
      << center_x
      << ","
      << center_y
      << ")"
      << " | original_center=("
      << bbox_center_x
      << ","
      << bbox_center_y
      << ")"
      << " | image="
      << image.cols
      << "x"
      << image.rows;

    logMessage(
      node_,
      LogLevel::WARN,
      message_stream.str());

    return false;
  }

  if (
    !std::isfinite(detection.bbox.size_x) ||
    !std::isfinite(detection.bbox.size_y) ||
    detection.bbox.size_x <= 0.0 ||
    detection.bbox.size_y <= 0.0)
  {
    std::ostringstream message_stream;

    message_stream
      << "Invalid bbox size for depth sampling"
      << " | size=("
      << detection.bbox.size_x
      << ","
      << detection.bbox.size_y
      << ")";

    logMessage(
      node_,
      LogLevel::WARN,
      message_stream.str());

    return false;
  }

  const int bbox_half_w =
    std::max(
    1,
    static_cast<int>(
      std::lround(
        detection.bbox.size_x /
        2.0)));

  const int bbox_half_h =
    std::max(
    1,
    static_cast<int>(
      std::lround(
        detection.bbox.size_y /
        2.0)));

  roi_radius =
    std::max(
    1,
    std::min({
      roi_radius,
      bbox_half_w,
      bbox_half_h
    }));

  const int x0 =
    std::clamp(
    center_x - roi_radius,
    0,
    image.cols - 1);

  const int y0 =
    std::clamp(
    center_y - roi_radius,
    0,
    image.rows - 1);

  const int x1 =
    std::clamp(
    center_x + roi_radius,
    0,
    image.cols - 1);

  const int y1 =
    std::clamp(
    center_y + roi_radius,
    0,
    image.rows - 1);

  if (x0 > x1 || y0 > y1) {
    std::ostringstream message_stream;

    message_stream
      << "Invalid depth ROI"
      << " | x=["
      << x0
      << ","
      << x1
      << "]"
      << " | y=["
      << y0
      << ","
      << y1
      << "]";

    logMessage(
      node_,
      LogLevel::WARN,
      message_stream.str());

    return false;
  }

  std::vector<double> values;

  values.reserve(
    static_cast<std::size_t>(
      (x1 - x0 + 1) *
      (y1 - y0 + 1)));

  std::size_t total_count = 0;
  std::size_t finite_count = 0;
  std::size_t zero_count = 0;
  std::size_t invalid_count = 0;
  std::size_t out_of_range_count = 0;

  double observed_min =
    std::numeric_limits<double>::max();

  double observed_max =
    std::numeric_limits<double>::lowest();

  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      ++total_count;

      double value_m = 0.0;

      if (image.type() == CV_16UC1) {
        value_m =
          static_cast<double>(
          image.at<std::uint16_t>(
            y,
            x)) *
          scale;
      } else {
        value_m =
          static_cast<double>(
          image.at<float>(
            y,
            x)) *
          scale;
      }

      if (!std::isfinite(value_m)) {
        ++invalid_count;
        continue;
      }

      ++finite_count;

      if (value_m <= 0.0) {
        ++zero_count;
        continue;
      }

      observed_min =
        std::min(
        observed_min,
        value_m);

      observed_max =
        std::max(
        observed_max,
        value_m);

      if (
        value_m < min_depth ||
        value_m > max_depth)
      {
        ++out_of_range_count;
        continue;
      }

      values.push_back(
        value_m);
    }
  }

  if (values.empty()) {
    std::ostringstream message_stream;

    message_stream
      << "No valid depth values"
      << " | image="
      << image.cols
      << "x"
      << image.rows
      << " | encoding="
      << depth.encoding
      << " | cv_type="
      << image.type()
      << " | ros_step="
      << depth.step
      << " | cv_step="
      << image.step
      << " | bbox_center=("
      << bbox_center_x
      << ","
      << bbox_center_y
      << ")"
      << " | pixel_center=("
      << center_x
      << ","
      << center_y
      << ")"
      << " | bbox_size=("
      << detection.bbox.size_x
      << ","
      << detection.bbox.size_y
      << ")"
      << " | ROI=x["
      << x0
      << ","
      << x1
      << "] y["
      << y0
      << ","
      << y1
      << "]"
      << " | depth_range=["
      << min_depth
      << ","
      << max_depth
      << "]"
      << " | total="
      << total_count
      << " | finite="
      << finite_count
      << " | zero="
      << zero_count
      << " | invalid_nan_or_inf="
      << invalid_count
      << " | out_of_range="
      << out_of_range_count;

    if (
      observed_min !=
      std::numeric_limits<double>::max())
    {
      message_stream
        << " | observed_min="
        << observed_min
        << " | observed_max="
        << observed_max;
    }

    logMessage(
      node_,
      LogLevel::WARN,
      message_stream.str());

    return false;
  }

  const auto middle =
    values.begin() +
    static_cast<std::ptrdiff_t>(
      values.size() /
      2);

  std::nth_element(
    values.begin(),
    middle,
    values.end());

  depth_m =
    *middle;

  if (!std::isfinite(depth_m)) {
    logMessage(
      node_,
      LogLevel::WARN,
      "Median depth result is not finite");

    return false;
  }

  std::ostringstream message_stream;

  message_stream
    << "Depth ROI success"
    << " | center=("
    << center_x
    << ","
    << center_y
    << ")"
    << " | ROI=x["
    << x0
    << ","
    << x1
    << "] y["
    << y0
    << ","
    << y1
    << "]"
    << " | total_samples="
    << total_count
    << " | finite_samples="
    << finite_count
    << " | valid_samples="
    << values.size()
    << " | invalid_nan_or_inf="
    << invalid_count
    << " | zero_samples="
    << zero_count
    << " | out_of_range_samples="
    << out_of_range_count
    << " | median_depth="
    << depth_m
    << " m";

  logMessage(
    node_,
    LogLevel::INFO,
    message_stream.str());

  return true;
}

void QrbTargetTracking::publishStop(
  const std::string & reason)
{
  if (!node_) {
    writeTrackingLog(
      rclcpp::Time(
        0,
        0,
        RCL_ROS_TIME),
      LogLevel::ERROR,
      std::string(
        "Unable to publish STOP because ROS node is null") +
      " | reason=" +
      reason);

    return;
  }

  std::string requested_topic =
    kDefaultCmdVelTopic;

  getInput(
    "cmd_vel_topic",
    requested_topic);

  if (requested_topic.empty()) {
    requested_topic =
      kDefaultCmdVelTopic;
  }

  if (
    !cmd_vel_pub_ ||
    cmd_vel_topic_ != requested_topic)
  {
    cmd_vel_pub_.reset();

    cmd_vel_topic_ =
      requested_topic;

    const auto qos =
      rclcpp::QoS(
      rclcpp::KeepLast(10))
      .reliable()
      .durability_volatile();

    try {
      cmd_vel_pub_ =
        node_->create_publisher<
        geometry_msgs::msg::TwistStamped>(
        cmd_vel_topic_,
        qos);
    } catch (const std::exception & exception) {
      std::ostringstream message_stream;

      message_stream
        << "Failed to create cmd_vel publisher in publishStop"
        << " | topic=" << cmd_vel_topic_
        << " | message_type="
        << "geometry_msgs/msg/TwistStamped"
        << " | exception=" << exception.what()
        << " | original_reason=" << reason;

      const std::string message =
        message_stream.str();

      logMessage(
        node_,
        LogLevel::ERROR,
        message);

      setOutput(
        "message",
        message);

      linear_pid_ = {};
      angular_pid_ = {};

      return;
    }

    std::ostringstream message_stream;

    message_stream
      << "Created cmd_vel publisher from publishStop"
      << " | topic=" << cmd_vel_topic_
      << " | message_type="
      << "geometry_msgs/msg/TwistStamped"
      << " | subscription_count="
      << cmd_vel_pub_->get_subscription_count();

    logMessage(
      node_,
      LogLevel::INFO,
      message_stream.str());
  }

  if (!cmd_vel_pub_) {
    const std::string message =
      std::string(
        "Unable to publish STOP because publisher is null") +
      " | topic=" +
      cmd_vel_topic_ +
      " | reason=" +
      reason;

    logMessage(
      node_,
      LogLevel::ERROR,
      message);

    setOutput(
      "message",
      message);

    linear_pid_ = {};
    angular_pid_ = {};

    return;
  }

  geometry_msgs::msg::TwistStamped stop;

  stop.header.stamp =
    node_->now();

  stop.header.frame_id =
    "base_link";

  stop.twist.linear.x = 0.0;
  stop.twist.linear.y = 0.0;
  stop.twist.linear.z = 0.0;

  stop.twist.angular.x = 0.0;
  stop.twist.angular.y = 0.0;
  stop.twist.angular.z = 0.0;

  const std::size_t subscription_count =
    cmd_vel_pub_->get_subscription_count();

  try {
    cmd_vel_pub_->publish(
      stop);
  } catch (const std::exception & exception) {
    std::ostringstream message_stream;

    message_stream
      << "Exception while publishing STOP command"
      << " | topic=" << cmd_vel_topic_
      << " | exception=" << exception.what()
      << " | original_reason=" << reason;

    const std::string message =
      message_stream.str();

    logMessage(
      node_,
      LogLevel::ERROR,
      message);

    setOutput(
      "message",
      message);

    linear_pid_ = {};
    angular_pid_ = {};

    return;
  }

  setOutput(
    "message",
    reason);

  linear_pid_ = {};
  angular_pid_ = {};

  std::ostringstream message_stream;

  message_stream
    << "STOP command published"
    << " | topic=" << cmd_vel_topic_
    << " | message_type="
    << "geometry_msgs/msg/TwistStamped"
    << " | subscription_count="
    << subscription_count
    << " | linear.x=0"
    << " | angular.z=0"
    << " | reason=" << reason;

  if (subscription_count == 0) {
    message_stream
      << " | warning=no matched cmd_vel subscribers";
  }

  logMessage(
    node_,
    LogLevel::WARN,
    message_stream.str());
}

}  // namespace qrb_target_tracking
