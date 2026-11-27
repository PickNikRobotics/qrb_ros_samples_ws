#include <qrb_yolo_behavior/qrb_yolo_behavior.hpp>

#include <functional>
#include <mutex>
#include <string>
#include <utility>

#include <spdlog/spdlog.h>

namespace qrb_yolo_behavior
{

QrbYoloBehavior::QrbYoloBehavior(
  const std::string& name,
  const BT::NodeConfiguration& config,
  const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>(
      name,
      config,
      shared_resources)
{
  if (!shared_resources)
  {
    throw BT::RuntimeError(
      "QrbYoloBehavior: BehaviorContext is null");
  }

  node_ = shared_resources->node;

  if (!node_)
  {
    throw BT::RuntimeError(
      "QrbYoloBehavior: ROS node in BehaviorContext is null");
  }

  const std::string image_topic =
    getInput<std::string>("image_topic").value_or("/image_raw");

  const std::string result_topic =
    getInput<std::string>("result_topic").value_or(
      "/yolo_detect_result");

  image_pub_ =
    node_->create_publisher<sensor_msgs::msg::Image>(
      image_topic,
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable());  
  //    rclcpp::SensorDataQoS());

  result_sub_ =
    node_->create_subscription<
      vision_msgs::msg::Detection2DArray>(
      result_topic,
      rclcpp::QoS(10),
      std::bind(
        &QrbYoloBehavior::resultCallback,
        this,
        std::placeholders::_1));

  RCLCPP_INFO(
    node_->get_logger(),
    "QrbYoloBehavior initialized. image_topic='%s', result_topic='%s'",
    image_topic.c_str(),
    result_topic.c_str());

  spdlog::info(
    "{} initialized. image_topic='{}', result_topic='{}'",
    name,
    image_topic,
    result_topic);
}

BT::PortsList QrbYoloBehavior::providedPorts()
{
  return {
    BT::InputPort<sensor_msgs::msg::Image>(
      "image",
      "Input image"),

    BT::InputPort<std::string>(
      "image_topic",
      "/image_raw",
      "Image topic published to the QRB YOLO node"),

    BT::InputPort<std::string>(
      "result_topic",
      "/yolo_detect_result",
      "Detection result topic from the QRB YOLO node"),

    BT::InputPort<double>(
      "timeout_sec",
      5.0,
      "Maximum time to wait for a YOLO result"),

    BT::OutputPort<vision_msgs::msg::Detection2DArray>(
      "yolo_detect_result",
      "YOLO detection result")
  };
}

BT::KeyValueVector QrbYoloBehavior::metadata()
{
  return {
    {
      "description",
      "Object detection behavior running on a Qualcomm IQ device"
    },
    {
      "subcategory",
      "User Created Behaviors"
    }
  };
}

BT::NodeStatus QrbYoloBehavior::onStart()
{
  if (!node_)
  {
    const std::string error_message =
      "QrbYoloBehavior cannot start because the ROS node is null";

    spdlog::error(
      "{}: {}",
      name(),
      error_message);

    getBehaviorContext()->logger->publishFailureMessage(
      name(),
      error_message);

    return BT::NodeStatus::FAILURE;
  }

  if (!image_pub_)
  {
    const std::string error_message =
      "QrbYoloBehavior cannot start because the image publisher is null";

    RCLCPP_ERROR(
      node_->get_logger(),
      "%s",
      error_message.c_str());

    spdlog::error(
      "{}: {}",
      name(),
      error_message);

    getBehaviorContext()->logger->publishFailureMessage(
      name(),
      error_message);

    return BT::NodeStatus::FAILURE;
  }

  if (!result_sub_)
  {
    const std::string error_message =
      "QrbYoloBehavior cannot start because the result subscription is null";

    RCLCPP_ERROR(
      node_->get_logger(),
      "%s",
      error_message.c_str());

    spdlog::error(
      "{}: {}",
      name(),
      error_message);

    getBehaviorContext()->logger->publishFailureMessage(
      name(),
      error_message);

    return BT::NodeStatus::FAILURE;
  }

  const auto image =
    getInput<sensor_msgs::msg::Image>("image");

  if (!image)
  {
    const std::string error_message =
      "QrbYoloBehavior failed to read required input port 'image': " +
      image.error();

    RCLCPP_ERROR(
      node_->get_logger(),
      "%s",
      error_message.c_str());

    spdlog::error(
      "{}: {}",
      name(),
      error_message);

    getBehaviorContext()->logger->publishFailureMessage(
      name(),
      error_message);

    return BT::NodeStatus::FAILURE;
  }

  timeout_sec_ =
    getInput<double>("timeout_sec").value_or(5.0);

  if (timeout_sec_ <= 0.0)
  {
    const std::string error_message =
      "QrbYoloBehavior received an invalid timeout_sec value: " +
      std::to_string(timeout_sec_);

    RCLCPP_ERROR(
      node_->get_logger(),
      "%s",
      error_message.c_str());

    spdlog::error(
      "{}: {}",
      name(),
      error_message);

    getBehaviorContext()->logger->publishFailureMessage(
      name(),
      error_message);

    return BT::NodeStatus::FAILURE;
  }

  {
    std::scoped_lock lock(mutex_);
    latest_result_.reset();
  }

  started_at_ = node_->now();

  image_pub_->publish(image.value());

  RCLCPP_INFO(
    node_->get_logger(),
    "QrbYoloBehavior published one image. Waiting up to %.3f seconds "
    "for the YOLO result",
    timeout_sec_);

  spdlog::info(
    "{}: published one image and started waiting for a YOLO result",
    name());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus QrbYoloBehavior::onRunning()
{
  vision_msgs::msg::Detection2DArray::ConstSharedPtr result;

  {
    std::scoped_lock lock(mutex_);
    result = latest_result_;
  }

  if (result)
  {
    setOutput(
      "yolo_detect_result",
      *result);

    RCLCPP_INFO(
      node_->get_logger(),
      "QrbYoloBehavior received a YOLO result containing %zu detections",
      result->detections.size());

    spdlog::info(
      "{}: received a YOLO result containing {} detections",
      name(),
      result->detections.size());

    return BT::NodeStatus::SUCCESS;
  }

  const double elapsed_sec =
    (node_->now() - started_at_).seconds();

  if (elapsed_sec <= timeout_sec_)
  {
    return BT::NodeStatus::RUNNING;
  }

  const std::string warning_message =
    "QrbYoloBehavior timed out after " +
    std::to_string(timeout_sec_) +
    " seconds while waiting for a YOLO detection result. "
    "Returning FAILURE so the Behavior Tree can request the next camera frame.";

  RCLCPP_WARN(
    node_->get_logger(),
    "%s",
    warning_message.c_str());

  spdlog::warn(
    "{}: {}",
    name(),
    warning_message);

  getBehaviorContext()->logger->publishWarnMessage(
    name(),
    warning_message);

  {
    std::scoped_lock lock(mutex_);
    latest_result_.reset();
  }

  return BT::NodeStatus::FAILURE;
}

void QrbYoloBehavior::onHalted()
{
  {
    std::scoped_lock lock(mutex_);
    latest_result_.reset();
  }

  if (node_)
  {
    RCLCPP_INFO(
      node_->get_logger(),
      "QrbYoloBehavior was halted and cleared the cached result");
  }

  spdlog::info(
    "{}: behavior was halted",
    name());
}

void QrbYoloBehavior::resultCallback(
  vision_msgs::msg::Detection2DArray::ConstSharedPtr msg)
{
  if (!msg)
  {
    if (node_)
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "QrbYoloBehavior received a null YOLO result message");
    }

    spdlog::warn(
      "{}: received a null YOLO result message",
      name());

    return;
  }

  if (status() != BT::NodeStatus::RUNNING)
  {
    if (node_)
    {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "QrbYoloBehavior ignored a YOLO result because the Behavior "
        "is not currently running");
    }

    return;
  }

  const std::size_t detection_count =
    msg->detections.size();

  {
    std::scoped_lock lock(mutex_);
    latest_result_ = std::move(msg);
  }

  if (node_)
  {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "QrbYoloBehavior stored a YOLO result containing %zu detections",
      detection_count);
  }
}

}  // namespace qrb_yolo_behavior
