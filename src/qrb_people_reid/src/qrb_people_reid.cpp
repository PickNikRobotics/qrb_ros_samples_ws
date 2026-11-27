#include <qrb_people_reid/qrb_people_reid.hpp>

#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <future>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace qrb_people_reid
{
namespace
{

constexpr char kImagePort[] = "image";
constexpr char kDetectionsPort[] = "detections";
constexpr char kReferenceFeaturePort[] = "reference_feature";
constexpr char kMatchThresholdPort[] = "match_threshold";
constexpr char kTemplateUpdateThresholdPort[] =
  "template_update_threshold";
constexpr char kTimeoutPort[] = "timeout_sec";

constexpr int kMinimumCropWidth = 16;
constexpr int kMinimumCropHeight = 32;

bool isValidFeature(const std::vector<float>& feature)
{
  if (feature.empty())
  {
    return false;
  }

  return std::all_of(
    feature.begin(),
    feature.end(),
    [](const float value)
    {
      return std::isfinite(value);
    });
}

}  // namespace

QrbPeopleReid::QrbPeopleReid(
  const std::string& name,
  const BT::NodeConfig& config,
  const std::shared_ptr<
    moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : BT::StatefulActionNode(name, config),
    shared_resources_(shared_resources)
{
  if (!shared_resources_)
  {
    throw BT::RuntimeError(
      "QrbPeopleReid: BehaviorContext is null");
  }

  node_ = shared_resources_->node;

  if (!node_)
  {
    throw BT::RuntimeError(
      "QrbPeopleReid: ROS node in BehaviorContext is null");
  }

  extract_client_ =
    node_->create_client<
      qrb_ros_people_reid::srv::ExtractFeature>(
      "/extract_feature");

  similarity_client_ =
    node_->create_client<
      qrb_ros_people_reid::srv::ComputeSimilarity>(
      "/compute_similarity");

  RCLCPP_INFO(
    node_->get_logger(),
    "QrbPeopleReid initialized. "
    "extract_service='/extract_feature', "
    "similarity_service='/compute_similarity'");
}

BT::PortsList QrbPeopleReid::providedPorts()
{
  return {
    BT::InputPort<sensor_msgs::msg::Image>(
      kImagePort,
      "RGB image containing detected people"),

    BT::InputPort<vision_msgs::msg::Detection2DArray>(
      kDetectionsPort,
      "YOLO person detections"),

    BT::BidirectionalPort<std::vector<float>>(
      kReferenceFeaturePort,
      "Persistent target Re-ID template"),

    BT::InputPort<double>(
      kMatchThresholdPort,
      0.35,
      "Maximum distance accepted as a match"),

    BT::InputPort<double>(
      kTemplateUpdateThresholdPort,
      0.20,
      "Maximum distance for template update"),

    BT::InputPort<double>(
      kTimeoutPort,
      3.0,
      "Timeout for each service request"),

    BT::OutputPort<float>(
      "similarity_score",
      "Best Re-ID distance; lower is more similar"),

    BT::OutputPort<bool>(
      "matched",
      "True when the target was initialized or matched"),

    BT::OutputPort<vision_msgs::msg::Detection2D>(
      "target_detection",
      "Matched person detection"),

    BT::OutputPort<std::vector<float>>(
      "current_feature",
      "Feature extracted for the selected target"),

    BT::OutputPort<std::string>(
      "message",
      "Diagnostic result")
  };
}

BT::NodeStatus QrbPeopleReid::onStart()
{
  resetRunState();

  const auto image_input =
    getInput<sensor_msgs::msg::Image>(kImagePort);

  const auto detections_input =
    getInput<vision_msgs::msg::Detection2DArray>(
      kDetectionsPort);

  if (!image_input)
  {
    return finishNoMatch(
      "Missing required input port 'image': " +
      image_input.error());
  }

  if (!detections_input)
  {
    return finishNoMatch(
      "Missing required input port 'detections': " +
      detections_input.error());
  }

  image_ = image_input.value();
  detections_ = detections_input.value();

  reference_feature_.clear();

  const auto reference_feature_input =
    getInput<std::vector<float>>(
      kReferenceFeaturePort);

  if (reference_feature_input)
  {
    reference_feature_ =
      reference_feature_input.value();
  }

  match_threshold_ =
    getInput<double>(kMatchThresholdPort)
      .value_or(0.35);

  template_update_threshold_ =
    getInput<double>(kTemplateUpdateThresholdPort)
      .value_or(0.20);

  timeout_sec_ =
    getInput<double>(kTimeoutPort)
      .value_or(3.0);

  if (!std::isfinite(timeout_sec_) ||
      timeout_sec_ <= 0.0)
  {
    return finishNoMatch(
      "Invalid timeout_sec value: " +
      std::to_string(timeout_sec_));
  }

  if (!std::isfinite(match_threshold_) ||
      match_threshold_ < 0.0)
  {
    return finishNoMatch(
      "Invalid match_threshold value: " +
      std::to_string(match_threshold_));
  }

  if (!std::isfinite(template_update_threshold_) ||
      template_update_threshold_ < 0.0)
  {
    return finishNoMatch(
      "Invalid template_update_threshold value: " +
      std::to_string(template_update_threshold_));
  }

  if (image_.width == 0 ||
      image_.height == 0 ||
      image_.step == 0 ||
      image_.data.empty())
  {
    return finishNoMatch(
      "Input image is invalid: width=" +
      std::to_string(image_.width) +
      ", height=" +
      std::to_string(image_.height) +
      ", step=" +
      std::to_string(image_.step) +
      ", data_size=" +
      std::to_string(image_.data.size()) +
      ", encoding='" +
      image_.encoding +
      "'");
  }

  const std::size_t expected_minimum_size =
    static_cast<std::size_t>(image_.step) *
    static_cast<std::size_t>(image_.height);

  if (image_.data.size() < expected_minimum_size)
  {
    return finishNoMatch(
      "Input image buffer is smaller than step * height: "
      "data_size=" +
      std::to_string(image_.data.size()) +
      ", expected_at_least=" +
      std::to_string(expected_minimum_size));
  }

  if (detections_.detections.empty())
  {
    return finishNoMatch(
      "No person detections received");
  }

  if (!reference_feature_.empty() &&
      !isValidFeature(reference_feature_))
  {
    return finishNoMatch(
      "Reference feature contains invalid values");
  }

  if (!extract_client_)
  {
    return finishNoMatch(
      "/extract_feature client is null");
  }

  if (!extract_client_->service_is_ready())
  {
    return finishNoMatch(
      "/extract_feature service is not ready");
  }

  if (!reference_feature_.empty())
  {
    if (!similarity_client_)
    {
      return finishNoMatch(
        "/compute_similarity client is null");
    }

    if (!similarity_client_->service_is_ready())
    {
      return finishNoMatch(
        "/compute_similarity service is not ready");
    }
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "QrbPeopleReid started: "
    "image=%ux%u encoding='%s' step=%u data_size=%zu, "
    "detections=%zu, reference_feature_size=%zu, "
    "match_threshold=%.4f, "
    "template_update_threshold=%.4f, "
    "timeout_sec=%.3f",
    image_.width,
    image_.height,
    image_.encoding.c_str(),
    image_.step,
    image_.data.size(),
    detections_.detections.size(),
    reference_feature_.size(),
    match_threshold_,
    template_update_threshold_,
    timeout_sec_);

  if (!beginNextCandidate())
  {
    return finishNoMatch(
      "No valid person crop in detections");
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus QrbPeopleReid::onRunning()
{
  if (state_ == State::IDLE)
  {
    return finishNoMatch(
      "QrbPeopleReid entered onRunning while idle");
  }

  if (timedOut())
  {
    return finishNoMatch(
      "People Re-ID service call timed out after " +
      std::to_string(timeout_sec_) +
      " seconds");
  }

  if (state_ == State::WAIT_EXTRACT)
  {
    if (!extract_future_.valid())
    {
      return finishNoMatch(
        "Extract feature future is invalid");
    }

    if (
      extract_future_.wait_for(
        std::chrono::seconds(0)) ==
      std::future_status::ready)
    {
      qrb_ros_people_reid::srv::
        ExtractFeature::Response::SharedPtr response;

      try
      {
        response = extract_future_.get();
      }
      catch (const std::exception& ex)
      {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Exception while receiving /extract_feature response "
          "for candidate %zu: %s",
          active_candidate_index_,
          ex.what());

        return beginNextCandidate()
          ? BT::NodeStatus::RUNNING
          : finishNoMatch(
              "All feature extraction requests failed");
      }

      if (!response)
      {
        RCLCPP_WARN(
          node_->get_logger(),
          "Feature extraction returned a null response "
          "for candidate %zu",
          active_candidate_index_);

        return beginNextCandidate()
          ? BT::NodeStatus::RUNNING
          : finishNoMatch(
              "No candidate feature could be extracted");
      }

      if (!response->success)
      {
        RCLCPP_WARN(
          node_->get_logger(),
          "Feature extraction failed for candidate %zu: %s",
          active_candidate_index_,
          response->message.c_str());

        return beginNextCandidate()
          ? BT::NodeStatus::RUNNING
          : finishNoMatch(
              "No candidate feature could be extracted");
      }

      if (!isValidFeature(response->feature))
      {
        RCLCPP_WARN(
          node_->get_logger(),
          "Feature extraction returned an empty or invalid "
          "feature for candidate %zu",
          active_candidate_index_);

        return beginNextCandidate()
          ? BT::NodeStatus::RUNNING
          : finishNoMatch(
              "No candidate returned a valid feature");
      }

      candidate_feature_ = response->feature;

      RCLCPP_INFO(
        node_->get_logger(),
        "Feature extracted for candidate %zu: "
        "feature_size=%zu",
        active_candidate_index_,
        candidate_feature_.size());

      /*
       * An empty reference feature means target initialization.
       * The first valid candidate becomes the reference template.
       */
      if (reference_feature_.empty())
      {
        reference_feature_ = candidate_feature_;

        setOutput(
          kReferenceFeaturePort,
          reference_feature_);

        setOutput(
          "current_feature",
          candidate_feature_);

        setOutput(
          "similarity_score",
          0.0F);

        setOutput(
          "matched",
      true);

        setOutput(
          "target_detection",
          detections_.detections.at(
            active_candidate_index_));

        setOutput(
          "message",
          std::string(
            "Reference feature initialized"));

        RCLCPP_INFO(
          node_->get_logger(),
          "Reference feature initialized from candidate %zu. "
          "feature_size=%zu",
          active_candidate_index_,
          reference_feature_.size());

        resetRunState();

        return BT::NodeStatus::SUCCESS;
      }

      if (!similarity_client_)
      {
        return finishNoMatch(
          "/compute_similarity client is null");
      }

      if (!similarity_client_->service_is_ready())
      {
        return finishNoMatch(
          "/compute_similarity service is not ready");
      }

      auto request =
        std::make_shared<
          qrb_ros_people_reid::srv::
            ComputeSimilarity::Request>();

      request->feature1 = reference_feature_;
      request->feature2 = candidate_feature_;

      RCLCPP_INFO(
        node_->get_logger(),
        "Calling /compute_similarity for candidate %zu: "
        "reference_size=%zu, candidate_size=%zu",
        active_candidate_index_,
        request->feature1.size(),
        request->feature2.size());

      try
      {
        similarity_future_ =
          similarity_client_->async_send_request(
            request);
      }
      catch (const std::exception& ex)
      {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Failed to call /compute_similarity for "
          "candidate %zu: %s",
          active_candidate_index_,
          ex.what());

        return beginNextCandidate()
          ? BT::NodeStatus::RUNNING
          : finishNoMatch(
              "All similarity requests failed");
      }

      request_start_ =
        std::chrono::steady_clock::now();

      state_ = State::WAIT_SIMILARITY;

      return BT::NodeStatus::RUNNING;
    }
  }

  if (state_ == State::WAIT_SIMILARITY)
  {
    if (!similarity_future_.valid())
    {
      return finishNoMatch(
        "Compute similarity future is invalid");
    }

    if (
      similarity_future_.wait_for(
        std::chrono::seconds(0)) ==
      std::future_status::ready)
    {
      qrb_ros_people_reid::srv::
        ComputeSimilarity::Response::SharedPtr response;

      try
      {
        response = similarity_future_.get();
      }
      catch (const std::exception& ex)
      {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Exception while receiving "
          "/compute_similarity response "
          "for candidate %zu: %s",
          active_candidate_index_,
          ex.what());

        response.reset();
      }

      if (
        response &&
        response->success &&
        std::isfinite(response->similarity))
      {
        RCLCPP_INFO(
          node_->get_logger(),
          "Candidate %zu Re-ID distance: %.4f, "
          "match_threshold=%.4f",
          active_candidate_index_,
          response->similarity,
          match_threshold_);

        if (response->similarity < best_score_)
        {
          best_score_ = response->similarity;
          best_index_ = active_candidate_index_;
          best_feature_ = candidate_feature_;
        }
      }
      else
      {
        RCLCPP_WARN(
          node_->get_logger(),
          "Similarity failed for candidate %zu: %s",
          active_candidate_index_,
          response
            ? response->message.c_str()
            : "null response");
      }

      if (beginNextCandidate())
      {
        return BT::NodeStatus::RUNNING;
      }

      const bool matched =
        std::isfinite(best_score_) &&
        best_score_ <= match_threshold_;

      setOutput(
        "matched",
        matched);

      setOutput(
        "similarity_score",
        best_score_);

      if (!matched)
      {
        return finishNoMatch(
          "No detection passed the Re-ID match threshold");
      }

      if (best_index_ >= detections_.detections.size())
      {
        return finishNoMatch(
          "Internal error: best detection index is out of range");
      }

      if (!isValidFeature(best_feature_))
      {
        return finishNoMatch(
          "Internal error: best feature is invalid");
      }

      setOutput(
        "target_detection",
        detections_.detections.at(best_index_));

      setOutput(
        "current_feature",
        best_feature_);

      if (best_score_ <= template_update_threshold_)
      {
        reference_feature_ = best_feature_;

        setOutput(
          kReferenceFeaturePort,
          reference_feature_);

        setOutput(
          "message",
          std::string(
            "Target matched and reference feature updated"));

        RCLCPP_INFO(
          node_->get_logger(),
          "Target matched at candidate %zu. "
          "distance=%.4f. Reference feature updated.",
          best_index_,
          best_score_);
      }
      else
      {
        setOutput(
          kReferenceFeaturePort,
          reference_feature_);

        setOutput(
          "message",
          std::string(
            "Target matched; reference feature kept"));

        RCLCPP_INFO(
          node_->get_logger(),
          "Target matched at candidate %zu. "
          "distance=%.4f. Reference feature kept.",
          best_index_,
          best_score_);
      }

      resetRunState();

      return BT::NodeStatus::SUCCESS;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void QrbPeopleReid::onHalted()
{
  if (node_)
  {
    RCLCPP_INFO(
      node_->get_logger(),
      "QrbPeopleReid halted");
  }

  resetRunState();
}

bool QrbPeopleReid::beginNextCandidate()
{
  while (
    candidate_index_ <
    detections_.detections.size())
  {
    active_candidate_index_ =
      candidate_index_++;

    try
    {
      const auto& detection =
        detections_.detections.at(
          active_candidate_index_);

      const sensor_msgs::msg::Image cropped_image =
        cropPerson(
          image_,
          detection);

      RCLCPP_INFO(
        node_->get_logger(),
        "Calling /extract_feature for candidate %zu: "
        "width=%u, height=%u, encoding='%s', "
        "step=%u, data_size=%zu",
        active_candidate_index_,
        cropped_image.width,
        cropped_image.height,
        cropped_image.encoding.c_str(),
        cropped_image.step,
        cropped_image.data.size());

      if (
        cropped_image.width == 0 ||
        cropped_image.height == 0 ||
        cropped_image.step == 0 ||
        cropped_image.data.empty())
      {
        throw std::runtime_error(
          "generated crop is empty before service request");
      }

      const std::size_t expected_minimum_size =
        static_cast<std::size_t>(
          cropped_image.step) *
        static_cast<std::size_t>(
          cropped_image.height);

      if (
        cropped_image.data.size() <
        expected_minimum_size)
      {
        throw std::runtime_error(
          "generated crop buffer is smaller than "
          "step * height");
      }

      auto request =
        std::make_shared<
          qrb_ros_people_reid::srv::
            ExtractFeature::Request>();

      request->image = cropped_image;

      extract_future_ =
        extract_client_->async_send_request(
          request);

      request_start_ =
        std::chrono::steady_clock::now();

      state_ = State::WAIT_EXTRACT;

      return true;
    }
    catch (const cv_bridge::Exception& ex)
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "Skipping candidate %zu due to "
        "cv_bridge error: %s",
        active_candidate_index_,
        ex.what());
    }
    catch (const cv::Exception& ex)
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "Skipping candidate %zu due to "
        "OpenCV error: %s",
        active_candidate_index_,
        ex.what());
    }
    catch (const std::exception& ex)
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "Skipping invalid candidate %zu: %s",
        active_candidate_index_,
        ex.what());
    }
  }

  return false;
}

sensor_msgs::msg::Image QrbPeopleReid::cropPerson(
  const sensor_msgs::msg::Image& image,
  const vision_msgs::msg::Detection2D& detection) const
{
  if (
    image.width == 0 ||
    image.height == 0 ||
    image.step == 0 ||
    image.data.empty())
  {
    throw std::runtime_error(
      "source image is empty or has invalid metadata");
  }

  const std::size_t expected_minimum_size =
    static_cast<std::size_t>(image.step) *
    static_cast<std::size_t>(image.height);

  if (image.data.size() < expected_minimum_size)
  {
    throw std::runtime_error(
      "source image data is smaller than step * height");
  }

  const auto& bbox = detection.bbox;

  const double center_x =
    bbox.center.position.x;

  const double center_y =
    bbox.center.position.y;

  const double width =
    bbox.size_x;

  const double height =
    bbox.size_y;

  if (
    !std::isfinite(center_x) ||
    !std::isfinite(center_y) ||
    !std::isfinite(width) ||
    !std::isfinite(height))
  {
    throw std::runtime_error(
      "bbox contains NaN or infinite values");
  }

  if (width <= 1.0 || height <= 1.0)
  {
    throw std::runtime_error(
      "bbox width or height is too small: width=" +
      std::to_string(width) +
      ", height=" +
      std::to_string(height));
  }

  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr =
      cv_bridge::toCvCopy(
        image,
        "bgr8");
  }
  catch (const cv_bridge::Exception& ex)
  {
    throw std::runtime_error(
      std::string(
        "cv_bridge failed to convert encoding '") +
      image.encoding +
      "' to bgr8: " +
      ex.what());
  }

  if (!cv_ptr || cv_ptr->image.empty())
  {
    throw std::runtime_error(
      "cv_bridge produced an empty OpenCV image");
  }

  const int raw_left =
    static_cast<int>(
      std::floor(
        center_x -
        width * 0.5));

  const int raw_top =
    static_cast<int>(
      std::floor(
        center_y -
        height * 0.5));

  const int raw_right =
    static_cast<int>(
      std::ceil(
        center_x +
        width * 0.5));

  const int raw_bottom =
    static_cast<int>(
      std::ceil(
        center_y +
        height * 0.5));

  const int left =
    std::clamp(
      raw_left,
      0,
      cv_ptr->image.cols);

  const int top =
    std::clamp(
      raw_top,
      0,
      cv_ptr->image.rows);

  const int right =
    std::clamp(
      raw_right,
      0,
      cv_ptr->image.cols);

  const int bottom =
    std::clamp(
      raw_bottom,
      0,
      cv_ptr->image.rows);

  if (right <= left || bottom <= top)
  {
    throw std::runtime_error(
      "bbox is outside the image or has zero area "
      "after clipping");
  }

  const int crop_width =
    right - left;

  const int crop_height =
    bottom - top;

  if (
    crop_width < kMinimumCropWidth ||
    crop_height < kMinimumCropHeight)
  {
    throw std::runtime_error(
      "person crop is too small for Re-ID: " +
      std::to_string(crop_width) +
      "x" +
      std::to_string(crop_height));
  }

  const cv::Rect roi(
    left,
    top,
    crop_width,
    crop_height);

  cv::Mat crop =
    cv_ptr->image(roi).clone();

  if (crop.empty())
  {
    throw std::runtime_error(
      "OpenCV produced an empty person crop");
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Created Re-ID crop: "
    "source=%dx%d, source_encoding='%s', "
    "bbox_center=(%.2f, %.2f), "
    "bbox_size=(%.2f, %.2f), "
    "clipped_roi=(x=%d, y=%d, width=%d, height=%d), "
    "crop=%dx%d",
    cv_ptr->image.cols,
    cv_ptr->image.rows,
    image.encoding.c_str(),
    center_x,
    center_y,
    width,
    height,
    left,
    top,
    crop_width,
    crop_height,
    crop.cols,
    crop.rows);

  const sensor_msgs::msg::Image::SharedPtr crop_msg =
    cv_bridge::CvImage(
      image.header,
      "bgr8",
      crop).toImageMsg();

  if (
    !crop_msg ||
    crop_msg->width == 0 ||
    crop_msg->height == 0 ||
    crop_msg->step == 0 ||
    crop_msg->data.empty())
  {
    throw std::runtime_error(
      "failed to create a valid ROS Image "
      "from the person crop");
  }

  return *crop_msg;
}

bool QrbPeopleReid::timedOut() const
{
  if (state_ == State::IDLE)
  {
    return false;
  }

  return std::chrono::duration<double>(
    std::chrono::steady_clock::now() -
    request_start_).count() > timeout_sec_;
}

BT::NodeStatus QrbPeopleReid::finishNoMatch(
  const std::string& reason)
{
  setOutput(
    "matched",
    false);

  setOutput(
    "message",
    reason);

  /*
   * Avoid leaving the previous successful score or feature
   * visible after this run fails.
   */
  setOutput(
    "similarity_score",
    std::numeric_limits<float>::infinity());

  setOutput(
    "current_feature",
    std::vector<float>{});

  if (node_)
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "QrbPeopleReid failed: %s",
      reason.c_str());
  }

  resetRunState();

  return BT::NodeStatus::FAILURE;
}

void QrbPeopleReid::resetRunState()
{
  state_ = State::IDLE;

  candidate_index_ = 0;
  active_candidate_index_ = 0;

  best_score_ =
    std::numeric_limits<float>::infinity();

  best_index_ = 0;

  best_feature_.clear();
  candidate_feature_.clear();
}

}  // namespace qrb_people_reid
