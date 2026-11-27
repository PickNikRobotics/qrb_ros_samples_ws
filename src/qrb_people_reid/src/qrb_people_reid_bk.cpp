#include <qrb_people_reid/qrb_people_reid.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <future>
#include <limits>
#include <stdexcept>

namespace qrb_people_reid
{
namespace
{
constexpr char kImagePort[] = "image";
constexpr char kDetectionsPort[] = "detections";
constexpr char kReferenceFeaturePort[] = "reference_feature";
constexpr char kMatchThresholdPort[] = "match_threshold";
constexpr char kTemplateUpdateThresholdPort[] = "template_update_threshold";
constexpr char kTimeoutPort[] = "timeout_sec";
}

QrbPeopleReid::QrbPeopleReid(
  const std::string& name, const BT::NodeConfig& config,
  const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : BT::StatefulActionNode(name, config),
    shared_resources_(shared_resources),
    node_(shared_resources->node)
{
  extract_client_ = node_->create_client<qrb_ros_people_reid::srv::ExtractFeature>("/extract_feature");
  similarity_client_ = node_->create_client<qrb_ros_people_reid::srv::ComputeSimilarity>("/compute_similarity");
}

BT::PortsList QrbPeopleReid::providedPorts()
{
  return {
    BT::InputPort<sensor_msgs::msg::Image>(kImagePort, "RGB image containing detected people"),
    BT::InputPort<vision_msgs::msg::Detection2DArray>(kDetectionsPort, "YOLO person detections"),
    BT::BidirectionalPort<std::vector<float>>(kReferenceFeaturePort, "Persistent target Re-ID template"),
    BT::InputPort<double>(kMatchThresholdPort, 0.35, "Maximum distance accepted as a match"),
    BT::InputPort<double>(kTemplateUpdateThresholdPort, 0.20, "Maximum distance for template update"),
    BT::InputPort<double>(kTimeoutPort, 3.0, "Timeout for each service request"),
    BT::OutputPort<float>("similarity_score", "Best Re-ID distance; lower is more similar"),
    BT::OutputPort<bool>("matched", "True when the target was initialized or matched"),
    BT::OutputPort<vision_msgs::msg::Detection2D>("target_detection", "Matched person detection"),
    BT::OutputPort<std::vector<float>>("current_feature", "Feature extracted for the selected target"),
    BT::OutputPort<std::string>("message", "Diagnostic result")
  };
}

BT::NodeStatus QrbPeopleReid::onStart()
{
  resetRunState();
  auto image = getInput<sensor_msgs::msg::Image>(kImagePort);
  auto detections = getInput<vision_msgs::msg::Detection2DArray>(kDetectionsPort);
  if (!image || !detections) {
    return finishNoMatch("Missing required input: image or detections");
  }
  image_ = image.value();
  detections_ = detections.value();
  getInput(kReferenceFeaturePort, reference_feature_);
  getInput(kMatchThresholdPort, match_threshold_);
  getInput(kTemplateUpdateThresholdPort, template_update_threshold_);
  getInput(kTimeoutPort, timeout_sec_);

  if (image_.data.empty() || image_.width == 0 || image_.height == 0) {
    return finishNoMatch("Input image is empty");
  }
  if (detections_.detections.empty()) {
    return finishNoMatch("No person detections received");
  }
  if (!extract_client_->service_is_ready()) {
    return finishNoMatch("/extract_feature service is not ready");
  }
  if (!reference_feature_.empty() && !similarity_client_->service_is_ready()) {
    return finishNoMatch("/compute_similarity service is not ready");
  }
  return beginNextCandidate() ? BT::NodeStatus::RUNNING
                              : finishNoMatch("No valid person crop in detections");
}

BT::NodeStatus QrbPeopleReid::onRunning()
{
  if (timedOut()) {
    return finishNoMatch("People Re-ID service call timed out");
  }

  if (state_ == State::WAIT_EXTRACT &&
      extract_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    const auto response = extract_future_.get();
    if (!response || !response->success || response->feature.empty()) {
      RCLCPP_WARN(node_->get_logger(), "Feature extraction failed for candidate %zu: %s",
                  active_candidate_index_, response ? response->message.c_str() : "null response");
      return beginNextCandidate() ? BT::NodeStatus::RUNNING : finishNoMatch("No candidate feature could be extracted");
    }
    candidate_feature_ = response->feature;

    // Empty reference means target initialization. The first valid candidate becomes the reference.
    if (reference_feature_.empty()) {
      reference_feature_ = candidate_feature_;
      setOutput(kReferenceFeaturePort, reference_feature_);
      setOutput("current_feature", candidate_feature_);
      setOutput("similarity_score", 0.0F);
      setOutput("matched", true);
      setOutput("target_detection", detections_.detections.at(active_candidate_index_));
      setOutput("message", std::string("Reference feature initialized"));
      resetRunState();
      return BT::NodeStatus::SUCCESS;
    }

    auto request = std::make_shared<qrb_ros_people_reid::srv::ComputeSimilarity::Request>();
    request->feature1 = reference_feature_;
    request->feature2 = candidate_feature_;
    similarity_future_ = similarity_client_->async_send_request(request);
    request_start_ = std::chrono::steady_clock::now();
    state_ = State::WAIT_SIMILARITY;
    return BT::NodeStatus::RUNNING;
  }

  if (state_ == State::WAIT_SIMILARITY &&
      similarity_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    const auto response = similarity_future_.get();
    if (response && response->success && std::isfinite(response->similarity)) {
      if (response->similarity < best_score_) {
        best_score_ = response->similarity;
        best_index_ = active_candidate_index_;
        best_feature_ = candidate_feature_;
      }
    } else {
      RCLCPP_WARN(node_->get_logger(), "Similarity failed for candidate %zu: %s",
                  active_candidate_index_, response ? response->message.c_str() : "null response");
    }

    if (beginNextCandidate()) {
      return BT::NodeStatus::RUNNING;
    }

    const bool matched = std::isfinite(best_score_) && best_score_ <= match_threshold_;
    setOutput("matched", matched);
    setOutput("similarity_score", best_score_);
    if (!matched) {
      return finishNoMatch("No detection passed the Re-ID match threshold");
    }

    setOutput("target_detection", detections_.detections.at(best_index_));
    setOutput("current_feature", best_feature_);
    if (best_score_ <= template_update_threshold_) {
      reference_feature_ = best_feature_;
      setOutput(kReferenceFeaturePort, reference_feature_);
      setOutput("message", std::string("Target matched and reference feature updated"));
    } else {
      setOutput(kReferenceFeaturePort, reference_feature_);
      setOutput("message", std::string("Target matched; reference feature kept"));
    }
    resetRunState();
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void QrbPeopleReid::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "QrbPeopleReid halted");
  resetRunState();
}

bool QrbPeopleReid::beginNextCandidate()
{
  while (candidate_index_ < detections_.detections.size()) {
    active_candidate_index_ = candidate_index_++;
    try {
      auto request = std::make_shared<qrb_ros_people_reid::srv::ExtractFeature::Request>();
      request->image = cropPerson(image_, detections_.detections.at(active_candidate_index_));
      extract_future_ = extract_client_->async_send_request(request);
      request_start_ = std::chrono::steady_clock::now();
      state_ = State::WAIT_EXTRACT;
      return true;
    } catch (const std::exception& ex) {
      RCLCPP_WARN(node_->get_logger(), "Skipping invalid candidate %zu: %s", active_candidate_index_, ex.what());
    }
  }
  return false;
}

sensor_msgs::msg::Image QrbPeopleReid::cropPerson(
  const sensor_msgs::msg::Image& image, const vision_msgs::msg::Detection2D& detection) const
{
  auto cv_ptr = cv_bridge::toCvCopy(image, "bgr8");
  const auto& bbox = detection.bbox;
  const int x = std::max(0, static_cast<int>(std::lround(bbox.center.position.x - bbox.size_x / 2.0)));
  const int y = std::max(0, static_cast<int>(std::lround(bbox.center.position.y - bbox.size_y / 2.0)));
  const int right = std::min(cv_ptr->image.cols, static_cast<int>(std::lround(bbox.center.position.x + bbox.size_x / 2.0)));
  const int bottom = std::min(cv_ptr->image.rows, static_cast<int>(std::lround(bbox.center.position.y + bbox.size_y / 2.0)));
  if (right <= x || bottom <= y) throw std::runtime_error("bbox is outside the image or has zero area");
  cv::Mat crop = cv_ptr->image(cv::Rect(x, y, right - x, bottom - y)).clone();
  return *cv_bridge::CvImage(image.header, "bgr8", crop).toImageMsg();
}

bool QrbPeopleReid::timedOut() const
{
  return state_ != State::IDLE &&
    std::chrono::duration<double>(std::chrono::steady_clock::now() - request_start_).count() > timeout_sec_;
}

BT::NodeStatus QrbPeopleReid::finishNoMatch(const std::string& reason)
{
  setOutput("matched", false);
  setOutput("message", reason);
  RCLCPP_WARN(node_->get_logger(), "%s", reason.c_str());
  resetRunState();
  return BT::NodeStatus::FAILURE;
}

void QrbPeopleReid::resetRunState()
{
  state_ = State::IDLE;
  candidate_index_ = 0;
  active_candidate_index_ = 0;
  best_score_ = std::numeric_limits<float>::infinity();
  best_index_ = 0;
  best_feature_.clear();
  candidate_feature_.clear();
}
}  // namespace qrb_people_reid
