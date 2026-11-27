// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "people_reid_node.hpp"
#include <cstring>
#include <chrono>

namespace qrb_ros {
namespace people_reid {

PeopleReIDNode::PeopleReIDNode(const rclcpp::NodeOptions & options)
: Node("people_reid_node", options), 
  is_waiting_tensor_(false),
  feature_ready_(false)
{
    RCLCPP_INFO(this->get_logger(), "Initializing People ReID Node...");

    // Separate callback groups to avoid service/subscriber scheduling interference
    service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    sub_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Create publisher for sending tensors to inference node
    tensor_pub_ = this->create_publisher<qrb_ros_tensor_list_msgs::msg::TensorList>(
        "/reid_inference_input_tensor", 10);

    // Create subscriber for receiving inference results
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = sub_cb_group_;
    tensor_sub_ = this->create_subscription<qrb_ros_tensor_list_msgs::msg::TensorList>(
        "/reid_tensor_output", 10, 
        std::bind(&PeopleReIDNode::tensor_callback, this, std::placeholders::_1),
        sub_options);

    // Create service servers
    extract_feature_srv_ = this->create_service<qrb_ros_people_reid::srv::ExtractFeature>(
        "extract_feature",
        std::bind(&PeopleReIDNode::extract_feature_callback, this, 
                  std::placeholders::_1, std::placeholders::_2));

    compute_similarity_srv_ = this->create_service<qrb_ros_people_reid::srv::ComputeSimilarity>(
        "compute_similarity",
        std::bind(&PeopleReIDNode::compute_similarity_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "People ReID Node initialized successfully");
    RCLCPP_INFO(this->get_logger(), "Services available:");
    RCLCPP_INFO(this->get_logger(), "  - /extract_feature");
    RCLCPP_INFO(this->get_logger(), "  - /compute_similarity");
}

// ==========================================
// Core Function: Extract Feature
// ==========================================
std::vector<float> PeopleReIDNode::extract_feature(const cv::Mat& image)
{
    std::unique_lock<std::mutex> lock(inference_mutex_);
    
    if (image.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Input image is empty");
        return std::vector<float>();
    }

    // Reset state
    feature_ready_ = false;
    latest_feature_.clear();

    // Send image to inference
    preprocess_and_send_to_inference(image);

    // Wait for inference result with timeout
    auto timeout = std::chrono::seconds(5);
    if (!inference_cv_.wait_for(lock, timeout, [this] { return feature_ready_; })) {
        RCLCPP_ERROR(this->get_logger(), "Timeout waiting for inference result");
        is_waiting_tensor_ = false;
        return std::vector<float>();
    }

    return latest_feature_;
}

// ==========================================
// Core Function: Compute Similarity
// ==========================================
float PeopleReIDNode::compute_similarity(
    const std::vector<float>& feature1, 
    const std::vector<float>& feature2)
{
    if (feature1.empty() || feature2.empty()) {
        RCLCPP_ERROR(this->get_logger(), "One or both feature vectors are empty");
        return -1.0f;
    }

    if (feature1.size() != feature2.size()) {
        RCLCPP_ERROR(this->get_logger(), 
            "Feature vectors have different sizes: %zu vs %zu", 
            feature1.size(), feature2.size());
        return -1.0f;
    }

    // Calculate cosine similarity
    float dot = 0.0f, norm_a = 0.0f, norm_b = 0.0f;
    for (size_t i = 0; i < feature1.size(); ++i) {
        dot += feature1[i] * feature2[i];
        norm_a += feature1[i] * feature1[i];
        norm_b += feature2[i] * feature2[i];
    }

    if (norm_a == 0.0f || norm_b == 0.0f) {
        RCLCPP_ERROR(this->get_logger(), "Feature vector has zero norm");
        return -1.0f;
    }

    // Return cosine distance (1 - cosine_similarity)
    float cosine_similarity = dot / (std::sqrt(norm_a) * std::sqrt(norm_b));
    float cosine_distance = 1.0f - cosine_similarity;

    RCLCPP_DEBUG(this->get_logger(), "Cosine distance: %.4f", cosine_distance);
    
    return cosine_distance;
}

// ==========================================
// Helper: Preprocess and Send to Inference
// ==========================================
void PeopleReIDNode::preprocess_and_send_to_inference(const cv::Mat& image)
{
    // Preprocess image for ReID model (OSNet expects 256x128 RGB)
    cv::Mat processed;
    cv::cvtColor(image, processed, cv::COLOR_BGR2RGB);
    cv::resize(processed, processed, cv::Size(128, 256), cv::INTER_LINEAR);
    processed.convertTo(processed, CV_32FC3, 1.0 / 255.0);

    // Set mean and standard deviation
    cv::Scalar mean(0.485, 0.456, 0.406);
    cv::Scalar std(0.229, 0.224, 0.225);
    processed = (processed - mean) / std;

    // Convert to NCHW format (channels first)
    int channels = processed.channels();
    std::vector<float> input_tensor;
    
    if (processed.isContinuous()) {
        for (int c = 0; c < channels; ++c) {
            for (size_t k = 0; k < processed.total(); ++k) {
                input_tensor.emplace_back(((float*)processed.data)[k * channels + c]);
            }
        }
    } else {
        for (int c = 0; c < channels; ++c) {
            for (int i = 0; i < processed.rows; ++i) {
                const float* row_ptr = processed.ptr<float>(i);
                for (int j = 0; j < processed.cols; ++j) {
                    input_tensor.emplace_back(row_ptr[j * channels + c]);
                }
            }
        }
    }

    // Create tensor message
    qrb_ros_tensor_list_msgs::msg::Tensor msg_tensor;
    msg_tensor.data.resize(input_tensor.size() * sizeof(float));
    msg_tensor.name = "image_tensor";
    msg_tensor.data_type = 2;  // float32
    std::memcpy(msg_tensor.data.data(), input_tensor.data(), msg_tensor.data.size());

    // Set waiting flag BEFORE publishing to avoid race condition
    // (inference might be very fast and publish result before flag is set)
    is_waiting_tensor_ = true;

    // Publish tensor
    auto msg_tensor_list = std::make_unique<qrb_ros_tensor_list_msgs::msg::TensorList>();
    msg_tensor_list->tensor_list.push_back(std::move(msg_tensor));
    tensor_pub_->publish(std::move(msg_tensor_list));

    RCLCPP_DEBUG(this->get_logger(), "Tensor sent to inference");
}

// ==========================================
// Helper: Parse Tensor to Float Vector
// ==========================================
std::vector<float> PeopleReIDNode::parse_tensor_to_float(
    const qrb_ros_tensor_list_msgs::msg::Tensor& tensor)
{
    std::vector<float> features;
    
    if (tensor.data_type != 2) {  // 2 = float32
        RCLCPP_ERROR(this->get_logger(), 
            "Expected float32 (2) tensor, but got %d", tensor.data_type);
        return features;
    }

    if (tensor.data.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Tensor data is empty");
        return features;
    }

    size_t num_elements = tensor.data.size() / sizeof(float);
    features.resize(num_elements);
    std::memcpy(features.data(), tensor.data.data(), num_elements * sizeof(float));

    RCLCPP_DEBUG(this->get_logger(), "Parsed feature vector with %zu elements", num_elements);
    
    return features;
}

// ==========================================
// Callback: Tensor Result from Inference
// ==========================================
void PeopleReIDNode::tensor_callback(
    const qrb_ros_tensor_list_msgs::msg::TensorList::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(),
        "Tensor callback received: is_waiting_tensor_=%d, feature_ready_=%d, tensor_count=%zu",
        is_waiting_tensor_, feature_ready_, msg->tensor_list.size());
    
    std::lock_guard<std::mutex> lock(inference_mutex_);

    if (!is_waiting_tensor_) {
        RCLCPP_WARN(this->get_logger(), "Received tensor but not waiting, ignoring");
        return;
    }

    if (msg->tensor_list.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty TensorList");
        is_waiting_tensor_ = false;
        feature_ready_ = true;
        inference_cv_.notify_one();
        return;
    }

    // Parse feature vector from first tensor
    latest_feature_ = parse_tensor_to_float(msg->tensor_list[0]);
    
    is_waiting_tensor_ = false;
    feature_ready_ = true;
    inference_cv_.notify_one();

    RCLCPP_DEBUG(this->get_logger(), "Feature extraction completed");
}

// ==========================================
// Service Callback: Extract Feature
// ==========================================
void PeopleReIDNode::extract_feature_callback(
    const std::shared_ptr<qrb_ros_people_reid::srv::ExtractFeature::Request> request,
    std::shared_ptr<qrb_ros_people_reid::srv::ExtractFeature::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Extract feature service called");

    try {
        // Convert ROS image to OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(request->image, "bgr8");
        
        // Extract feature
        std::vector<float> feature = extract_feature(cv_ptr->image);

        if (feature.empty()) {
            response->success = false;
            response->message = "Failed to extract feature";
            RCLCPP_ERROR(this->get_logger(), "Feature extraction failed");
        } else {
            response->feature = feature;
            response->success = true;
            response->message = "Feature extracted successfully";
            RCLCPP_INFO(this->get_logger(), 
                "Feature extracted: %zu dimensions", feature.size());
        }
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Exception: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "Exception in extract_feature: %s", e.what());
    }
}

// ==========================================
// Service Callback: Compute Similarity
// ==========================================
void PeopleReIDNode::compute_similarity_callback(
    const std::shared_ptr<qrb_ros_people_reid::srv::ComputeSimilarity::Request> request,
    std::shared_ptr<qrb_ros_people_reid::srv::ComputeSimilarity::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Compute similarity service called");

    try {
        float similarity = compute_similarity(request->feature1, request->feature2);

        if (similarity < 0.0f) {
            response->success = false;
            response->message = "Failed to compute similarity";
            response->similarity = -1.0f;
            RCLCPP_ERROR(this->get_logger(), "Similarity computation failed");
        } else {
            response->similarity = similarity;
            response->success = true;
            response->message = "Similarity computed successfully";
            RCLCPP_INFO(this->get_logger(), 
                "Similarity computed: %.4f", similarity);
        }
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Exception: ") + e.what();
        response->similarity = -1.0f;
        RCLCPP_ERROR(this->get_logger(), "Exception in compute_similarity: %s", e.what());
    }
}

}  // namespace people_reid
}  // namespace qrb_ros

// ==========================================
// Register ROS 2 Component
// ==========================================
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::people_reid::PeopleReIDNode)
