// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef PEOPLE_REID_NODE_HPP_
#define PEOPLE_REID_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <qrb_ros_tensor_list_msgs/msg/tensor_list.hpp>
#include <qrb_ros_tensor_list_msgs/msg/tensor.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include "qrb_ros_people_reid/srv/extract_feature.hpp"
#include "qrb_ros_people_reid/srv/compute_similarity.hpp"

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <rclcpp/callback_group.hpp>

namespace qrb_ros {
namespace people_reid {

class PeopleReIDNode : public rclcpp::Node
{
public:
    explicit PeopleReIDNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~PeopleReIDNode() = default;

private:
    // Mutex for thread safety
    std::mutex inference_mutex_;
    std::condition_variable inference_cv_;
    bool is_waiting_tensor_;
    std::vector<float> latest_feature_;
    bool feature_ready_;
    
    // Callback groups
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;
    rclcpp::CallbackGroup::SharedPtr sub_cb_group_;
    
    // Publishers and Subscribers for internal inference pipeline
    rclcpp::Publisher<qrb_ros_tensor_list_msgs::msg::TensorList>::SharedPtr tensor_pub_;
    rclcpp::Subscription<qrb_ros_tensor_list_msgs::msg::TensorList>::SharedPtr tensor_sub_;
    
    // Service servers
    rclcpp::Service<qrb_ros_people_reid::srv::ExtractFeature>::SharedPtr extract_feature_srv_;
    rclcpp::Service<qrb_ros_people_reid::srv::ComputeSimilarity>::SharedPtr compute_similarity_srv_;
    
    // Core functions
    std::vector<float> extract_feature(const cv::Mat& image);
    float compute_similarity(const std::vector<float>& feature1, const std::vector<float>& feature2);
    
    // Helper functions
    void preprocess_and_send_to_inference(const cv::Mat& image);
    std::vector<float> parse_tensor_to_float(const qrb_ros_tensor_list_msgs::msg::Tensor& tensor);
    
    // Callbacks
    void tensor_callback(const qrb_ros_tensor_list_msgs::msg::TensorList::SharedPtr msg);
    void extract_feature_callback(
        const std::shared_ptr<qrb_ros_people_reid::srv::ExtractFeature::Request> request,
        std::shared_ptr<qrb_ros_people_reid::srv::ExtractFeature::Response> response);
    void compute_similarity_callback(
        const std::shared_ptr<qrb_ros_people_reid::srv::ComputeSimilarity::Request> request,
        std::shared_ptr<qrb_ros_people_reid::srv::ComputeSimilarity::Response> response);
};

}  // namespace people_reid
}  // namespace qrb_ros

#endif  // PEOPLE_REID_NODE_HPP_
