// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "qrb_ros_people_reid/srv/extract_feature.hpp"
#include "qrb_ros_people_reid/srv/compute_similarity.hpp"

class ReIDTestClient : public rclcpp::Node
{
public:
    ReIDTestClient() : Node("reid_test_client")
    {
        extract_client_ = this->create_client<qrb_ros_people_reid::srv::ExtractFeature>(
            "extract_feature");
        similarity_client_ = this->create_client<qrb_ros_people_reid::srv::ComputeSimilarity>(
            "compute_similarity");
    }

    std::vector<float> extract_feature(const std::string& image_path)
    {
        // Load image
        cv::Mat image = cv::imread(image_path);
        if (image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load image: %s", image_path.c_str());
            return std::vector<float>();
        }

        RCLCPP_INFO(this->get_logger(), "Loaded image: %s (%dx%d)", 
            image_path.c_str(), image.cols, image.rows);

        // Convert to ROS message
        auto ros_image = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();

        // Wait for service
        while (!extract_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service");
                return std::vector<float>();
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for extract_feature service...");
        }

        // Create request
        auto request = std::make_shared<qrb_ros_people_reid::srv::ExtractFeature::Request>();
        request->image = *ros_image;

        // Call service
        RCLCPP_INFO(this->get_logger(), "Calling extract_feature service...");
        auto future = extract_client_->async_send_request(request);

        // Wait for response
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Feature extracted: %zu dimensions", 
                    response->feature.size());
                return response->feature;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Feature extraction failed: %s", 
                    response->message.c_str());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }

        return std::vector<float>();
    }

    float compute_similarity(const std::vector<float>& feature1, 
                            const std::vector<float>& feature2)
    {
        // Wait for service
        while (!similarity_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service");
                return -1.0f;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for compute_similarity service...");
        }

        // Create request
        auto request = std::make_shared<qrb_ros_people_reid::srv::ComputeSimilarity::Request>();
        request->feature1 = feature1;
        request->feature2 = feature2;

        // Call service
        RCLCPP_INFO(this->get_logger(), "Calling compute_similarity service...");
        auto future = similarity_client_->async_send_request(request);

        // Wait for response
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Similarity computed: %.4f", 
                    response->similarity);
                return response->similarity;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Similarity computation failed: %s", 
                    response->message.c_str());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }

        return -1.0f;
    }

private:
    rclcpp::Client<qrb_ros_people_reid::srv::ExtractFeature>::SharedPtr extract_client_;
    rclcpp::Client<qrb_ros_people_reid::srv::ComputeSimilarity>::SharedPtr similarity_client_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <image1_path> <image2_path>" << std::endl;
        std::cerr << "Example: " << argv[0] << " person1.jpg person2.jpg" << std::endl;
        return 1;
    }

    auto client = std::make_shared<ReIDTestClient>();

    // Extract features from both images
    RCLCPP_INFO(client->get_logger(), "=== Extracting features from image 1 ===");
    auto feature1 = client->extract_feature(argv[1]);
    
    if (feature1.empty()) {
        RCLCPP_ERROR(client->get_logger(), "Failed to extract feature from image 1");
        return 1;
    }

    RCLCPP_INFO(client->get_logger(), "=== Extracting features from image 2 ===");
    auto feature2 = client->extract_feature(argv[2]);
    
    if (feature2.empty()) {
        RCLCPP_ERROR(client->get_logger(), "Failed to extract feature from image 2");
        return 1;
    }

    // Compute similarity
    RCLCPP_INFO(client->get_logger(), "=== Computing similarity ===");
    float similarity = client->compute_similarity(feature1, feature2);

    if (similarity >= 0.0f) {
        RCLCPP_INFO(client->get_logger(), 
            "\n========================================");
        RCLCPP_INFO(client->get_logger(), 
            "RESULT: Cosine Distance = %.4f", similarity);
        RCLCPP_INFO(client->get_logger(), 
            "  (0.0 = identical, 2.0 = opposite)");
        RCLCPP_INFO(client->get_logger(), 
            "========================================\n");
        
        if (similarity < 0.3f) {
            RCLCPP_INFO(client->get_logger(), "Interpretation: Same person (high confidence)");
        } else if (similarity < 0.6f) {
            RCLCPP_INFO(client->get_logger(), "Interpretation: Possibly same person");
        } else {
            RCLCPP_INFO(client->get_logger(), "Interpretation: Different persons");
        }
    } else {
        RCLCPP_ERROR(client->get_logger(), "Failed to compute similarity");
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
