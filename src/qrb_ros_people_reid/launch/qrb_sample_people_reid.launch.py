# Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

def generate_launch_description():

    # ==========================================
    # 1. Default Parameters
    # ==========================================
    
    # Re-ID model parameters
    default_reid_model_path = os.path.join(
        os.path.dirname(__file__), "..", "osnet.bin"
    )
    default_backend_option = "/usr/lib/libQnnHtp.so"
    
    # ==========================================
    # 2. Declare Launch Arguments
    # ==========================================
    
    reid_model_arg = DeclareLaunchArgument(
        "reid_model_path",
        default_value=default_reid_model_path,
        description="Path to the ReID model file (osnet.bin)"
    )
    
    backend_option_arg = DeclareLaunchArgument(
        "backend_option",
        default_value=default_backend_option,
        description="QNN backend library path"
    )

    # ==========================================
    # 3. Define Composable Nodes
    # ==========================================

    # Re-ID Service Node: Provides extract_feature and compute_similarity services
    people_reid_node = ComposableNode(
        package="qrb_ros_people_reid",
        plugin="qrb_ros::people_reid::PeopleReIDNode",
        name="people_reid_node",
        parameters=[],
        # This node publishes to: /reid_inference_input_tensor
        # This node subscribes to: /reid_tensor_output
        # This node provides services:
        #   - /extract_feature
        #   - /compute_similarity
    )

    # Re-ID Inference Node: Performs neural network inference
    reid_inference_node = ComposableNode(
        package='qrb_ros_nn_inference',
        plugin='qrb_ros::nn_inference::QrbRosInferenceNode',
        name='reid_inference_node',
        parameters=[{
            'backend_option': LaunchConfiguration("backend_option"),
            'model_path': LaunchConfiguration("reid_model_path")
        }],
        remappings=[
            ('qrb_inference_input_tensor', '/reid_inference_input_tensor'),
            ('qrb_inference_output_tensor', '/reid_tensor_output')
        ]
    )

    # ==========================================
    # 4. Assemble Container
    # ==========================================
    # Use multi-threaded container to avoid deadlock when service callback
    # waits for subscription callback in the same executor
    container = ComposableNodeContainer(
        name="people_reid_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",  # Multi-threaded executor
        composable_node_descriptions=[
            people_reid_node,
            reid_inference_node
        ],
        output="screen",
    )

    return LaunchDescription([
        # Declare all arguments
        reid_model_arg,
        backend_option_arg,
        
        # Start container
        container,
    ])
