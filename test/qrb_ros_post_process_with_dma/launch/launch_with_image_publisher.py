# Copyright (c) 2026 Qualcomm Innovation Center, Inc.
# SPDX-License-Identifier: BSD-3-Clause-Clear

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    post_process_package_name = 'qrb_ros_post_process_with_dma'
    post_process_package_path = get_package_share_directory(post_process_package_name)

    image_path_arg = DeclareLaunchArgument(
        'image_path',
        default_value=os.path.join(post_process_package_path, 'resource', 'input_image.jpg'),
        description='Path to the input image file'
    )
    image_path = LaunchConfiguration('image_path')
    LogInfo(msg=['IMAGE_PATH: ', image_path])

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value="/home/ubuntu/work/AI-models/Depth-Anything-V2.bin",
        description='Path to the model file'
    )
    model_path = LaunchConfiguration('model_path')
    LogInfo(msg=['MODEL_PATH: ', model_path])

    # Keep consistent topic wiring:
    # - image_publisher publishes /image_raw without namespace
    # - all three nodes run under namespace in the same container
    namespace = "sample_container"

    image_publish_node = Node(
        package='image_publisher',
        executable='image_publisher_node',
        name='image_publisher_node',
        output='screen',
        parameters=[
            {'filename': image_path},
            {'rate': 10.0},
        ],
    )

    # qrb_ros_pre_process_with_dma component
    pre_process_node = ComposableNode(
        package="qrb_ros_pre_process_with_dma",
        namespace=namespace,
        plugin="qrb_ros_pre_process_with_dma::QrbRosPreProcessWithDmaNode",
        name="pre_process_node",
        parameters=[{
            "input_width": 518,
            "input_height": 518
        }],
    )

    # qrb_ros_nn_inference component
    nn_inference_node = ComposableNode(
        package="qrb_ros_nn_inference",
        namespace=namespace,
        plugin="qrb_ros::nn_inference::QrbRosInferenceNode",
        name="nn_inference_node",
        parameters=[{
            "backend_option": "/usr/lib/libQnnHtp.so",
            "model_path": model_path,
            "log_level": "warn"
        }],
    )

    # qrb_ros_post_process_with_dma component
    post_process_node = ComposableNode(
        package="qrb_ros_post_process_with_dma",
        namespace=namespace,
        plugin="qrb_ros_post_process_with_dma::QrbRosPostProcessWithDmaNode",
        name="post_process_node",
        parameters=[os.path.join(post_process_package_path, 'config', 'config.yaml')],
    )

    # All three nodes in the same container for zero-copy DMA-BUF sharing
    container = ComposableNodeContainer(
        name="container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        composable_node_descriptions=[
            pre_process_node,
            nn_inference_node,
            post_process_node,
        ],
        sigterm_timeout="3",
        sigkill_timeout="5",
    )

    return LaunchDescription([
        image_path_arg,
        model_path_arg,
        image_publish_node,
        container,
    ])
