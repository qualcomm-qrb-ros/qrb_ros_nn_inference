# qrb_ros_pre_process_with_dma

## Overview

This package provides a ROS2 node for pre-processing images for depth estimation using DMA-BUF (Direct Memory Access Buffer) for efficient zero-copy data transfer.

## Description

The `qrb_ros_pre_process_with_dma` node is responsible for:
- Subscribing to raw image data from the `/image_raw` topic
- Pre-processing images (resizing, padding, normalization)
- Allocating DMA-BUF memory using RPCMEM (ION) for zero-copy transfer
- Publishing processed tensor data to the `qrb_inference_input_tensor` topic

This node is designed to work in conjunction with:
- `qrb_ros_nn_inference`: Performs neural network inference
- `qrb_ros_post_process_with_dma`: Post-processes inference results

## Topics

### Subscribed Topics
- `/image_raw` (sensor_msgs/Image): Raw input images

### Published Topics
- `qrb_inference_input_tensor` (qrb_ros_tensor_list_msgs/TensorList): Preprocessed tensor data

## Parameters

- `input_width` (int, default: 518): Model input width
- `input_height` (int, default: 518): Model input height

## Usage

This node is typically launched as part of a composable node container along with the inference and post-processing nodes.

See the launch file in `qrb_ros_post_process_with_dma` package for an example.

## Dependencies

- rclcpp
- sensor_msgs
- cv_bridge
- qrb_ros_tensor_list_msgs
- OpenCV
- libcdsprpc (for RPCMEM/ION support)
