# qrb_ros_post_process_with_dma

## Overview

This package provides a ROS2 node for post-processing depth estimation inference results using DMA-BUF (Direct Memory Access Buffer) for efficient zero-copy data transfer.

## Description

The `qrb_ros_post_process_with_dma` node is responsible for:
- Subscribing to inference output tensors from the `qrb_inference_output_tensor` topic
- Reading depth data from DMA-BUF memory
- Post-processing depth maps (rescaling, cropping, colorization)
- Publishing colorized depth maps to the `depth_map` topic

This node is designed to work in conjunction with:
- `qrb_ros_pre_process_with_dma`: Pre-processes input images
- `qrb_ros_nn_inference`: Performs neural network inference

## Features

- **Zero-copy DMA-BUF transfer**: Reads inference results directly from DMA-BUF memory
- **Post-processing pipeline**: Includes rescaling, cropping, normalization, and colorization
- **Image saving**: Saves both grayscale and colored depth maps to disk
- **Composable node**: Can be loaded into a component container for intra-process communication

## Topics

### Subscribed Topics
- `qrb_inference_output_tensor` (qrb_ros_tensor_list_msgs/TensorList): Inference output tensors

### Published Topics
- `depth_map` (sensor_msgs/Image): Colorized depth map images

## Parameters

- `input_width` (int, default: 518): Model input width
- `input_height` (int, default: 518): Model input height

## Launch Files

### launch_with_image_publisher.py

This launch file demonstrates the complete depth estimation pipeline by launching all three nodes in a single composable node container:

1. **image_publisher_node**: Publishes test images
2. **pre_process_node**: Pre-processes images and publishes tensors
3. **nn_inference_node**: Performs depth estimation inference
4. **post_process_node**: Post-processes and visualizes results

**Usage:**
```bash
ros2 launch qrb_ros_post_process_with_dma launch_with_image_publisher.py
```

**Launch Arguments:**
- `image_path`: Path to input image (default: uses image from new_sample_depth_estimation package)
- `model_path`: Path to the depth estimation model file (default: /home/ubuntu/work/AI-models/Depth-Anything-V2.bin)

## Architecture

All three nodes run in the same component container to enable zero-copy DMA-BUF sharing:

```
image_publisher -> /image_raw
                      |
                      v
              pre_process_node -> qrb_inference_input_tensor
                                         |
                                         v
                                  nn_inference_node -> qrb_inference_output_tensor
                                                              |
                                                              v
                                                       post_process_node -> depth_map
```

## Dependencies

- rclcpp
- sensor_msgs
- cv_bridge
- qrb_ros_tensor_list_msgs
- qrb_ros_nn_inference
- lib_mem_dmabuf
- OpenCV

## Output

The node saves depth maps to `/home/ubuntu/ros-ws/nn_output/`:
- `{frame_count}_gray.png`: Grayscale normalized depth map
- `{frame_count}_color.png`: Colorized depth map using INFERNO colormap
