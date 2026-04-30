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

## QuickStart

1. download model
   ```bash
   mkdir -p /home/ubuntu/work/AI-models && cd /home/ubuntu/work/AI-models && \
   wget https://huggingface.co/qualcomm/Depth-Anything-V2/resolve/19ce3645e11de17eed7e869eebcc07dd352834f3/Depth-Anything-V2.bin?download=true -O Depth-Anything-V2.bin
   ```
2. download source code
   ```bash
   mkdir -p /home/ubuntu/work/ros_ws/src && \
   cd /home/ubuntu/work/ros_ws/src && \
   git clone https://github.com/qualcomm-qrb-ros/qrb_ros_nn_inference && \
   git clone https://github.com/qualcomm-qrb-ros/qrb_ros_interfaces && \
   ```
3. build qrb_ros_post_process_with_dma
   ```bash
   sudo apt install -y ros-dev-tools && \
   cd qrb_ros_nn_inference && \
   rm test/qrb_ros_pre_process_with_dma/COLCON_IGNORE && \
   rm test/qrb_ros_post_process_with_dma/COLCON_IGNORE && \
   cd /home/ubuntu/work/ros_ws && \
   colcon build --packages-up-to qrb_ros_pre_process_with_dma qrb_ros_post_process_with_dma
   ```
4. run the inference pipeline
   ```bash
   source install/setup.bash && ros2 launch qrb_ros_post_process_with_dma launch_with_image_publisher.py
   ```
5. post_process node saves depth maps to `/tmp/nn_output/`:
- `{frame_count}_gray.png`: Grayscale normalized depth map
- `{frame_count}_color.png`: Colorized depth map using INFERNO colormap

## Topics

### Subscribed Topics
- `qrb_inference_output_tensor` (qrb_ros_tensor_list_msgs/TensorList): Inference output tensors

### Published Topics
- `depth_map` (sensor_msgs/Image): Colorized depth map images

## Parameters

- `input_width` (int, default: 518): Model input width
- `input_height` (int, default: 518): Model input height

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
- OpenCV