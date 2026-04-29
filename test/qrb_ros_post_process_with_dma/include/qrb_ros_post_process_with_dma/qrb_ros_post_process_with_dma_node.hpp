// Copyright (c) 2026 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_POST_PROCESS_WITH_DMA__QRB_ROS_POST_PROCESS_WITH_DMA_NODE_HPP_
#define QRB_ROS_POST_PROCESS_WITH_DMA__QRB_ROS_POST_PROCESS_WITH_DMA_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "qrb_ros_tensor_list_msgs/msg/tensor_list.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace qrb_ros_post_process_with_dma
{

class QrbRosPostProcessWithDmaNode : public rclcpp::Node
{
public:
  explicit QrbRosPostProcessWithDmaNode(const rclcpp::NodeOptions & options);
  ~QrbRosPostProcessWithDmaNode() override;

private:
  using Tensor = qrb_ros_tensor_list_msgs::msg::Tensor;
  using TensorList = qrb_ros_tensor_list_msgs::msg::TensorList;

  void infer_callback(const TensorList & msg);

  // Post-process (same math as python sample)
  cv::Mat postprocess(const float * depth,
      int orig_w,
      int orig_h,
      float scale,
      int pad_left,
      int pad_top) const;

private:
  rclcpp::Subscription<TensorList>::SharedPtr infer_sub_{ nullptr };
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_{ nullptr };

  // state from last preprocess (these need to be shared between pre and post process nodes)
  // For now, using fixed values based on typical image size
  int last_orig_w_{ 640 };
  int last_orig_h_{ 480 };
  float last_scale_{ 1.0f };
  int last_pad_left_{ 0 };
  int last_pad_top_{ 0 };

  // model input config (NHWC float32)
  int in_w_{ 518 };
  int in_h_{ 518 };
  int in_c_{ 3 };

  // RPCMEM library handle for memory management
  void * rpcmem_lib_{ nullptr };
};

}  // namespace qrb_ros_post_process_with_dma

#endif  // QRB_ROS_POST_PROCESS_WITH_DMA__QRB_ROS_POST_PROCESS_WITH_DMA_NODE_HPP_
