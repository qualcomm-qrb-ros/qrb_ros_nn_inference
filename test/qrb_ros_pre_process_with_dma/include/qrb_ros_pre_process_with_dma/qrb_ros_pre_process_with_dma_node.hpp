// Copyright (c) 2026 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_PRE_PROCESS_WITH_DMA__QRB_ROS_PRE_PROCESS_WITH_DMA_NODE_HPP_
#define QRB_ROS_PRE_PROCESS_WITH_DMA__QRB_ROS_PRE_PROCESS_WITH_DMA_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "qrb_ros_tensor_list_msgs/msg/tensor_list.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace qrb_ros_pre_process_with_dma
{

class QrbRosPreProcessWithDmaNode : public rclcpp::Node
{
public:
  explicit QrbRosPreProcessWithDmaNode(const rclcpp::NodeOptions & options);
  ~QrbRosPreProcessWithDmaNode() override;

private:
  using Tensor = qrb_ros_tensor_list_msgs::msg::Tensor;
  using TensorList = qrb_ros_tensor_list_msgs::msg::TensorList;

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  // Pre-process (same math as python sample)
  cv::Mat nv12_to_bgr(const uint8_t * nv12, int width, int height) const;
  cv::Mat preprocess(const cv::Mat & bgr, std::vector<float> & chw, int & orig_w, int & orig_h,
      float & scale, int & pad_left, int & pad_top) const;

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_{nullptr};
  rclcpp::Publisher<TensorList>::SharedPtr infer_pub_{nullptr};

  // model input config (NHWC float32)
  int in_w_{518};
  int in_h_{518};
  int in_c_{3};

  // RPCMEM (ION) input buffer
  void * rpcmem_lib_{nullptr};
  void * input_ptr_{nullptr};
  int input_fd_{-1};
  size_t input_bytes_{0};
};

}  // namespace qrb_ros_pre_process_with_dma

#endif  // QRB_ROS_PRE_PROCESS_WITH_DMA__QRB_ROS_PRE_PROCESS_WITH_DMA_NODE_HPP_
