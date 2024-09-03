// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_INFERENCE_NODE_HPP_
#define QRB_ROS_INFERENCE_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "qrb_inference_manager.hpp"
#include "qrb_ros_tensor_list_msgs/msg/tensor_list.hpp"

namespace qrb_ros::nn_inference
{

namespace custom_msg = qrb_ros_tensor_list_msgs::msg;

enum class TensorDataType
{
  UINT8,
  INT8,
  FLOAT32,
  FLOAT64,
};

class QrbRosInferenceNode : public rclcpp::Node
{
public:
  QrbRosInferenceNode(const rclcpp::NodeOptions & options);
  ~QrbRosInferenceNode() = default;

private:
  std::unique_ptr<qrb::inference_mgr::QrbInferenceManager> qrb_inference_mgr_{ nullptr };
  rclcpp::Subscription<custom_msg::TensorList>::SharedPtr sub_{ nullptr };
  rclcpp::Publisher<custom_msg::TensorList>::SharedPtr pub_{ nullptr };

  bool init(const std::string & model_path,
      const std::string & backend_option,
      const std::string & qnn_syslib_path);
  void subscription_callback(const custom_msg::TensorList & msg);
  void publish_msg(custom_msg::TensorList pub_tensors);
};

}  // namespace qrb_ros::nn_inference

#endif