// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_POST_PROCESS_NODE_HPP_
#define QRB_ROS_POST_PROCESS_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include "qrb_ros_tensor_list_msgs/msg/tensor_list.hpp"

namespace qrb_ros::post_process
{

namespace custom_msg = qrb_ros_tensor_list_msgs::msg;

class QrbRosPostProcessNode : public rclcpp::Node
{
public:
  QrbRosPostProcessNode(const rclcpp::NodeOptions & options);
  ~QrbRosPostProcessNode();

private:
  rclcpp::Subscription<custom_msg::TensorList>::SharedPtr sub_{ nullptr };

  void subscription_callback(const custom_msg::TensorList & msg);
};

}  // namespace qrb_ros::post_process

#endif