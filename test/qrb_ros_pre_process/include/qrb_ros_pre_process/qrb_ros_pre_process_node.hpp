// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_PRE_PROCESS_NODE_HPP_
#define QRB_ROS_PRE_PROCESS_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include "qrb_ros_tensor_list_msgs/msg/tensor_list.hpp"

namespace qrb_ros::pre_process
{
using tensor_list = qrb_ros_tensor_list_msgs::msg::TensorList;

class QrbRosPreProcessNode : public rclcpp::Node
{
public:
  QrbRosPreProcessNode(const rclcpp::NodeOptions & options);
  ~QrbRosPreProcessNode();

private:
  rclcpp::Publisher<tensor_list>::SharedPtr pub_{ nullptr };
  rclcpp::TimerBase::SharedPtr timer_{ nullptr };
  void init(std::string image_path);
};

}  // namespace qrb_ros::pre_process

#endif