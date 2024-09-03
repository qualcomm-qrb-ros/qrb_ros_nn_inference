// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <fstream>
#include <filesystem>

#include "qrb_ros_pre_process/qrb_ros_pre_process_node.hpp"

namespace qrb_ros::pre_process
{

QrbRosPreProcessNode::QrbRosPreProcessNode(const rclcpp::NodeOptions& options) : Node("QrbRosPreProcessNode", options)
{
  this->init();
}

QrbRosPreProcessNode::~QrbRosPreProcessNode()
{
  RCLCPP_INFO(this->get_logger(), "QrbRosPreProcessNode stopping...");
}

void QrbRosPreProcessNode::init()
{
  RCLCPP_INFO(this->get_logger(), "QrbRosPreProcessNode init successfully!");

  rclcpp::PublisherOptions pub_option;
  pub_option.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  pub_ = this->create_publisher<tensor_list>("qrb_inference_input_tensor", 10, pub_option);

  int time_interval = 3;
  timer_ = this->create_wall_timer(std::chrono::seconds(time_interval), [this]() {
    tensor_list msg;
    qrb_ros_tensor_list_msgs::msg::Tensor tensor;
    tensor.data_type = 2; // float32
    tensor.name = "input tensor name";
    tensor.shape = std::vector<uint32_t>{640,640,3};

    std::string workspace = std::string{std::getenv("QRB_ROS_WS")};
    std::string image_path {workspace + "/src/qrb_ros_nn_inference/test/qrb_ros_pre_process/image/image.raw"};
    RCLCPP_INFO(this->get_logger(), "pre-process node is publishing %s", image_path.c_str());

    std::ifstream file(image_path, std::ios::binary);
    tensor.data = std::vector<uint8_t>((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    msg.tensor_list.emplace_back(tensor);

    pub_->publish(std::move(msg));
  });
}

}  // namespace qrb_ros::pre_process

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::pre_process::QrbRosPreProcessNode)