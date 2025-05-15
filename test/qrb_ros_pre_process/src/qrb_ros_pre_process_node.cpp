// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_pre_process/qrb_ros_pre_process_node.hpp"

#include <filesystem>
#include <fstream>

namespace qrb_ros::pre_process
{

QrbRosPreProcessNode::QrbRosPreProcessNode(const rclcpp::NodeOptions & options)
  : Node("QrbRosPreProcessNode", options)
{
  std::string image_path = this->declare_parameter("image_path", "");
  if (image_path.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No image for model inference!");
    rclcpp::shutdown();
  }

  this->init(image_path);
}

QrbRosPreProcessNode::~QrbRosPreProcessNode()
{
  RCLCPP_INFO(this->get_logger(), "QrbRosPreProcessNode stopping...");
}

void QrbRosPreProcessNode::init(std::string image_path)
{
  RCLCPP_INFO(this->get_logger(), "QrbRosPreProcessNode init successfully!");

  rclcpp::PublisherOptions pub_option;
  pub_option.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  pub_ = this->create_publisher<tensor_list>("qrb_inference_input_tensor", 10, pub_option);

  int time_interval = 3;
  timer_ = this->create_wall_timer(std::chrono::seconds(time_interval), [this, image_path]() {
    tensor_list msg;
    qrb_ros_tensor_list_msgs::msg::Tensor tensor;
    tensor.data_type = 2;  // float32
    tensor.name = "input_tensor";
    tensor.shape = std::vector<uint32_t>{ 640, 640, 3 };

    std::ifstream file(image_path, std::ios::binary);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open image file: %s", image_path.c_str());
    }

    file.seekg(0, std::ios::end);
    size_t file_size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<uint8_t> img_data(file_size);
    file.read(reinterpret_cast<char *>(img_data.data()), file_size);
    file.close();

    tensor.data = std::move(img_data);
    msg.tensor_list.emplace_back(tensor);

    RCLCPP_INFO(this->get_logger(), "pre-process node is publishing %s", image_path.c_str());
    pub_->publish(std::move(msg));
  });
}

}  // namespace qrb_ros::pre_process

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::pre_process::QrbRosPreProcessNode)