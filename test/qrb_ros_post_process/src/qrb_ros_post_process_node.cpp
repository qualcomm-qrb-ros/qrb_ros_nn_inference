// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_post_process/qrb_ros_post_process_node.hpp"

#include <filesystem>
#include <fstream>

namespace qrb_ros::post_process
{

QrbRosPostProcessNode::QrbRosPostProcessNode(const rclcpp::NodeOptions & options)
  : Node("QrbRosPostProcessNode", options)
{
  RCLCPP_INFO(this->get_logger(), "QrbRosPostProcessNode starting...");

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  this->sub_ = this->create_subscription<custom_msg::TensorList>("qrb_inference_output_tensor", 10,
      std::bind(&QrbRosPostProcessNode::subscription_callback, this, std::placeholders::_1),
      sub_opt);
}

QrbRosPostProcessNode::~QrbRosPostProcessNode()
{
  RCLCPP_INFO(this->get_logger(), "QrbRosPostProcessNode stopping...");
}

void QrbRosPostProcessNode::subscription_callback(const custom_msg::TensorList & msg)
{
  std::string workspace = std::string{ std::getenv("QRB_ROS_WS") };
  std::string result_path{ workspace +
                           "/src/qrb_ros_nn_inference/test/qrb_ros_post_process/"
                           "inference_result/" };

  if (false == std::filesystem::exists(result_path)) {
    if (std::filesystem::create_directory(result_path)) {
      RCLCPP_INFO(
          this->get_logger(), "The inference results will be stored in %s", result_path.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Failed to create the folder: %s", result_path.c_str());
      rclcpp::shutdown();
    }
  }

  for (auto result_tensor : msg.tensor_list) {
    auto result_tensor_data = result_tensor.data;
    std::string result_tensor_path = result_path + result_tensor.name + ".raw";
    RCLCPP_INFO(this->get_logger(), "Write result tensor under: %s", result_tensor_path.c_str());
    std::ofstream output_file(result_tensor_path, std::ios::binary);
    output_file.write(
        reinterpret_cast<const char *>(result_tensor_data.data()), result_tensor_data.size());
  }
}

}  // namespace qrb_ros::post_process

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::post_process::QrbRosPostProcessNode)