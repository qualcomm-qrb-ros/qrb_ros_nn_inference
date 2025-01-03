// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_pre_process/qrb_ros_pre_process_node.hpp"

#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>

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

    cv::Mat img = cv::imread(image_path);
    cv::resize(img, img, cv::Size(640, 640), 0, 0, cv::INTER_AREA);
    img.convertTo(img, CV_32F, 1.0 / 255.0);

    std::vector<uint8_t> img_data;
    if (img.isContinuous()) {
      img_data.assign((uint8_t *)img.datastart, (uint8_t *)img.dataend);
    } else {
      for (int i = 0; i < img.rows; ++i) {
        img_data.insert(img_data.end(), img.ptr<uint8_t>(i),
            img.ptr<uint8_t>(i) + img.cols * img.channels() * sizeof(float));
      }
    }

    tensor.data = std::move(img_data);
    msg.tensor_list.emplace_back(tensor);

    RCLCPP_INFO(this->get_logger(), "pre-process node is publishing %s", image_path.c_str());
    pub_->publish(std::move(msg));
  });
}

}  // namespace qrb_ros::pre_process

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::pre_process::QrbRosPreProcessNode)