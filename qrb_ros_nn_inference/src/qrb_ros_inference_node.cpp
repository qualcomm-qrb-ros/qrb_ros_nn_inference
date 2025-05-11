// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_nn_inference/qrb_ros_inference_node.hpp"

namespace qrb_ros::nn_inference
{

/**
 * \brief inferenece init, create subscriper, create publisher
 */
QrbRosInferenceNode::QrbRosInferenceNode(const rclcpp::NodeOptions & options)
  : Node("qrb_ros_inference_node", options)
{
  std::string backend_option = this->declare_parameter("backend_option", "");
  std::string model_path = this->declare_parameter("model_path", "");

  if (false == this->init(model_path, backend_option)) {
    rclcpp::shutdown();
  }

  rclcpp::PublisherOptions pub_option;
  pub_option.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  this->pub_ =
      this->create_publisher<custom_msg::TensorList>("qrb_inference_output_tensor", 10, pub_option);

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  this->sub_ = this->create_subscription<custom_msg::TensorList>("qrb_inference_input_tensor", 10,
      std::bind(&QrbRosInferenceNode::subscription_callback, this, std::placeholders::_1), sub_opt);

  RCLCPP_INFO(this->get_logger(), "Inference init successfully!");
}

/**
 * \brief callback func of subscriper, do model inference for every topic msg
 * \param msg msg from subscribed topic
 */
void QrbRosInferenceNode::subscription_callback(const custom_msg::TensorList & msg)
{
  RCLCPP_INFO(this->get_logger(), "Got model input data, start executing inference...");

  const auto input_tensor = (msg.tensor_list)[0];

  if (static_cast<int>(TensorDataType::FLOAT32) == input_tensor.data_type) {  // only support
                                                                              // float32
    if (false == this->qrb_inference_mgr_->inference_execute(input_tensor.data)) {
      RCLCPP_ERROR(this->get_logger(), "Inference execute fail!");
      rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(), "Inference execute successfully!");

    custom_msg::TensorList pub_tensors;
    pub_tensors.header = msg.header;
    this->publish_msg(pub_tensors);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Input tensor data type not support!");
  }
}

/**
 * \brief callback func of publisher, change inference result to ros msg then output
 */
void QrbRosInferenceNode::publish_msg(custom_msg::TensorList pub_tensors)
{
  const auto result_tensors = this->qrb_inference_mgr_->get_output_tensors();

  for (auto rt : result_tensors) {
    custom_msg::Tensor tensor;
    tensor.data_type = rt.data_type;
    tensor.name = rt.output_tensor_name;
    tensor.shape = rt.output_tensor_shape;
    tensor.data = rt.output_tensor_data;
    pub_tensors.tensor_list.emplace_back(tensor);
  }

  RCLCPP_INFO(this->get_logger(), "Publish the inference result...");
  this->pub_->publish(std::move(pub_tensors));
}

/**
 * \brief initilize the qrb_inference_mgr_
 * \param model_path path of model
 * \param backend_option backend lib of QNN
 * \return true if success or false for failed
 */
bool QrbRosInferenceNode::init(const std::string & model_path, const std::string & backend_option)
try {
  qrb_inference_mgr_ =
      std::make_unique<qrb::inference_mgr::QrbInferenceManager>(model_path, backend_option);

  return true;
} catch (const std::logic_error & e) {
  RCLCPP_ERROR(this->get_logger(), e.what());
  return false;
}

}  // namespace qrb_ros::nn_inference

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::nn_inference::QrbRosInferenceNode)