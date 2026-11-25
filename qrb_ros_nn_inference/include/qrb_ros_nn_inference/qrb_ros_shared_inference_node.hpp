// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_SHARED_INFERENCE_NODE_HPP_
#define QRB_ROS_SHARED_INFERENCE_NODE_HPP_

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "qrb_inference_manager.hpp"
#include "qrb_ros_tensor_list_msgs/msg/tensor_list.hpp"

namespace qrb_ros::nn_inference
{

namespace custom_msg = qrb_ros_tensor_list_msgs::msg;

// Loads a single multi-graph QNN context binary and exposes one pub/sub pair per graph.
// Topics are named qrb_inference_input_tensor_N / qrb_inference_output_tensor_N (N = 0, 1, ...).
// Parameters:
//   backend_option  (string)  - QNN backend library path, e.g. /usr/lib/libQnnHtp.so
//   model_path      (string)  - path to the combined multi-graph .bin file
//   graph_name_0    (string)  - name of the first graph in the context binary
//   graph_name_1    (string)  - name of the second graph in the context binary
class QrbRosSharedInferenceNode : public rclcpp::Node
{
public:
  explicit QrbRosSharedInferenceNode(const rclcpp::NodeOptions & options);
  ~QrbRosSharedInferenceNode() = default;

private:
  std::unique_ptr<qrb::inference_mgr::QrbSharedContextLoader> loader_{ nullptr };
  std::vector<std::unique_ptr<qrb::inference_mgr::QrbInferenceManager>> graph_mgrs_;
  std::vector<rclcpp::Subscription<custom_msg::TensorList>::SharedPtr> subs_;
  std::vector<rclcpp::Publisher<custom_msg::TensorList>::SharedPtr> pubs_;

  void graph_callback(custom_msg::TensorList::ConstSharedPtr msg, std::size_t idx);
  void publish_result(std::size_t idx, const custom_msg::TensorList & input_msg);
};

}  // namespace qrb_ros::nn_inference

#endif
