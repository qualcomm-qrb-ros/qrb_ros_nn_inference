// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_nn_inference/qrb_ros_shared_inference_node.hpp"

#include <chrono>
#include <functional>
#include <string>

namespace qrb_ros::nn_inference
{

QrbRosSharedInferenceNode::QrbRosSharedInferenceNode(const rclcpp::NodeOptions & options)
: Node("qrb_ros_shared_inference_node", options)
{
  const std::string backend_option = this->declare_parameter("backend_option", "");
  const std::string model_path = this->declare_parameter("model_path", "");
  const std::string graph_name_0 = this->declare_parameter("graph_name_0", "");
  const std::string graph_name_1 = this->declare_parameter("graph_name_1", "");

  if (model_path.empty()) {
    RCLCPP_ERROR(this->get_logger(), "model_path parameter is required");
    rclcpp::shutdown();
    return;
  }

  const std::vector<std::string> graph_names = { graph_name_0, graph_name_1 };

  for (const auto & name : graph_names) {
    if (name.empty()) {
      RCLCPP_ERROR(this->get_logger(),
          "graph_name_0 and graph_name_1 parameters are required");
      rclcpp::shutdown();
      return;
    }
  }

  // Load the combined context binary once.
  try {
    loader_ = std::make_unique<qrb::inference_mgr::QrbSharedContextLoader>(
        model_path, backend_option);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load shared context: %s", e.what());
    rclcpp::shutdown();
    return;
  }

  // Create one QrbInferenceManager per graph, one pub/sub pair per graph.
  for (std::size_t i = 0; i < graph_names.size(); ++i) {
    try {
      graph_mgrs_.push_back(loader_->make_graph_manager(graph_names[i]));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(),
          "Failed to create graph manager for '%s': %s", graph_names[i].c_str(), e.what());
      rclcpp::shutdown();
      return;
    }

    // Each subscription gets its own Reentrant callback group so both can fire concurrently
    // under component_container_mt.
    auto cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cb_group;
    sub_opts.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

    const std::string in_topic = "qrb_inference_input_tensor_" + std::to_string(i);
    const std::string out_topic = "qrb_inference_output_tensor_" + std::to_string(i);

    rclcpp::PublisherOptions pub_opts;
    pub_opts.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

    pubs_.push_back(
        this->create_publisher<custom_msg::TensorList>(out_topic, 10, pub_opts));

    subs_.push_back(
        this->create_subscription<custom_msg::TensorList>(
            in_topic, 10,
            [this, i](custom_msg::TensorList::ConstSharedPtr msg) {
              this->graph_callback(msg, i);
            },
            sub_opts));

    RCLCPP_INFO(this->get_logger(),
        "Graph %zu ('%s'): sub=%s pub=%s",
        i, graph_names[i].c_str(), in_topic.c_str(), out_topic.c_str());
  }

  RCLCPP_INFO(this->get_logger(),
      "QrbRosSharedInferenceNode ready: %zu graphs loaded from %s",
      graph_names.size(), model_path.c_str());
}

void QrbRosSharedInferenceNode::graph_callback(
    custom_msg::TensorList::ConstSharedPtr msg, std::size_t idx)
{
  auto t0 = std::chrono::steady_clock::now();

  if (msg->tensor_list.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Graph %zu: empty tensor list", idx);
    return;
  }

  const auto & input_tensor = msg->tensor_list[0];

  bool ok = false;
#ifdef QRB_TENSOR_HAS_DMABUF
  if (input_tensor.dmabuf_fd >= 0 && input_tensor.dmabuf_size > 0) {
    ok = graph_mgrs_[idx]->inference_execute_dmabuf(
        input_tensor.dmabuf_fd, input_tensor.dmabuf_size, input_tensor.dmabuf_offset);
  } else {
    ok = graph_mgrs_[idx]->inference_execute(input_tensor.data);
  }
#else
  ok = graph_mgrs_[idx]->inference_execute(input_tensor.data);
#endif

  auto t1 = std::chrono::steady_clock::now();
  double exec_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  if (!ok) {
    RCLCPP_ERROR(this->get_logger(), "Graph %zu: inference_execute failed (%.2fms)", idx, exec_ms);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Graph %zu: execute=%.2fms data_size=%zu",
      idx, exec_ms, input_tensor.data.size());

  publish_result(idx, *msg);
}

void QrbRosSharedInferenceNode::publish_result(
    std::size_t idx, const custom_msg::TensorList & input_msg)
{
  const auto result_tensors = graph_mgrs_[idx]->get_output_tensors();

  custom_msg::TensorList pub_msg;
  pub_msg.header = input_msg.header;

  for (auto rt : result_tensors) {
    custom_msg::Tensor tensor;
    tensor.data_type = rt.data_type;
    tensor.name = rt.output_tensor_name;
    tensor.shape = rt.output_tensor_shape;

#ifdef QRB_TENSOR_HAS_DMABUF
    if (rt.output_dmabuf_fd >= 0 && rt.output_dmabuf_size > 0) {
      tensor.dmabuf_fd = rt.output_dmabuf_fd;
      tensor.dmabuf_size = rt.output_dmabuf_size;
      tensor.dmabuf_offset = rt.output_dmabuf_offset;
      tensor.dmabuf_ptr = rt.output_dmabuf_ptr;
    } else {
      tensor.data = std::move(rt.output_tensor_data);
    }
#else
    tensor.data = std::move(rt.output_tensor_data);
#endif

    pub_msg.tensor_list.emplace_back(std::move(tensor));
  }

  RCLCPP_DEBUG(this->get_logger(), "Graph %zu: publishing %zu output tensors",
      idx, pub_msg.tensor_list.size());
  pubs_[idx]->publish(std::move(pub_msg));
}

}  // namespace qrb_ros::nn_inference

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::nn_inference::QrbRosSharedInferenceNode)
