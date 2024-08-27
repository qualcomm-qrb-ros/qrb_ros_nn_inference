// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_INFERENCE_MANAGER_QRB_INFERENCE_HPP_
#define QRB_INFERENCE_MANAGER_QRB_INFERENCE_HPP_

#include <memory>
#include <string>
#include <iostream>
#include <vector>

namespace qrb::inference_mgr
{

template<typename... Args>
void QRB_INFO(Args&&... args) {
  std::cout << "\033[32m[QRB INFO] \033[0m";
  (std::cout << ... << args) << std::endl;
}

template<typename... Args>
void QRB_WARNING(Args&&... args) {
  std::cout << "\033[33m[QRB WRANING] \033[0m";
  (std::cout << ... << args) << std::endl;
}

template<typename... Args>
void QRB_ERROR(Args&&... args) {
  std::cout << "\033[31m[QRB ERROR] \033[0m";
  (std::cout << ... << args) << std::endl;
}

enum class StatusCode {
  SUCCESS,
  FAILURE,
  FAILURE_INPUT_LIST_EXHAUSTED,
  FAILURE_SYSTEM_ERROR,
  FAILURE_SYSTEM_COMMUNICATION_ERROR,
  QNN_FEATURE_UNSUPPORTED
};

struct OutputTensor {
  std::vector<uint8_t> output_tensor_data;
  std::string output_tensor_name;
  std::vector<uint32_t> output_tensor_shape;
};

class QrbInference
{
public:
  QrbInference() = default;
  virtual ~QrbInference() = default;
  virtual StatusCode inference_init() = 0;
  virtual StatusCode inference_graph_init() = 0;
  virtual StatusCode inference_execute(const std::vector<uint8_t> &input_tensor_data) = 0;
  virtual const std::vector<OutputTensor> get_output_tensors() = 0;
}; // class QrbInference

} // namespace qrb::inference_mgr

#endif
