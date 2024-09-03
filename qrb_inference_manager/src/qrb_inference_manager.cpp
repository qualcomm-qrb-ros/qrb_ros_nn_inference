// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_inference_manager.hpp"

#include <stdexcept>

#include "qnn_delegate_inference/qnn_delegate_inference.hpp"
#include "qnn_inference/qnn_inference.hpp"

namespace qrb::inference_mgr
{

/**
 * \brief initialize qrb_inference_ to QnnInference or QnnDelegateInference
 * \param backend_option backend lib of QNN
 * \param model_path path of model
 * \param qnn_syslib_path path of libQnnSystem.so
 * \throw std::logic_error, if param not meet requirement
 */
QrbInferenceManager::QrbInferenceManager(const std::string & model_path,
    const std::string & backend_option,
    const std::string & qnn_syslib_path)
{
  auto param_error = std::logic_error("ERROR: Node parameters are invalid!");
  auto check_empty = [param_error](std::string s) {
    if (s.empty()) {
      throw param_error;
    }
  };

  auto is_so_model = (std::string::npos != model_path.find(".so"));
  auto is_bin_model = (std::string::npos != model_path.find(".bin"));
  auto is_tflite_model = (std::string::npos != model_path.find(".tflite"));

  if (!is_so_model && !is_bin_model && !is_tflite_model) {
    throw param_error;
  }

  check_empty(model_path);

  if (is_tflite_model) {
    qrb_inference_ = std::make_unique<QnnDelegateInference>(model_path, backend_option);
  } else {
    check_empty(backend_option);
    if (is_so_model) {
      qrb_inference_ = std::make_unique<QnnInference>(model_path, backend_option);
    } else {
      check_empty(qnn_syslib_path);
      qrb_inference_ = std::make_unique<QnnInference>(model_path, backend_option, qnn_syslib_path);
    }
  }

  if (qrb_inference_->inference_init() != StatusCode::SUCCESS) {
    throw std::logic_error("ERROR: Inference init fail!");
  }

  if (qrb_inference_->inference_graph_init() != StatusCode::SUCCESS) {
    throw std::logic_error("ERROR: Inference graph init fail!");
  }
}

bool QrbInferenceManager::inference_execute(const std::vector<uint8_t> & input_tensor_data)
{
  if (qrb_inference_->inference_execute(input_tensor_data) == StatusCode::SUCCESS) {
    return true;
  }
  return false;
}

std::vector<OutputTensor> QrbInferenceManager::get_output_tensors()
{
  return this->qrb_inference_->get_output_tensors();
}

}  // namespace qrb::inference_mgr
