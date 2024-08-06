// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_inference_manager.hpp"
#include "qnn_inference/qnn_inference.hpp"
#include "qnn_delegate_inference/qnn_delegate_inference.hpp"

namespace qrb::inference_mgr
{

/**
 * \brief initialize qrb_inference_ to QnnInferenceFromFile or QnnInference or QnnDelegateInference
 * \param backend_option backend lib of QNN
 * \param model_path path of model
 * \param inference_from_file whether get input data of model from file path
 * \throw std::logic_error, if param not meet requirement
*/
QrbInferenceManager::QrbInferenceManager(
  const std::string &backend_option,
  const std::string &model_path,
  const bool &inference_from_file)
{
  auto param_error = std::logic_error("ERROR: Node parameters are invalid!");
  auto check_empty = [param_error](std::string s) {
    if(s.empty()) {
      throw param_error;
    }
  };

  check_empty(model_path);

  auto is_qnn_model = (std::string::npos != model_path.find(".so"));
  auto is_tflite_model = (std::string::npos != model_path.find(".tflite"));

  if(is_qnn_model) { // qnn model, need to provide vaild backend_option
    check_empty(backend_option);

    if(true == inference_from_file) {
      qrb_inference_ = std::make_unique<QnnInferenceFromFile>(backend_option, model_path);
    }
    else {
      qrb_inference_ = std::make_unique<QnnInference>(backend_option, model_path);
    }
  }
  else if(is_tflite_model) { // tflite model
    qrb_inference_ = std::make_unique<QnnDelegateInference>(model_path);
  }
  else {
    throw param_error;
  }

  if(qrb_inference_->inference_init() != StatusCode::SUCCESS) {
    throw std::logic_error("ERROR: Inference init fail!");
  }

  if(qrb_inference_->inference_graph_init() != StatusCode::SUCCESS) {
    throw std::logic_error("ERROR: Inference graph init fail!");
  }
}

bool QrbInferenceManager::inference_execute(const std::vector<uint8_t> &input_tensor_data)
{
  if(qrb_inference_->inference_execute(input_tensor_data) == StatusCode::SUCCESS) {
    return true;
  }
  return false;
}

std::vector<OutputTensor> QrbInferenceManager::get_output_tensors()
{
  return this->qrb_inference_->get_output_tensors();
}

} // namespace qrb::inference_mgr