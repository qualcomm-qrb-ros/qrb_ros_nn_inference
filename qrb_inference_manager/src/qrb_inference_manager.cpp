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
 * \throw std::logic_error, if param not meet requirement
 */
QrbInferenceManager::QrbInferenceManager(const std::string & model_path,
    const std::string & backend_option)
{
  auto is_so_model = (std::string::npos != model_path.find(".so"));
  auto is_bin_model = (std::string::npos != model_path.find(".bin"));
  auto is_tflite_model = (std::string::npos != model_path.find(".tflite"));

  if (!is_so_model && !is_bin_model && !is_tflite_model) {
    throw std::logic_error("ERROR: Model format NOT support!");
  }

  if (is_tflite_model) {
    qrb_inference_ = std::make_unique<QnnDelegateInference>(model_path, backend_option);
  } else {
    qrb_inference_ = std::make_unique<QnnInference>(model_path, backend_option);
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

bool QrbInferenceManager::inference_execute_dmabuf(int dmabuf_fd,
    uint32_t dmabuf_size,
    uint64_t dmabuf_offset)
{
  auto * qnn_inference = dynamic_cast<QnnInference *>(qrb_inference_.get());
  if (nullptr == qnn_inference) {
    QRB_ERROR("DMA-BUF inference is only supported by QnnInference (HTP backend).");
    return false;
  }

  if (qnn_inference->inference_execute_dmabuf(dmabuf_fd, dmabuf_size, dmabuf_offset) ==
      StatusCode::SUCCESS) {
    return true;
  }
  return false;
}

std::vector<OutputTensor> QrbInferenceManager::get_output_tensors()
{
  return this->qrb_inference_->get_output_tensors();
}

}  // namespace qrb::inference_mgr
