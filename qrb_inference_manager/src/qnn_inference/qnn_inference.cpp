// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qnn_inference/qnn_inference.hpp"

namespace qrb::inference_mgr
{

/**
 * \brief file path of input and output are fixed
*/
QnnInferenceFromFile::QnnInferenceFromFile(const std::string &backend_option, const std::string &model_path)
{
  this->qnn_inference_fromfile_impl_ = std::make_unique<QnnInferenceFromFileImpl>(backend_option, model_path);
}

StatusCode QnnInferenceFromFile::inference_init()
{
  if(this->qnn_inference_fromfile_impl_->initialize()!= StatusCode::SUCCESS) {
    return StatusCode::FAILURE;
  }

  if(this->qnn_inference_fromfile_impl_->initialize_backend() != StatusCode::SUCCESS) {
    return StatusCode::FAILURE;
  }

  if(this->qnn_inference_fromfile_impl_->create_device() != StatusCode::SUCCESS) {
    return StatusCode::FAILURE;
  }

  return StatusCode::SUCCESS;
}

StatusCode QnnInferenceFromFile::inference_graph_init()
{
  if(this->qnn_inference_fromfile_impl_->create_context() != StatusCode::SUCCESS) {
    return StatusCode::FAILURE;
  }

  if(this->qnn_inference_fromfile_impl_->compose_graphs() != StatusCode::SUCCESS) {
    return StatusCode::FAILURE;
  }

  if(this->qnn_inference_fromfile_impl_->finalize_graphs() != StatusCode::SUCCESS) {
    return StatusCode::FAILURE;
  }
  return StatusCode::SUCCESS;
}

StatusCode QnnInferenceFromFile::inference_execute([[maybe_unused]] const std::vector<uint8_t> &input_tensor_data)
{
  if(this->qnn_inference_fromfile_impl_->execute_graphs() != StatusCode::SUCCESS) {
    return StatusCode::FAILURE;
  }

  return StatusCode::SUCCESS;
}

/// not used in QnnInferenceFromFile
const std::vector<OutputTensor> QnnInferenceFromFile::get_output_tensors()
{
  std::vector<OutputTensor> not_used;
  return not_used;
}


//*---------------------------------------------QnnInference---------------------------------------------*//
QnnInference::QnnInference(const std::string &backend_option, const std::string &model_path)
{
  this->qnn_inference_impl_ = std::make_unique<QnnInferenceImpl>(backend_option, model_path);
}

StatusCode QnnInference::inference_init()
{
  if(this->qnn_inference_impl_->initialize()!= StatusCode::SUCCESS) {
    return StatusCode::FAILURE;
  }

  if(this->qnn_inference_impl_->initialize_backend() != StatusCode::SUCCESS) {
    return StatusCode::FAILURE;
  }

  if(this->qnn_inference_impl_->create_device() != StatusCode::SUCCESS) {
    return StatusCode::FAILURE;
  }

  return StatusCode::SUCCESS;
}

StatusCode QnnInference::inference_graph_init()
{
  if(this->qnn_inference_impl_->create_context() != StatusCode::SUCCESS) {
    return StatusCode::FAILURE;
  }

  if(this->qnn_inference_impl_->compose_graphs() != StatusCode::SUCCESS) {
    return StatusCode::FAILURE;
  }

  if(this->qnn_inference_impl_->finalize_graphs() != StatusCode::SUCCESS) {
    return StatusCode::FAILURE;
  }

  return StatusCode::SUCCESS;
}

StatusCode QnnInference::inference_execute(const std::vector<uint8_t> &input_tensor_data)
{
  if(input_tensor_data.size() == 0) {
    return StatusCode::FAILURE;
  }

  if(this->qnn_inference_impl_->execute_graphs(input_tensor_data) != StatusCode::SUCCESS) {
    return StatusCode::FAILURE;
  }

  return StatusCode::SUCCESS;
}

const std::vector<OutputTensor> QnnInference::get_output_tensors()
{
  return this->qnn_inference_impl_->get_output_tensors();
}

} // namespace qrb::inference_mgr