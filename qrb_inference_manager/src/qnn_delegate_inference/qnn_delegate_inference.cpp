// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear
#include "qnn_delegate_inference/qnn_delegate_inference.hpp"

#include <cstring>

#include "TFLiteDelegate/QnnTFLiteDelegate.h"

namespace qrb::inference_mgr
{

QnnDelegateInference::QnnDelegateInference(const std::string & model_path,
    const std::string & backend_option)
  : model_path_(model_path)
  , backend_option_(backend_option){

  };

QnnDelegateInference::~QnnDelegateInference()
{
  if (this->interpreter_ != nullptr) {
    TfLiteInterpreterDelete(interpreter_);
  }

  if (this->options_ != nullptr) {
    TfLiteInterpreterOptionsDelete(options_);
  }

  if (this->model_ != nullptr) {
    TfLiteModelDelete(model_);
  }

  if (this->delegate_ != nullptr) {
    TfLiteQnnDelegateDelete(this->delegate_);
  }
}

StatusCode QnnDelegateInference::register_qnn_delegate()
{
  if (false == backend_option_.empty()) {
    TfLiteQnnDelegateOptions options = TfLiteQnnDelegateOptionsDefault();
    if (backend_option_ == "gpu") {
      options.backend_type = kGpuBackend;
      QRB_INFO("GPU delegate create successfully!");
    } else if (backend_option_ == "htp") {
      options.backend_type = kHtpBackend;
      QRB_INFO("HTP delegate create successfully!");
    } else if (backend_option_ == "dsp") {
      options.backend_type = kDspBackend;
      QRB_INFO("DSP delegate create successfully!");
    } else {
      QRB_ERROR("QNN Delegate type NOT support!");
      return StatusCode::FAILURE;
    }

    this->delegate_ = TfLiteQnnDelegateCreate(&options);
    if (this->delegate_ == nullptr) {
      QRB_ERROR("Qnn Delegate create fail!");
      return StatusCode::FAILURE;
    }
    TfLiteInterpreterOptionsAddDelegate(options_, delegate_);
  }

  return StatusCode::SUCCESS;
}

StatusCode QnnDelegateInference::inference_init()
{
  model_ = TfLiteModelCreateFromFile(model_path_.c_str());
  if (model_ == nullptr) {
    QRB_ERROR("TFLite model create fail!");
    return StatusCode::FAILURE;
  }

  options_ = TfLiteInterpreterOptionsCreate();
  if (this->register_qnn_delegate() == StatusCode::FAILURE) {
    return StatusCode::FAILURE;
  }

  TfLiteInterpreterOptionsSetNumThreads(options_, 4);
  interpreter_ = TfLiteInterpreterCreate(model_, options_);
  if (interpreter_ == nullptr) {
    QRB_ERROR("TFLite interpreter create fail!");
    return StatusCode::FAILURE;
  }

  TfLiteInterpreterAllocateTensors(interpreter_);

  if (this->register_qnn_delegate() == StatusCode::FAILURE) {
    return StatusCode::FAILURE;
  }

  return StatusCode::SUCCESS;
}

StatusCode QnnDelegateInference::inference_graph_init()
{
  return StatusCode::SUCCESS;
}

StatusCode QnnDelegateInference::inference_execute(const std::vector<uint8_t> & input_tensor_data)
{
  TfLiteTensor * input_tensor = TfLiteInterpreterGetInputTensor(interpreter_, 0);

  if (input_tensor->type != kTfLiteFloat32) {
    QRB_ERROR("The data type of input tensor need to be FLOAT32!");
    return StatusCode::FAILURE;
  }

  if (input_tensor_data.size() != input_tensor->bytes) {
    QRB_ERROR("The size of input tensor should be ", input_tensor->bytes, ", but receive ",
        input_tensor_data.size(), "byte");
    return StatusCode::FAILURE;
  }

  TfLiteTensorCopyFromBuffer(
      input_tensor, input_tensor_data.data(), input_tensor_data.size() * sizeof(float));

  if (TfLiteStatus::kTfLiteOk != TfLiteInterpreterInvoke(interpreter_)) {
    QRB_ERROR("TFLite invoke fail!");
    return StatusCode::FAILURE;
  }

  for (int32_t i = 0; i < TfLiteInterpreterGetOutputTensorCount(interpreter_); i++) {
    const TfLiteTensor * result_tensor = TfLiteInterpreterGetOutputTensor(interpreter_, i);

    OutputTensor output_tensor;

    output_tensor.output_tensor_name = result_tensor->name;

    output_tensor.output_tensor_data = std::vector<uint8_t>(result_tensor->bytes);
    memcpy(output_tensor.output_tensor_data.data(), result_tensor->data.f, result_tensor->bytes);

    for (auto j = 0; j < result_tensor->dims->size; j++) {
      output_tensor.output_tensor_shape.emplace_back(result_tensor->dims->data[j]);
    }

    this->output_tensor_.emplace_back(output_tensor);
  }

  return StatusCode::SUCCESS;
}

const std::vector<OutputTensor> QnnDelegateInference::get_output_tensors()
{
  return std::move(this->output_tensor_);
}

}  // namespace qrb::inference_mgr