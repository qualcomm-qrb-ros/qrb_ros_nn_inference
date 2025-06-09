// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qnn_delegate_inference/qnn_delegate_inference.hpp"

#include <tensorflow/lite/kernels/register.h>

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

    if (TfLiteStatus::kTfLiteOk == this->interpreter_->ModifyGraphWithDelegate(this->delegate_)) {
      QRB_INFO("Enable QNN delegate Successfully!");
    } else {
      QRB_ERROR("Qnn Delegate register fail!");
      return StatusCode::FAILURE;
    }
  }

  return StatusCode::SUCCESS;
}

StatusCode QnnDelegateInference::inference_init()
{
  this->model_ = tflite::FlatBufferModel::BuildFromFile((this->model_path_).c_str());
  if (this->model_ == nullptr) {
    QRB_ERROR("TFLite model load fail!");
    return StatusCode::FAILURE;
  }

  tflite::ops::builtin::BuiltinOpResolver resolver;
  tflite::InterpreterBuilder(*(this->model_), resolver)(&(this->interpreter_));
  if (this->interpreter_ == nullptr) {
    QRB_ERROR("TFLite interpreter build fail!");
    return StatusCode::FAILURE;
  }

  if (TfLiteStatus::kTfLiteOk != this->interpreter_->AllocateTensors()) {
    QRB_ERROR("TFLite input tensor create fail!");
    return StatusCode::FAILURE;
  }

  if (this->register_qnn_delegate() == StatusCode::FAILURE) {
    return StatusCode::FAILURE;
  }

  QRB_INFO("Inference init Successfully!");

  return StatusCode::SUCCESS;
}

StatusCode QnnDelegateInference::inference_graph_init()
{
  return StatusCode::SUCCESS;
}

StatusCode QnnDelegateInference::inference_execute(const std::vector<uint8_t> & input_tensor_data)
{
  auto tflite_dtype_to_qrb_dtype = [](const TfLiteType & tflite_dtype) -> int32_t {
    switch (tflite_dtype) {
      case kTfLiteUInt8:
        return 0;
      case kTfLiteInt8:
        return 1;
      case kTfLiteFloat32:
        return 2;
      case kTfLiteFloat64:
        return 3;
      default:
        return -1;
    }
  };

  auto input_tensor = this->interpreter_->tensor(this->interpreter_->inputs()[0]);

  if (-1 == tflite_dtype_to_qrb_dtype(input_tensor->type)) {
    QRB_ERROR("The input data type of model is NOT supported");
    return StatusCode::FAILURE;
  }

  if (input_tensor_data.size() != input_tensor->bytes) {
    QRB_ERROR("The size of input tensor should be ", input_tensor->bytes, ", but receive ",
        input_tensor_data.size(), "byte");
    return StatusCode::FAILURE;
  }

  // input_tensor->data.raw = input_tensor_data.data();
  memcpy(input_tensor->data.f, input_tensor_data.data(), input_tensor_data.size());

  if (TfLiteStatus::kTfLiteOk != this->interpreter_->Invoke()) {
    QRB_ERROR("TFLite invoke fail!");
    return StatusCode::FAILURE;
  }

  for (size_t i = 0; i < this->interpreter_->outputs().size(); i++) {
    auto result_tensor = this->interpreter_->output_tensor(i);
    OutputTensor output_tensor;
    output_tensor.data_type = tflite_dtype_to_qrb_dtype(result_tensor->type);
    if (output_tensor.data_type == -1) {
      QRB_ERROR("The output data type of model is NOT supported");
      return StatusCode::FAILURE;
    }

    output_tensor.output_tensor_name = result_tensor->name;
    output_tensor.output_tensor_data = std::vector<uint8_t>(result_tensor->bytes);
    memcpy(output_tensor.output_tensor_data.data(), result_tensor->data.raw,
        result_tensor->bytes);  // reinterpret_cast<uint8_t*>

    for (auto j = 0; j < result_tensor->dims->size; j++) {
      output_tensor.output_tensor_shape.emplace_back(result_tensor->dims->data[j]);
    }

    this->output_tensor_.emplace_back(output_tensor);
  }

  QRB_INFO("Inference execute Successfully!");

  return StatusCode::SUCCESS;
}

const std::vector<OutputTensor> QnnDelegateInference::get_output_tensors()
{
  return std::move(this->output_tensor_);
}

}  // namespace qrb::inference_mgr
