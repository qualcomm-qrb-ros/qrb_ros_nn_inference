#include <tensorflow/lite/kernels/register.h>

#include "qnn_delegate_inference/qnn_delegate_inference.hpp"
#include "TFLiteDelegate/QnnTFLiteDelegate.h"

namespace qrb::inference_mgr
{

QnnDelegateInference::QnnDelegateInference(const std::string &model_path):model_path_(model_path)
{

};

QnnDelegateInference::~QnnDelegateInference()
{
  if(this->delegate_ != nullptr) {
    TfLiteQnnDelegateDelete(this->delegate_);
  }
}

StatusCode QnnDelegateInference::register_qnn_delegate()
{
  TfLiteQnnDelegateOptions options = TfLiteQnnDelegateOptionsDefault();
  options.backend_type = kGpuBackend;

  this->delegate_ = TfLiteQnnDelegateCreate(&options);
  if (this->delegate_ == nullptr) {
    QRB_DEBUG("ERROR: Qnn Delegate create fail!");
    return StatusCode::FAILURE;
  }

  if(TfLiteStatus::kTfLiteOk == this->interpreter_->ModifyGraphWithDelegate(this->delegate_)) {
    QRB_DEBUG("INFO: Enable QNN delegate Successfully!");
  }
  else {
    QRB_DEBUG("ERROR: Qnn Delegate register fail!");
    return StatusCode::FAILURE;
  }

  return StatusCode::SUCCESS;
}

StatusCode QnnDelegateInference::inference_init()
{
  this->model_ = tflite::FlatBufferModel::BuildFromFile((this->model_path_).c_str());
  if (this->model_ == nullptr) {
    QRB_DEBUG("ERROR: TFLite model load fail!");
    return StatusCode::FAILURE;
  }

  tflite::ops::builtin::BuiltinOpResolver resolver;
  tflite::InterpreterBuilder(*(this->model_), resolver)(&(this->interpreter_));
  if (this->interpreter_ == nullptr) {
    QRB_DEBUG("ERROR: TFLite interpreter build fail!");
    return StatusCode::FAILURE;
  }

  if(TfLiteStatus::kTfLiteOk != this->interpreter_->AllocateTensors()) {
    QRB_DEBUG("ERROR: TFLite input tensor create fail!");
    return StatusCode::FAILURE;
  }

  if(this->register_qnn_delegate() == StatusCode::FAILURE) {
    return StatusCode::FAILURE;
  }

  QRB_DEBUG("INFO: Inference init Successfully!");

  return StatusCode::SUCCESS;
}

StatusCode QnnDelegateInference::inference_graph_init()
{
  QRB_DEBUG("INFO: Inference graph init Successfully!");
  return StatusCode::SUCCESS;
}

StatusCode QnnDelegateInference::inference_execute(const std::vector<uint8_t> &input_tensor_data)
{
  auto input_tensor = this->interpreter_->tensor(this->interpreter_->inputs()[0]);

  if(input_tensor->type != kTfLiteFloat32) {
    QRB_DEBUG("ERROR: The data type of input tensor need to be FLOAT32!");
    return StatusCode::FAILURE;
  }

  if(input_tensor_data.size() != input_tensor->bytes) {
    QRB_DEBUG("ERROR: The size of input tensor should be ", input_tensor->bytes, ", but receive ", input_tensor_data.size(), "byte");
    return StatusCode::FAILURE;
  }

  // input_tensor->data.raw = input_tensor_data.data();
  memcpy(input_tensor->data.f, input_tensor_data.data(), input_tensor_data.size());

  if(TfLiteStatus::kTfLiteOk != this->interpreter_->Invoke()) {
    QRB_DEBUG("ERROR: TFLite invoke fail!");
    return StatusCode::FAILURE;
  }

  for(size_t i = 0; i < this->interpreter_->outputs().size(); i++) {
    auto result_tensor = this->interpreter_->output_tensor(i);
    OutputTensor output_tensor;

    output_tensor.output_tensor_name_ = result_tensor->name;

    output_tensor.output_tensor_data_ = std::vector<uint8_t>(result_tensor->bytes);
    memcpy(output_tensor.output_tensor_data_.data(), result_tensor->data.f, result_tensor->bytes); // reinterpret_cast<uint8_t*>

    for(auto j = 0; j < result_tensor->dims->size; j++) {
      output_tensor.output_tensor_shape_.emplace_back(result_tensor->dims->data[j]);
    }

    this->output_tensor_.emplace_back(output_tensor);
  }

  QRB_DEBUG("INFO: Inference execute Successfully!");

  return StatusCode::SUCCESS;
}

const std::vector<OutputTensor> QnnDelegateInference::get_output_tensors()
{
  return std::move(this->output_tensor_);
}

} // namespace qrb::inference_mgr