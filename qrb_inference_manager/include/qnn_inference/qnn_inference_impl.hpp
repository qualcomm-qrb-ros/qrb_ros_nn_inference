// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QNN_INFERENCE_IMPL_HPP_
#define QNN_INFERENCE_IMPL_HPP_

#include <cstdint>
#include <vector>
#include <unordered_map>

#include "qrb_inference.hpp"
#include "IOTensor.hpp"

namespace qrb::inference_mgr
{

class QnnInferenceCommon { // common part of QnnInferenceImpl and QnnInferenceFromFileImpl
public:
  QnnInferenceCommon(const std::string &backend_option, const std::string &model_path);
  ~QnnInferenceCommon();
  StatusCode initialize();
  StatusCode initialize_backend();
  StatusCode create_device();
  StatusCode create_context();
  StatusCode compose_graphs();
  StatusCode finalize_graphs();

  const std::string backend_option_;
  const std::string model_path_;
  bool support_device = false;
  uint32_t graphs_count_ = 0;
  void *backend_handle_ = nullptr;
  void *model_handle_ = nullptr;
  qnn_wrapper_api::GraphInfo_t **graphs_info_ = nullptr;
  Qnn_ContextHandle_t context_ = nullptr;
  Qnn_DeviceHandle_t device_handle_ = nullptr;
  Qnn_LogHandle_t log_handle_ = nullptr;
  qnn::tools::sample_app::QnnFunctionPointers qnn_function_pointers_;

private:
  void free_context();
  void free_device();
}; // class QnnInferenceCommon

class QnnInferenceImpl {
public:
  QnnInferenceImpl(const std::string &backend_option, const std::string &model_path);
  ~QnnInferenceImpl() = default;
  StatusCode initialize();
  StatusCode initialize_backend();
  StatusCode create_device();
  StatusCode create_context();
  StatusCode compose_graphs();
  StatusCode finalize_graphs();
  StatusCode execute_graphs(const std::vector<uint8_t> &input_tensor_data);
  const std::vector<OutputTensor> get_output_tensors();

private:
  std::vector<OutputTensor> output_tensor_;
  std::unique_ptr<QnnInferenceCommon> qnn_inference_common_{nullptr};

  StatusCode populateInputTensorsFromBuffer(Qnn_Tensor_t* input, const std::vector<uint8_t> &input_tensor_data);
  StatusCode writeOutputTensorsToBuffer(Qnn_Tensor_t* outputs, uint32_t number_of_outputs);
}; // class QnnInferenceImpl

class QnnInferenceFromFileImpl {
public:
  QnnInferenceFromFileImpl(const std::string &backend_option, const std::string &model_path);
  ~QnnInferenceFromFileImpl() = default;
  StatusCode initialize();
  StatusCode initialize_backend();
  StatusCode create_device();
  StatusCode create_context();
  StatusCode compose_graphs();
  StatusCode finalize_graphs();
  StatusCode execute_graphs();

private:
  std::unique_ptr<QnnInferenceCommon> qnn_inference_common_{nullptr};
  std::string inputfile_path_;
  std::string outputfile_path_;
  std::vector<std::vector<std::vector<std::string>>> inputfile_lists_;
  std::vector<std::unordered_map<std::string, uint32_t>> inputname_to_index_;
}; // class QnnInferenceFromFileImpl

} // namespace qrb::inference_mgr

#endif