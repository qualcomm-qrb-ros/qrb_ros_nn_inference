// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_INFERENCE_MANAGER_QNN_DELEGATE_INFERENCE_HPP_
#define QRB_INFERENCE_MANAGER_QNN_DELEGATE_INFERENCE_HPP_

#include <tensorflow/lite/c/c_api.h>
#include <tensorflow/lite/c/c_api_experimental.h>
#include <tensorflow/lite/c/common.h>

#include <string>

#include "qrb_inference.hpp"

namespace qrb::inference_mgr
{

class QnnDelegateInference : public QrbInference
{
public:
  QnnDelegateInference(const std::string & model_path, const std::string & backend_option);
  ~QnnDelegateInference();
  StatusCode inference_init() override;
  StatusCode inference_graph_init() override;
  StatusCode inference_execute(const std::vector<uint8_t> & input_tensor_data) override;
  const std::vector<OutputTensor> get_output_tensors() override;

private:
  const std::string model_path_;
  const std::string backend_option_ = "";
  std::vector<OutputTensor> output_tensor_;

  TfLiteModel * model_ = nullptr;
  TfLiteInterpreterOptions * options_ = nullptr;
  TfLiteInterpreter * interpreter_ = nullptr;
  TfLiteDelegate * delegate_ = nullptr;

  StatusCode register_qnn_delegate();
};
}  // namespace qrb::inference_mgr

#endif
