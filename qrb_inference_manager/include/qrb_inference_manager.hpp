// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_INFERENCE_MANAGER_QRB_INFERENCE_MANAGER_HPP_
#define QRB_INFERENCE_MANAGER_QRB_INFERENCE_MANAGER_HPP_

#include "qrb_inference.hpp"

namespace qrb::inference_mgr
{

class QrbInferenceManager
{
public:
  QrbInferenceManager(const std::string &model_path,
                      const std::string &backend_option = "",
                      const std::string &qnn_syslib_path = "");
  ~QrbInferenceManager() = default;
  bool inference_execute(const std::vector<uint8_t> &input_tensor_data);
  std::vector<OutputTensor> get_output_tensors();

private:
  std::unique_ptr<QrbInference> qrb_inference_{nullptr};
}; // class QrbInferenceManager

} // namespace qrb::inference_mgr

#endif