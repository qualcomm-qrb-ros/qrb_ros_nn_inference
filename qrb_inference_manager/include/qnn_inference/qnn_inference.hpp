// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_INFERENCE_MANAGER_QNN_INFERENCE_HPP_
#define QRB_INFERENCE_MANAGER_QNN_INFERENCE_HPP_

#include "qnn_inference/qnn_inference_impl.hpp"
#include "qrb_inference.hpp"

namespace qrb::inference_mgr
{

class QnnInference : public QrbInference
{
public:
  QnnInference(const std::string & model_path,
      const std::string & backend_option,
      const std::string & qnn_syslib_path);
  QnnInference(const std::string & model_path, const std::string & backend_option);
  ~QnnInference();
  StatusCode inference_init() override;
  StatusCode inference_graph_init() override;
  StatusCode inference_execute(const std::vector<uint8_t> & input_tensor_data) override;
  const std::vector<OutputTensor> get_output_tensors() override;

private:
  const std::string model_path_;
  const std::string backend_option_;
  const std::string qnn_syslib_path_;
  bool load_model_from_binary = false;
  void * backend_lib_handle = nullptr;
  void * sys_lib_handle_ = nullptr;
  void * backend_handle_ = nullptr;
  void * model_handle_ = nullptr;
  Qnn_DeviceHandle_t device_handle_ = nullptr;
  Qnn_ContextHandle_t context_ = nullptr;
  GraphInfo ** graphs_info_ = nullptr;
  uint32_t graphs_count_ = 0;
  bool support_device = false;
  std::vector<OutputTensor> output_tensor_;
  std::unique_ptr<QnnInterface> qnn_interface_{ nullptr };

  StatusCode initialize_backend();
  StatusCode create_device();
  StatusCode create_context();
  StatusCode compose_graphs();
  StatusCode finalize_graphs();
  void free_graphs_info();
  void free_context();
  void free_device();
  void free_backend();

private:
  StatusCode init_graph_from_binary();
  std::tuple<std::shared_ptr<uint8_t[]>, uint64_t> read_binary_model();
  StatusCode get_and_set_graph_info_from_binary(const std::shared_ptr<uint8_t[]> model_buf,
      const uint64_t model_buf_size);
  StatusCode set_up_graph_info(const QnnSystemContext_BinaryInfo_t * binary_info);
  StatusCode create_context_from_binary(const std::shared_ptr<uint8_t[]> model_buf,
      const uint64_t model_buf_size);
};  // class QnnInference

}  // namespace qrb::inference_mgr

#endif