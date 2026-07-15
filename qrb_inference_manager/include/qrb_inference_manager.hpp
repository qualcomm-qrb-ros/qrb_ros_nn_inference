// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_INFERENCE_MANAGER_QRB_INFERENCE_MANAGER_HPP_
#define QRB_INFERENCE_MANAGER_QRB_INFERENCE_MANAGER_HPP_

#include <memory>
#include <string>

#include "qnn_inference/qnn_inference.hpp"
#include "qrb_inference.hpp"

namespace qrb::inference_mgr
{

class QrbInferenceManager
{
public:
  // Standard constructor: loads model_path independently.
  QrbInferenceManager(const std::string & model_path, const std::string & backend_option = "");

  // Shared-context constructor: borrows context and interface from QrbSharedContextLoader.
  // owner_graph_info: tensor metadata from the owner for this graph (for I/O buffer setup).
  QrbInferenceManager(Qnn_ContextHandle_t shared_context,
      QnnInterface * shared_interface,
      const std::string & graph_name,
      const GraphInfo * owner_graph_info = nullptr);

  ~QrbInferenceManager() = default;
  bool inference_execute(const std::vector<uint8_t> & input_tensor_data);
  // Zero-copy variant: passes src pointer directly to QNN without memcpy.
  bool inference_execute_ptr(const void * src, size_t src_size);
  // Hybrid: regular input copy + rpcmem output buffers (zero-copy output side).
  bool inference_execute_ptr_dmabuf_out(const void * src, size_t src_size);
  bool inference_execute_dmabuf(int dmabuf_fd, uint32_t dmabuf_size, uint64_t dmabuf_offset = 0);
  std::vector<OutputTensor> get_output_tensors();

private:
  std::unique_ptr<QrbInference> qrb_inference_{ nullptr };
};  // class QrbInferenceManager

// Loads a multi-graph QNN context binary once and vends per-graph QrbInferenceManager instances.
// Owns the QnnContext, QnnDevice, and QnnBackend for the lifetime of all vended managers.
// Must outlive all managers created via make_graph_manager().
class QrbSharedContextLoader
{
public:
  QrbSharedContextLoader(const std::string & model_path, const std::string & backend_option);
  ~QrbSharedContextLoader() = default;

  // Returns a QrbInferenceManager that executes the named graph within the shared context.
  // Throws std::logic_error if graph_name is not found or init fails.
  std::unique_ptr<QrbInferenceManager> make_graph_manager(const std::string & graph_name);

private:
  // Owns the context, device, and backend. Never used for inference — only for lifetime.
  std::unique_ptr<QnnInference> owner_;
};  // class QrbSharedContextLoader

}  // namespace qrb::inference_mgr

#endif
