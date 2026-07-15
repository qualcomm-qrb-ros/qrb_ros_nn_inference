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

/**
 * \brief Shared-context constructor: borrows an already-created QNN context.
 *        Only the named graph is retrieved; context lifetime is managed externally.
 * \param shared_context  QNN context handle owned by QrbSharedContextLoader
 * \param shared_interface  QnnInterface pointer owned by QrbSharedContextLoader
 * \param graph_name  name of the graph to retrieve from the shared context
 * \throw std::logic_error if graph retrieval or init fails
 */
QrbInferenceManager::QrbInferenceManager(Qnn_ContextHandle_t shared_context,
    QnnInterface * shared_interface,
    const std::string & graph_name,
    const GraphInfo * owner_graph_info)
{
  auto inference = std::make_unique<QnnInference>(
      shared_context, shared_interface, graph_name, owner_graph_info);

  if (inference->inference_init() != StatusCode::SUCCESS) {
    throw std::logic_error("ERROR: Shared-context inference init fail for graph: " + graph_name);
  }

  if (inference->inference_graph_init() != StatusCode::SUCCESS) {
    throw std::logic_error(
        "ERROR: Shared-context inference graph init fail for graph: " + graph_name);
  }

  qrb_inference_ = std::move(inference);
}

bool QrbInferenceManager::inference_execute(const std::vector<uint8_t> & input_tensor_data)
{
  if (qrb_inference_->inference_execute(input_tensor_data) == StatusCode::SUCCESS) {
    return true;
  }
  return false;
}

bool QrbInferenceManager::inference_execute_ptr(const void * src, size_t src_size)
{
  auto * qnn_inference = dynamic_cast<QnnInference *>(qrb_inference_.get());
  if (nullptr == qnn_inference) {
    QRB_ERROR("inference_execute_ptr is only supported by QnnInference.");
    return false;
  }
  return qnn_inference->inference_execute_ptr(src, src_size) == StatusCode::SUCCESS;
}

bool QrbInferenceManager::inference_execute_ptr_dmabuf_out(const void * src, size_t src_size)
{
  auto * qnn_inference = dynamic_cast<QnnInference *>(qrb_inference_.get());
  if (nullptr == qnn_inference) {
    QRB_ERROR("inference_execute_ptr_dmabuf_out is only supported by QnnInference.");
    return false;
  }
  return qnn_inference->inference_execute_ptr_dmabuf_out(src, src_size) == StatusCode::SUCCESS;
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

// ---------------------------------------------------------------------------
// QrbSharedContextLoader
// ---------------------------------------------------------------------------

/**
 * \brief Load a multi-graph QNN context binary and own the resulting context.
 *        Use make_graph_manager() to obtain per-graph inference handles.
 * \param model_path     path to the combined multi-graph .bin file
 * \param backend_option backend lib (e.g. "/usr/lib/libQnnHtp.so")
 * \throw std::logic_error if loading or init fails
 */
QrbSharedContextLoader::QrbSharedContextLoader(const std::string & model_path,
    const std::string & backend_option)
{
  owner_ = std::make_unique<QnnInference>(model_path, backend_option);

  if (owner_->inference_init() != StatusCode::SUCCESS) {
    throw std::logic_error("ERROR: QrbSharedContextLoader backend/device init failed");
  }

  if (owner_->inference_graph_init() != StatusCode::SUCCESS) {
    throw std::logic_error("ERROR: QrbSharedContextLoader context/graph init failed");
  }

  QRB_INFO("QrbSharedContextLoader: shared context loaded from ", model_path);
}

/**
 * \brief Create a QrbInferenceManager that executes the named graph within the shared context.
 * \param graph_name  name of the graph as embedded in the context binary
 * \return unique_ptr to a ready-to-use QrbInferenceManager
 * \throw std::logic_error if graph_name is not found or init fails
 */
std::unique_ptr<QrbInferenceManager> QrbSharedContextLoader::make_graph_manager(
    const std::string & graph_name)
{
  const GraphInfo * gi = owner_->get_graph_info_for(graph_name);
  if (gi == nullptr) {
    QRB_WARNING("make_graph_manager: no GraphInfo found for '", graph_name,
        "' — tensor metadata will be unavailable");
  }
  return std::make_unique<QrbInferenceManager>(
      owner_->get_context(), owner_->get_interface(), graph_name, gi);
}

}  // namespace qrb::inference_mgr
