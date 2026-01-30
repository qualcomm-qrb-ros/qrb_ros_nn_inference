// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qnn_inference/qnn_inference.hpp"

namespace qrb::inference_mgr
{

QnnInference::QnnInference(const std::string & model_path, const std::string & backend_option)
  : model_path_(model_path), backend_option_(backend_option)
{
  auto is_bin_model = (std::string::npos != model_path.find(".bin"));

  if (is_bin_model) {
    QRB_INFO("Loading model from binary file: ", model_path);

    load_model_from_binary = true;
    qnn_interface_ = std::make_unique<QnnInterface>(
        backend_option_, &backend_lib_handle, qnn_syslib_path_, &sys_lib_handle_);
  } else {
    qnn_interface_ = std::make_unique<QnnInterface>(
        model_path_, backend_option_, &backend_handle_, &model_handle_);
  }
}

QnnInference::~QnnInference()
{
  free_graphs_info();
  free_context();
  free_device();
  free_backend();

  if (backend_lib_handle) {
    ::dlclose(backend_lib_handle);
    backend_lib_handle = nullptr;
  }

  if (model_handle_) {
    ::dlclose(model_handle_);
    model_handle_ = nullptr;
  }

  if (sys_lib_handle_) {
    ::dlclose(sys_lib_handle_);
    sys_lib_handle_ = nullptr;
  }
}

StatusCode QnnInference::inference_init()
{
  if (initialize_backend() != StatusCode::SUCCESS) {
    return StatusCode::FAILURE;
  }

  if (create_device() != StatusCode::SUCCESS) {
    return StatusCode::FAILURE;
  }

  return StatusCode::SUCCESS;
}

StatusCode QnnInference::inference_graph_init()
{
  if (true == load_model_from_binary) {
    if (init_graph_from_binary() != StatusCode::SUCCESS) {
      return StatusCode::FAILURE;
    }
  } else {
    if (create_context() != StatusCode::SUCCESS) {
      return StatusCode::FAILURE;
    }

    if (compose_graphs() != StatusCode::SUCCESS) {
      return StatusCode::FAILURE;
    }

    if (finalize_graphs() != StatusCode::SUCCESS) {
      return StatusCode::FAILURE;
    }
  }

  return StatusCode::SUCCESS;
}

StatusCode QnnInference::inference_execute(const std::vector<uint8_t> & input_tensor_data)
{
  if (input_tensor_data.size() == 0) {
    QRB_ERROR("Input tensor is NULL!");
    return StatusCode::FAILURE;
  }

  for (uint32_t i = 0; i < graphs_count_; i++) {
    const auto & graphs_info = (*(graphs_info_))[i];

    auto io_tensors =
        QnnTensor(graphs_info.num_of_input_tensors, graphs_info.num_of_output_tensors);

    if (StatusCode::SUCCESS != io_tensors.setup_tensors(io_tensors.inputs,
                                   io_tensors.num_of_input_tensors, graphs_info.input_tensors)) {
      QRB_ERROR("Setup input tensors failed!");
      return StatusCode::FAILURE;
    }

    if (StatusCode::SUCCESS != io_tensors.setup_tensors(io_tensors.outputs,
                                   io_tensors.num_of_output_tensors, graphs_info.output_tensors)) {
      QRB_ERROR("Setup output tensors failed!");
      return StatusCode::FAILURE;
    }

    if (StatusCode::SUCCESS != io_tensors.write_input_tensors(input_tensor_data)) {
      QRB_ERROR("Write QNN input tensors failed!");
      return StatusCode::FAILURE;
    }

    if (QNN_GRAPH_NO_ERROR != this->qnn_interface_->interface.graphExecute(graphs_info.graph,
                                  io_tensors.inputs, io_tensors.num_of_input_tensors,
                                  io_tensors.outputs, io_tensors.num_of_output_tensors, nullptr,
                                  nullptr)) {
      QRB_ERROR("QNN graphExecute failed!");
      return StatusCode::FAILURE;
    }

#ifndef __hexagon__
    output_tensor_ = io_tensors.read_output_tensors(graphs_info.num_of_output_tensors);
    if (output_tensor_.size() == 0) {
      QRB_ERROR("Get ouput tensors failed!");
    }
#endif
  }

  return StatusCode::SUCCESS;
}

StatusCode QnnInference::inference_execute_dmabuf(
    int dmabuf_fd, uint32_t dmabuf_size, uint64_t dmabuf_offset)
{
  // Track previous inference output resources for cleanup.
  //
  // NOTE:
  // - Output RPCMEM buffers are freed by the downstream consumer (post-process node) using dmabuf_ptr.
  // - Therefore, we must NOT keep an owning RpcMemManager here; otherwise we'd risk double-free.
  // - We still must memDeRegister the output mem handles to avoid leaking QNN registrations.
  static std::vector<Qnn_MemHandle_t> prev_output_handles;
  static int inference_count = 0;
  inference_count++;

  QRB_INFO("========== INFERENCE #", inference_count, " START ==========");
  QRB_INFO("[MEMORY] Input: fd=", dmabuf_fd, " size=", dmabuf_size, " offset=", dmabuf_offset);

  if (dmabuf_fd < 0 || dmabuf_size == 0) {
    QRB_ERROR("Invalid DMA-BUF input: fd=", dmabuf_fd, " size=", dmabuf_size);
    return StatusCode::FAILURE;
  }

  // Clean up previous output handles before starting new inference
  if (!prev_output_handles.empty()) {
    QRB_INFO("[MEMORY] Cleaning up ", prev_output_handles.size(),
        " previous output handles from inference #", inference_count - 1);
    for (auto & handle : prev_output_handles) {
      if (handle != nullptr) {
        qnn_interface_->interface.memDeRegister(&handle, 1u);
      }
    }
    prev_output_handles.clear();
  }

  for (uint32_t g = 0; g < graphs_count_; g++) {
    const auto & graph_info = (*(graphs_info_))[g];

    // Current implementation supports single input tensor (common for the provided ROS sample).
    // Extending to multi-input requires offset/size per tensor.
    if (graph_info.num_of_input_tensors != 1) {
      QRB_ERROR("DMA-BUF input path currently supports single input tensor only. num_inputs=",
          graph_info.num_of_input_tensors);
      return StatusCode::FAILURE;
    }

    auto io_tensors = QnnTensor(graph_info.num_of_input_tensors, graph_info.num_of_output_tensors);

    if (StatusCode::SUCCESS != io_tensors.setup_tensors(io_tensors.inputs,
                                   io_tensors.num_of_input_tensors, graph_info.input_tensors)) {
      QRB_ERROR("Setup input tensors failed!");
      return StatusCode::FAILURE;
    }

    if (StatusCode::SUCCESS != io_tensors.setup_tensors(io_tensors.outputs,
                                   io_tensors.num_of_output_tensors, graph_info.output_tensors)) {
      QRB_ERROR("Setup output tensors failed!");
      return StatusCode::FAILURE;
    }

    // DMA-BUF path does not use QnnTensor's per-tensor clientBuf allocations. The setup_tensors()
    // implementation allocates clientBuf.data for each tensor via malloc(), which will accumulate
    // on the heap if not explicitly freed per frame. For zero-copy ION path we clear them here.
    for (uint32_t in_i = 0; in_i < graph_info.num_of_input_tensors; in_i++) {
      if (io_tensors.inputs[in_i].version == QNN_TENSOR_VERSION_1 &&
          io_tensors.inputs[in_i].v1.clientBuf.data != nullptr) {
        free(io_tensors.inputs[in_i].v1.clientBuf.data);
        io_tensors.inputs[in_i].v1.clientBuf.data = nullptr;
        io_tensors.inputs[in_i].v1.clientBuf.dataSize = 0;
      } else if (io_tensors.inputs[in_i].version == QNN_TENSOR_VERSION_2 &&
                 io_tensors.inputs[in_i].v2.clientBuf.data != nullptr) {
        free(io_tensors.inputs[in_i].v2.clientBuf.data);
        io_tensors.inputs[in_i].v2.clientBuf.data = nullptr;
        io_tensors.inputs[in_i].v2.clientBuf.dataSize = 0;
      }
    }
    for (uint32_t out_i = 0; out_i < graph_info.num_of_output_tensors; out_i++) {
      if (io_tensors.outputs[out_i].version == QNN_TENSOR_VERSION_1 &&
          io_tensors.outputs[out_i].v1.clientBuf.data != nullptr) {
        free(io_tensors.outputs[out_i].v1.clientBuf.data);
        io_tensors.outputs[out_i].v1.clientBuf.data = nullptr;
        io_tensors.outputs[out_i].v1.clientBuf.dataSize = 0;
      } else if (io_tensors.outputs[out_i].version == QNN_TENSOR_VERSION_2 &&
                 io_tensors.outputs[out_i].v2.clientBuf.data != nullptr) {
        free(io_tensors.outputs[out_i].v2.clientBuf.data);
        io_tensors.outputs[out_i].v2.clientBuf.data = nullptr;
        io_tensors.outputs[out_i].v2.clientBuf.dataSize = 0;
      }
    }

    // Register input shared buffer as ION (RPCMEM-backed fd).
    Qnn_MemDescriptor_t input_mem_desc = QNN_MEM_DESCRIPTOR_INIT;
    input_mem_desc.memShape = {io_tensors.inputs[0].v1.rank, io_tensors.inputs[0].v1.dimensions, nullptr};
    input_mem_desc.dataType = io_tensors.inputs[0].v1.dataType;
    input_mem_desc.memType = QNN_MEM_TYPE_ION;
    input_mem_desc.ionInfo.fd = dmabuf_fd;

    Qnn_MemHandle_t input_mem_handle = nullptr;
    auto rc = qnn_interface_->interface.memRegister(context_, &input_mem_desc, 1u, &input_mem_handle);
    if (QNN_SUCCESS != rc) {
      const char * err_msg = nullptr;
      qnn_interface_->interface.errorGetMessage(rc, &err_msg);
      QRB_ERROR("memRegister(input DMA-BUF) failed: ", (err_msg ? err_msg : "unknown"), " (", rc, ")");
      return StatusCode::FAILURE;
    }

    io_tensors.inputs[0].v1.memType = QNN_TENSORMEMTYPE_MEMHANDLE;
    io_tensors.inputs[0].v1.memHandle = input_mem_handle;

    // Register each output tensor as its own ION buffer (rpcmem-backed) to get a distinct fd.
    std::vector<Qnn_MemHandle_t> output_mem_handles(graph_info.num_of_output_tensors, nullptr);
    std::vector<int> output_fds(graph_info.num_of_output_tensors, -1);
    std::vector<uint32_t> output_sizes(graph_info.num_of_output_tensors, 0);
    std::vector<void *> output_ptrs(graph_info.num_of_output_tensors, nullptr);

    constexpr int RPCMEM_HEAP_ID_SYSTEM = 25;
    constexpr uint32_t RPCMEM_DEFAULT_FLAGS = 1;

    auto cleanup_outputs = [&] {
      for (uint32_t i = 0; i < graph_info.num_of_output_tensors; i++) {
        if (output_mem_handles[i] != nullptr) {
          qnn_interface_->interface.memDeRegister(&output_mem_handles[i], 1u);
          output_mem_handles[i] = nullptr;
        }
      }
      qnn_interface_->interface.memDeRegister(&input_mem_handle, 1u);
    };

    for (uint32_t out_i = 0; out_i < graph_info.num_of_output_tensors; out_i++) {
      auto * out_tensor = &(io_tensors.outputs[out_i]);

      auto shape = io_tensors.get_tensor_shape(out_tensor);
      uint32_t output_tensor_size = io_tensors.get_tensor_size(out_tensor, shape);

      // IMPORTANT:
      // output RPCMEM buffer ownership is transferred to downstream via dmabuf_ptr, where it will be
      // freed using rpcmem_free(ptr). Therefore, we must NOT keep an owning RpcMemManager here;
      // otherwise it will free the buffer at end of scope (causing invalid output).
      auto rpc_mgr = std::make_shared<RpcMemManager>();
      if (StatusCode::SUCCESS != rpc_mgr->init()) {
        QRB_ERROR("RpcMemManager init failed");
        cleanup_outputs();
        return StatusCode::FAILURE;
      }

      if (StatusCode::SUCCESS != rpc_mgr->alloc(output_tensor_size, RPCMEM_HEAP_ID_SYSTEM, RPCMEM_DEFAULT_FLAGS)) {
        QRB_ERROR("RpcMemManager alloc failed for size: ", output_tensor_size);
        cleanup_outputs();
        return StatusCode::FAILURE;
      }

      int fd = rpc_mgr->get_fd();
      void * ptr = rpc_mgr->get_ptr();
      QRB_INFO("[MEMORY] Allocated output[", out_i, "]: ptr=", ptr, " fd=", fd, " size=", output_tensor_size);

      Qnn_MemDescriptor_t out_mem_desc = QNN_MEM_DESCRIPTOR_INIT;
      out_mem_desc.memShape = {out_tensor->v1.rank, out_tensor->v1.dimensions, nullptr};
      out_mem_desc.dataType = out_tensor->v1.dataType;
      out_mem_desc.memType = QNN_MEM_TYPE_ION;
      out_mem_desc.ionInfo.fd = fd;

      Qnn_MemHandle_t out_mem_handle = nullptr;
      auto out_rc = qnn_interface_->interface.memRegister(context_, &out_mem_desc, 1u, &out_mem_handle);
      if (QNN_SUCCESS != out_rc) {
        const char * err_msg = nullptr;
        qnn_interface_->interface.errorGetMessage(out_rc, &err_msg);
        QRB_ERROR("memRegister(output ION) failed: ", (err_msg ? err_msg : "unknown"), " (", out_rc, ")");
        cleanup_outputs();
        return StatusCode::FAILURE;
      }

      out_tensor->v1.memType = QNN_TENSORMEMTYPE_MEMHANDLE;
      out_tensor->v1.memHandle = out_mem_handle;

      output_mem_handles[out_i] = out_mem_handle;
      output_fds[out_i] = fd;
      output_sizes[out_i] = output_tensor_size;
      output_ptrs[out_i] = ptr;

      // Transfer ownership to downstream. Downstream will call rpcmem_free(ptr) using dmabuf_ptr.
      // Ensure our destructor does NOT free the buffer. We still keep this object alive until the
      // next inference cycle to keep libcdsprpc handle and symbol pointers valid.
      rpc_mgr->disown();
      static std::vector<std::shared_ptr<RpcMemManager>> s_output_keepalive;
      s_output_keepalive.push_back(rpc_mgr);
    }

    // Execute graph
    QRB_DEBUG("=== About to execute graph ===");
    QRB_DEBUG("Graph handle: ", graph_info.graph);
    QRB_DEBUG("Num inputs: ", io_tensors.num_of_input_tensors);
    QRB_DEBUG("Num outputs: ", io_tensors.num_of_output_tensors);
    QRB_DEBUG("Input[0] memType: ", io_tensors.inputs[0].v1.memType);
    QRB_DEBUG("Calling graphExecute...");

    auto exec_rc = qnn_interface_->interface.graphExecute(graph_info.graph,
                                  io_tensors.inputs, io_tensors.num_of_input_tensors,
                                  io_tensors.outputs, io_tensors.num_of_output_tensors, nullptr,
                                  nullptr);

    QRB_DEBUG("graphExecute returned: ", exec_rc);

    if (QNN_GRAPH_NO_ERROR != exec_rc) {
      QRB_ERROR("QNN graphExecute failed with code: ", exec_rc);
      cleanup_outputs();
      return StatusCode::FAILURE;
    }

    QRB_DEBUG("graphExecute succeeded!");

    // Immediately deregister input handle after graph execution completes
    // The input memory is owned by the caller (pre-process node) and will be reused
    QRB_DEBUG("Deregistering input_mem_handle after graph execution...");
    auto input_dereg_rc = qnn_interface_->interface.memDeRegister(&input_mem_handle, 1u);
    QRB_DEBUG("Input memDeRegister returned: ", input_dereg_rc);

#ifndef __hexagon__
    // Produce OutputTensor list with DMA-BUF metadata and no data copy.
    QRB_DEBUG("=== Processing output tensors ===");
    QRB_DEBUG("Number of output tensors: ", graph_info.num_of_output_tensors);

    output_tensor_.clear();
    output_tensor_.reserve(graph_info.num_of_output_tensors);

    for (uint32_t out_i = 0; out_i < graph_info.num_of_output_tensors; out_i++) {
      const auto * out_tensor = &(io_tensors.outputs[out_i]);

      auto shape = io_tensors.get_tensor_shape(out_tensor);

      OutputTensor ot;
      ot.output_tensor_name = out_tensor->v1.name;

      ot.output_tensor_shape.reserve(shape.size());
      for (size_t i = 0; i < shape.size(); i++) {
        ot.output_tensor_shape.push_back(static_cast<uint32_t>(shape[i]));
      }

      ot.data_type = io_tensors.qnn_dtype_to_qrb_dtype(out_tensor->v1.dataType);

      // tensor data is in DMA-BUF
      ot.output_dmabuf_fd = output_fds[out_i];
      ot.output_dmabuf_offset = dmabuf_offset;
      ot.output_dmabuf_size = output_sizes[out_i];
      // Downstream will free this RPCMEM allocation using rpcmem_free(ptr).
      ot.output_dmabuf_ptr = reinterpret_cast<uint64_t>(output_ptrs[out_i]);

      output_tensor_.emplace_back(std::move(ot));
    }
#endif

    // IMPORTANT: Manually free both input and output tensors to prevent destructor issues
    // Input/output dimensions and names point to graph_info which is still valid
    // We must prevent double-free by clearing these pointers before destructor runs

    QRB_DEBUG("Manually freeing input tensors...");
    for (uint32_t in_i = 0; in_i < graph_info.num_of_input_tensors; in_i++) {
      io_tensors.inputs[in_i].v1.memType = QNN_TENSORMEMTYPE_RAW;
      io_tensors.inputs[in_i].v1.clientBuf.data = nullptr;
      io_tensors.inputs[in_i].v1.clientBuf.dataSize = 0;
      io_tensors.inputs[in_i].v1.dimensions = nullptr;
      io_tensors.inputs[in_i].v1.name = nullptr;
    }
    free(io_tensors.inputs);
    io_tensors.inputs = nullptr;
    io_tensors.num_of_input_tensors = 0;
    QRB_DEBUG("Input tensors manually freed");

    QRB_DEBUG("Manually freeing output tensors...");
    for (uint32_t out_i = 0; out_i < graph_info.num_of_output_tensors; out_i++) {
      io_tensors.outputs[out_i].v1.memType = QNN_TENSORMEMTYPE_RAW;
      io_tensors.outputs[out_i].v1.clientBuf.data = nullptr;
      io_tensors.outputs[out_i].v1.clientBuf.dataSize = 0;
      io_tensors.outputs[out_i].v1.dimensions = nullptr;
      io_tensors.outputs[out_i].v1.name = nullptr;
    }
    free(io_tensors.outputs);
    io_tensors.outputs = nullptr;
    io_tensors.num_of_output_tensors = 0;
    QRB_DEBUG("Output tensors manually freed");

    // Save output handles for cleanup in next inference (deregister only).
    // Output RPCMEM is owned by downstream consumer.
    prev_output_handles = std::move(output_mem_handles);
    QRB_DEBUG("Saved ", prev_output_handles.size(), " output handles for next cleanup");
  }

  QRB_DEBUG("inference_execute_dmabuf returning SUCCESS");
  return StatusCode::SUCCESS;
}

const std::vector<OutputTensor> QnnInference::get_output_tensors()
{
  return std::move(output_tensor_);
}

StatusCode QnnInference::initialize_backend()
{
  auto qnn_status = qnn_interface_->interface.backendCreate(
      nullptr, (const QnnBackend_Config_t **)(nullptr), &(backend_handle_));

  if (QNN_BACKEND_NO_ERROR != qnn_status) {
    QRB_ERROR("Could not initialize backend due to error = %d!", qnn_status);
    return StatusCode::FAILURE;
  }

  QRB_INFO(backend_option_, " initialize successfully");
  return StatusCode::SUCCESS;
}

StatusCode QnnInference::create_device()
{
  auto is_device_property_supported = [this] {
    if (nullptr != qnn_interface_->interface.propertyHasCapability) {
      auto qnn_status = qnn_interface_->interface.propertyHasCapability(QNN_PROPERTY_GROUP_DEVICE);

      if (QNN_PROPERTY_NOT_SUPPORTED == qnn_status) {
        QRB_WARNING("Device property is not supported!");
      } else if (QNN_PROPERTY_ERROR_UNKNOWN_KEY == qnn_status) {
        QRB_ERROR("Device property is not known to backend!");
        return StatusCode::FAILURE;
      }
    }
    return StatusCode::SUCCESS;
  };

  if (StatusCode::FAILURE != is_device_property_supported()) {
    if (nullptr != qnn_interface_->interface.deviceCreate) {
      auto qnn_status = qnn_interface_->interface.deviceCreate(nullptr, nullptr, &(device_handle_));

      if (QNN_SUCCESS != qnn_status && QNN_DEVICE_ERROR_UNSUPPORTED_FEATURE != qnn_status) {
        QRB_ERROR("Failed to create device!");
        return StatusCode::FAILURE;
      }
    }
    support_device_ = true;
  }

  QRB_INFO("Qnn device initialize successfully");
  return StatusCode::SUCCESS;
}

StatusCode QnnInference::create_context()
{
  if (QNN_CONTEXT_NO_ERROR != qnn_interface_->interface.contextCreate(
                                  backend_handle_, device_handle_, nullptr, &(context_))) {
    QRB_ERROR("Could not create context!");
    return StatusCode::FAILURE;
  }
  return StatusCode::SUCCESS;
}

StatusCode QnnInference::compose_graphs()
{
  if (ModelError::MODEL_NO_ERROR !=
      qnn_interface_->compose_graphs(backend_handle_, qnn_interface_->interface, context_, nullptr,
          0, &(graphs_info_), &(graphs_count_), false, nullptr, QNN_LOG_LEVEL_MAX)) {
    QRB_ERROR("Failed in composeGraphs()!");
    return StatusCode::FAILURE;
  }
  return StatusCode::SUCCESS;
}

StatusCode QnnInference::finalize_graphs()
{
  for (size_t i = 0; i < graphs_count_; i++) {
    if (QNN_GRAPH_NO_ERROR !=
        qnn_interface_->interface.graphFinalize((*graphs_info_)[i].graph, nullptr, nullptr)) {
      return StatusCode::FAILURE;
    }
  }

  return StatusCode::SUCCESS;
}

void QnnInference::free_graphs_info()
{
  if (graphs_info_ == nullptr) {
    return;
  }

  for (uint32_t i = 0; i < graphs_count_; i++) {
    auto & graph_info = graphs_info_[i];
    free(graph_info->graph_name);
    graph_info->graph_name = nullptr;

    QnnTensor tensor_ops;
    tensor_ops.free_qnn_tensors(graph_info->input_tensors, graph_info->num_of_input_tensors);
    tensor_ops.free_qnn_tensors(graph_info->output_tensors, graph_info->num_of_output_tensors);
  }

  free(*graphs_info_);
  *graphs_info_ = nullptr;
  free(graphs_info_);
  graphs_info_ = nullptr;
}

void QnnInference::free_context()
{
  if (QNN_CONTEXT_NO_ERROR != qnn_interface_->interface.contextFree(context_, nullptr)) {
    QRB_ERROR("Failed to free context!");
  }

  context_ = nullptr;
}

void QnnInference::free_device()
{
  if (true == support_device_) {
    if (nullptr != qnn_interface_->interface.deviceFree) {
      auto qnn_status = qnn_interface_->interface.deviceFree(device_handle_);
      if (QNN_SUCCESS != qnn_status && QNN_DEVICE_ERROR_UNSUPPORTED_FEATURE != qnn_status) {
        QRB_ERROR("Failed to free device!");
      }
    }
  }

  device_handle_ = nullptr;
}

void QnnInference::free_backend()
{
  if (qnn_interface_->interface.backendFree != nullptr) {
    if (QNN_BACKEND_NO_ERROR != qnn_interface_->interface.backendFree(backend_handle_)) {
      QRB_ERROR("Could not free backend!");
    }
  }

  backend_handle_ = nullptr;
}

StatusCode QnnInference::init_graph_from_binary()
{
  auto [model_buf, model_buf_size] = read_binary_model();
  if (nullptr == model_buf) {
    return StatusCode::FAILURE;
  }

  if (StatusCode::SUCCESS != get_and_set_graph_info_from_binary(model_buf, model_buf_size)) {
    return StatusCode::FAILURE;
  }

  if (StatusCode::SUCCESS != create_context_from_binary(model_buf, model_buf_size)) {
    return StatusCode::FAILURE;
  }

  QRB_INFO("Initialize Qnn graph from binary file successfully");
  return StatusCode::SUCCESS;
}

std::tuple<std::shared_ptr<uint8_t[]>, uint64_t> QnnInference::read_binary_model()
{
  uint64_t buf_size = 0;
  std::ifstream model_binary(model_path_, std::ifstream::binary);
  if (!model_binary.is_open() || model_binary.fail()) {
    QRB_ERROR("Fail to open model file: ", model_path_);
    return { nullptr, 0 };
  }

  model_binary.seekg(0, model_binary.end);
  buf_size = model_binary.tellg();
  model_binary.seekg(0, model_binary.beg);
  if (0 == buf_size) {
    QRB_ERROR("Received path to an empty file. Nothing to deserialize.");
    return { nullptr, 0 };
  }

  std::shared_ptr<uint8_t[]> model_buf(new uint8_t[buf_size]);
  if (!model_binary.read(reinterpret_cast<char *>(model_buf.get()), buf_size)) {
    QRB_ERROR("Fail to read model file: ", model_path_);
    return { nullptr, 0 };
  }

  return { model_buf, buf_size };
}

StatusCode QnnInference::get_and_set_graph_info_from_binary(
    const std::shared_ptr<uint8_t[]> model_buf,
    const uint64_t model_buf_size)
{
  QnnSystemContext_Handle_t sys_context_headle = nullptr;
  if (QNN_SUCCESS !=
      qnn_interface_->qnn_system_interface.systemContextCreate(&sys_context_headle)) {
    QRB_ERROR("Could not create system handle.");
    return StatusCode::FAILURE;
  }

  const QnnSystemContext_BinaryInfo_t * binary_info = nullptr;
  Qnn_ContextBinarySize_t binary_info_size = 0;
  if (QNN_SUCCESS !=
      qnn_interface_->qnn_system_interface.systemContextGetBinaryInfo(sys_context_headle,
          static_cast<void *>(model_buf.get()), model_buf_size, &binary_info, &binary_info_size)) {
    QRB_ERROR("Failed to get context binary info.");
    return StatusCode::FAILURE;
  }

  if (StatusCode::SUCCESS != set_up_graph_info(binary_info)) {
    return StatusCode::FAILURE;
  }

  qnn_interface_->qnn_system_interface.systemContextFree(sys_context_headle);
  sys_context_headle = nullptr;

  return StatusCode::SUCCESS;
}

template <typename T>
StatusCode QnnInference::copy_graph_info(T graph_info_from_binary)
{
  graphs_info_ = (GraphInfo **)calloc(graphs_count_, sizeof(GraphInfo *));
  if (nullptr == graphs_info_) {
    return StatusCode::FAILURE;
  }

  for (uint32_t i = 0; i < graphs_count_; i++) {
    auto graph_info_dst = (GraphInfo *)calloc(1, sizeof(GraphInfo));
    if (nullptr == graph_info_dst) {
      return StatusCode::FAILURE;
    }

    graph_info_dst->graph_name =
        (char *)malloc(sizeof(char) * strlen(graph_info_from_binary.graphName));
    if (nullptr == graph_info_dst->graph_name) {
      return StatusCode::FAILURE;
    }
    memcpy(graph_info_dst->graph_name, graph_info_from_binary.graphName,
        strlen(graph_info_from_binary.graphName));

    auto set_up_tensors_info = [](const Qnn_Tensor_t * src, const uint32_t cnt,
                                   Qnn_Tensor_t *& dst) {
      dst = (Qnn_Tensor_t *)calloc(cnt, sizeof(Qnn_Tensor_t));
      if (nullptr == dst) {
        return StatusCode::FAILURE;
      }

      QnnTensor tensor_ops;

      for (size_t i = 0; i < cnt; i++) {
        dst[i] = QNN_TENSOR_INIT;
        if (StatusCode::SUCCESS != tensor_ops.tensor_info_deep_copy(&dst[i], &src[i])) {
          return StatusCode::FAILURE;
        }
      }

      return StatusCode::SUCCESS;
    };

    graph_info_dst->num_of_input_tensors = graph_info_from_binary.numGraphInputs;
    if (StatusCode::SUCCESS != set_up_tensors_info(graph_info_from_binary.graphInputs,
                                   graph_info_from_binary.numGraphInputs,
                                   graph_info_dst->input_tensors)) {
      return StatusCode::FAILURE;
    }

    graph_info_dst->num_of_output_tensors = graph_info_from_binary.numGraphOutputs;
    if (StatusCode::SUCCESS != set_up_tensors_info(graph_info_from_binary.graphOutputs,
                                   graph_info_from_binary.numGraphOutputs,
                                   graph_info_dst->output_tensors)) {
      return StatusCode::FAILURE;
    }

    graphs_info_[i] = graph_info_dst;
  }

  return StatusCode::SUCCESS;
}

/// @brief set up graphs_info_ based on binary_info
/// @param binary_info
/// @return SUCCESS or FAILURE
StatusCode QnnInference::set_up_graph_info(const QnnSystemContext_BinaryInfo_t * binary_info)
{
  if (nullptr == binary_info) {
    return StatusCode::FAILURE;
  }

  switch (binary_info->version) {
    case QNN_SYSTEM_CONTEXT_BINARY_INFO_VERSION_1: {
      graphs_count_ = binary_info->contextBinaryInfoV1.numGraphs;
      if (StatusCode::SUCCESS !=
          copy_graph_info(binary_info->contextBinaryInfoV1.graphs->graphInfoV1)) {
        return StatusCode::FAILURE;
      }
      break;
    }
    case QNN_SYSTEM_CONTEXT_BINARY_INFO_VERSION_2: {
      graphs_count_ = binary_info->contextBinaryInfoV2.numGraphs;
      if (StatusCode::SUCCESS !=
          copy_graph_info(binary_info->contextBinaryInfoV2.graphs->graphInfoV2)) {
        return StatusCode::FAILURE;
      }
      break;
    }
    case QNN_SYSTEM_CONTEXT_BINARY_INFO_VERSION_3: {
      graphs_count_ = binary_info->contextBinaryInfoV3.numGraphs;
      if (StatusCode::SUCCESS !=
          copy_graph_info(binary_info->contextBinaryInfoV3.graphs->graphInfoV3)) {
        return StatusCode::FAILURE;
      }
      break;
    }
    default: {
      QRB_ERROR("Unrecognized system context binary info version.");
      return StatusCode::FAILURE;
    }
  }

  return StatusCode::SUCCESS;
}

StatusCode QnnInference::create_context_from_binary(const std::shared_ptr<uint8_t[]> model_buf,
    const uint64_t model_buf_size)
{
  if (qnn_interface_->interface.contextCreateFromBinary(backend_handle_, device_handle_, nullptr,
          static_cast<void *>(model_buf.get()), model_buf_size, &context_, nullptr)) {
    QRB_ERROR("Could not create context from binary!");
    return StatusCode::FAILURE;
  }

  for (uint32_t i = 0; i < graphs_count_; i++) {
    if (QNN_SUCCESS != qnn_interface_->interface.graphRetrieve(
                           context_, (*graphs_info_)[i].graph_name, &((*graphs_info_)[i].graph))) {
      QRB_ERROR("Unable to retrieve graph handle for the graph");
      return StatusCode::FAILURE;
    }
  }
  return StatusCode::SUCCESS;
}

}  // namespace qrb::inference_mgr
