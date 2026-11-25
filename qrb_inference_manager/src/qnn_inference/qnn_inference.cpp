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

QnnInference::QnnInference(Qnn_ContextHandle_t shared_context,
    QnnInterface * shared_interface,
    const std::string & graph_name,
    const GraphInfo * owner_graph_info)
  : owned_context_(false), borrowed_interface_(shared_interface)
{
  // Borrow the already-created context and interface — do not take ownership.
  context_ = shared_context;

  QRB_INFO("Retrieving shared-context graph: '", graph_name, "'");

  // Allocate graphs_info_ as a pointer to a contiguous GraphInfo array (same layout as owner).
  graphs_info_ = (GraphInfo **)calloc(1, sizeof(GraphInfo *));
  if (graphs_info_ == nullptr) {
    QRB_ERROR("Failed to allocate graphs_info_ for shared-context graph: ", graph_name);
    return;
  }
  *graphs_info_ = (GraphInfo *)calloc(1, sizeof(GraphInfo));
  if (*graphs_info_ == nullptr) {
    QRB_ERROR("Failed to allocate GraphInfo for shared-context graph: ", graph_name);
    free(graphs_info_);
    graphs_info_ = nullptr;
    return;
  }

  GraphInfo & gi = (*graphs_info_)[0];

  auto rc = borrowed_interface_->interface_.graphRetrieve(
      context_, graph_name.c_str(), &(gi.graph));
  if (QNN_SUCCESS != rc) {
    QRB_ERROR("graphRetrieve failed for graph '", graph_name, "' rc=", rc);
    return;
  }
  graphs_count_ = 1;

  // Copy graph name.
  gi.graph_name = (char *)malloc(graph_name.size() + 1);
  if (gi.graph_name != nullptr) {
    memcpy(gi.graph_name, graph_name.c_str(), graph_name.size() + 1);
  }

  // Copy tensor metadata from the owner's GraphInfo so inference_execute can set up I/O buffers.
  if (owner_graph_info != nullptr) {
    QnnTensor tensor_ops;
    gi.num_of_input_tensors = owner_graph_info->num_of_input_tensors;
    if (gi.num_of_input_tensors > 0 && owner_graph_info->input_tensors != nullptr) {
      if (StatusCode::SUCCESS != tensor_ops.setup_tensors(
              gi.input_tensors, gi.num_of_input_tensors, owner_graph_info->input_tensors)) {
        QRB_ERROR("Failed to copy input tensor metadata for graph: ", graph_name);
        graphs_count_ = 0;
        return;
      }
    }
    gi.num_of_output_tensors = owner_graph_info->num_of_output_tensors;
    if (gi.num_of_output_tensors > 0 && owner_graph_info->output_tensors != nullptr) {
      if (StatusCode::SUCCESS != tensor_ops.setup_tensors(
              gi.output_tensors, gi.num_of_output_tensors, owner_graph_info->output_tensors)) {
        QRB_ERROR("Failed to copy output tensor metadata for graph: ", graph_name);
        graphs_count_ = 0;
        return;
      }
    }
    QRB_INFO("Shared-context graph '", graph_name, "': ",
        gi.num_of_input_tensors, " input(s), ", gi.num_of_output_tensors, " output(s)");
  } else {
    QRB_WARNING("No owner_graph_info provided for '", graph_name,
        "' — tensor metadata unavailable, inference may fail");
  }

  QRB_INFO("Shared-context graph '", graph_name, "' retrieved successfully");
}

QnnInference::~QnnInference()
{
  free_graphs_info();
  if (owned_context_) {
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
  } else {
    // Borrowed interface — do not delete, just null the pointer.
    borrowed_interface_ = nullptr;
    context_ = nullptr;
  }
}

StatusCode QnnInference::inference_init()
{
  // Shared-context instances have no backend/device to initialize.
  if (!owned_context_) {
    return StatusCode::SUCCESS;
  }

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
  // Shared-context instances already have their graph retrieved in the constructor.
  if (!owned_context_) {
    return graphs_count_ > 0 ? StatusCode::SUCCESS : StatusCode::FAILURE;
  }

  if (true == load_model_from_binary) {
    if (init_graph_from_binary() != StatusCode::SUCCESS) {
      return StatusCode::FAILURE;
    }

    const char * no_perf_env = std::getenv("QNN_HTP_NO_PERF");
    bool skip_perf = (no_perf_env != nullptr && std::string(no_perf_env) == "1");
    bool is_htp = (backend_option_.find("Htp") != std::string::npos);

    if (is_htp && !skip_perf) {
      if (init_performance() != StatusCode::SUCCESS) {
        QRB_ERROR("HTP performance enable failed!");
        return StatusCode::FAILURE;
      }
    } else if (skip_perf) {
      QRB_INFO("QNN_HTP_NO_PERF=1 is set, skipping HTP performance initialization");
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
  return inference_execute_ptr(input_tensor_data.data(), input_tensor_data.size());
}

StatusCode QnnInference::inference_execute_ptr(const void * src, size_t src_size)
{
  if (src == nullptr || src_size == 0) {
    QRB_ERROR("Input tensor pointer is NULL or size is 0!");
    return StatusCode::FAILURE;
  }

  for (uint32_t i = 0; i < graphs_count_; i++) {
    const auto & graphs_info = (*(graphs_info_))[i];

    auto io_tensors =
        QnnTensor(graphs_info.num_of_input_tensors, graphs_info.num_of_output_tensors);

    if (StatusCode::SUCCESS != io_tensors.setup_tensors(io_tensors.inputs_,
                                   io_tensors.num_of_input_tensors_, graphs_info.input_tensors)) {
      QRB_ERROR("Setup input tensors failed!");
      return StatusCode::FAILURE;
    }

    if (StatusCode::SUCCESS != io_tensors.setup_tensors(io_tensors.outputs_,
                                   io_tensors.num_of_output_tensors_, graphs_info.output_tensors)) {
      QRB_ERROR("Setup output tensors failed!");
      return StatusCode::FAILURE;
    }

    if (StatusCode::SUCCESS != io_tensors.write_input_tensors_ptr(src, src_size)) {
      QRB_ERROR("Write QNN input tensors failed!");
      return StatusCode::FAILURE;
    }

    auto graph_exec_rc = this->iface()->interface_.graphExecute(graphs_info.graph,
        io_tensors.inputs_, io_tensors.num_of_input_tensors_, io_tensors.outputs_,
        io_tensors.num_of_output_tensors_, nullptr, nullptr);
    if (QNN_GRAPH_NO_ERROR != graph_exec_rc) {
      QRB_ERROR("QNN graphExecute failed!");
      log_error_details(graph_exec_rc);
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

StatusCode QnnInference::inference_execute_ptr_dmabuf_out(const void * src, size_t src_size)
{
  // NOTE: QNN HTP requires both input and output to use the same memory path.
  // Mixing ClientBuffer input with MEMHANDLE output causes graphExecute to crash.
  // This method is kept for API completeness but falls back to regular execute.
  return inference_execute_ptr(src, src_size);
}

StatusCode QnnInference::inference_execute_dmabuf(int dmabuf_fd,
    uint32_t dmabuf_size,
    uint64_t dmabuf_offset)
{
  if (dmabuf_fd < 0 || dmabuf_size == 0) {
    QRB_ERROR("Invalid DMA-BUF input: fd=", dmabuf_fd, " size=", dmabuf_size);
    return StatusCode::FAILURE;
  }

  for (uint32_t g = 0; g < graphs_count_; g++) {
    const auto & graph_info = (*(graphs_info_))[g];

    auto io_tensors = QnnTensor(graph_info.num_of_input_tensors, graph_info.num_of_output_tensors);
    io_tensors.use_mem_handle_ = true;

    if (StatusCode::SUCCESS != io_tensors.setup_tensors(io_tensors.inputs_,
                                   io_tensors.num_of_input_tensors_, graph_info.input_tensors)) {
      QRB_ERROR("Setup input tensors failed!");
      return StatusCode::FAILURE;
    }

    if (StatusCode::SUCCESS != io_tensors.setup_tensors(io_tensors.outputs_,
                                   io_tensors.num_of_output_tensors_, graph_info.output_tensors)) {
      QRB_ERROR("Setup output tensors failed!");
      return StatusCode::FAILURE;
    }

    // INPUT: Register once, reuse across frames
    if (cached_input_fd_ != dmabuf_fd || cached_input_handle_ == nullptr) {
      // Input fd changed or first frame: (de)register
      if (cached_input_handle_ != nullptr) {
        iface()->interface_.memDeRegister(&cached_input_handle_, 1u);
        cached_input_handle_ = nullptr;
      }

      if (graph_info.num_of_input_tensors == 1) {
        Qnn_MemDescriptor_t input_mem_desc = QNN_MEM_DESCRIPTOR_INIT;
        input_mem_desc.memShape = { io_tensors.inputs_[0].v1.rank,
          io_tensors.inputs_[0].v1.dimensions, nullptr };
        input_mem_desc.dataType = io_tensors.inputs_[0].v1.dataType;
        input_mem_desc.memType = QNN_MEM_TYPE_ION;
        input_mem_desc.ionInfo.fd = dmabuf_fd;

        auto rc = iface()->interface_.memRegister(
            context_, &input_mem_desc, 1u, &cached_input_handle_);
        if (QNN_SUCCESS != rc) {
          const char * err_msg = nullptr;
          iface()->interface_.errorGetMessage(rc, &err_msg);
          QRB_ERROR("memRegister(input) failed: ", (err_msg ? err_msg : "unknown"));
          cached_input_handle_ = nullptr;
          return StatusCode::FAILURE;
        }
      } else {
        // Multi-input: use offset-based registration (first input only cached)
        Qnn_MemDescriptor_t input_mem_desc = QNN_MEM_DESCRIPTOR_INIT;
        input_mem_desc.memShape = { io_tensors.inputs_[0].v1.rank,
          io_tensors.inputs_[0].v1.dimensions, nullptr };
        input_mem_desc.dataType = io_tensors.inputs_[0].v1.dataType;
        input_mem_desc.memType = QNN_MEM_TYPE_CUSTOM;

        QnnMemHtp_Descriptor_t htp_mem_desc;
        htp_mem_desc.type = QNN_HTP_MEM_SHARED_BUFFER;
        htp_mem_desc.size = dmabuf_size;
        QnnHtpMem_SharedBufferConfig_t htp_shared_buf_config = { dmabuf_fd, 0 };
        htp_mem_desc.sharedBufferConfig = htp_shared_buf_config;
        input_mem_desc.customInfo = &htp_mem_desc;

        auto rc = iface()->interface_.memRegister(
            context_, &input_mem_desc, 1u, &cached_input_handle_);
        if (QNN_SUCCESS != rc) {
          const char * err_msg = nullptr;
          iface()->interface_.errorGetMessage(rc, &err_msg);
          QRB_ERROR("memRegister(input) failed: ", (err_msg ? err_msg : "unknown"));
          cached_input_handle_ = nullptr;
          return StatusCode::FAILURE;
        }
      }

      cached_input_fd_ = dmabuf_fd;
    }

    // Set cached input handle on tensor
    io_tensors.inputs_[0].v1.memType = QNN_TENSORMEMTYPE_MEMHANDLE;
    io_tensors.inputs_[0].v1.memHandle = cached_input_handle_;

    // OUTPUT: Allocate once, reuse across frames
    if (!dmabuf_cache_initialized_) {
      constexpr int RPCMEM_HEAP_ID_SYSTEM = 25;
      constexpr uint32_t RPCMEM_DEFAULT_FLAGS = 1;

      cached_output_buffers_.resize(graph_info.num_of_output_tensors);
      cached_output_handles_.resize(graph_info.num_of_output_tensors, nullptr);
      cached_output_fds_.resize(graph_info.num_of_output_tensors, -1);
      cached_output_sizes_.resize(graph_info.num_of_output_tensors, 0);
      cached_output_ptrs_.resize(graph_info.num_of_output_tensors, nullptr);

      for (uint32_t out_i = 0; out_i < graph_info.num_of_output_tensors; out_i++) {
        auto * out_tensor = &(io_tensors.outputs_[out_i]);
        auto shape = io_tensors.get_tensor_shape(out_tensor);
        uint32_t output_tensor_size = io_tensors.get_tensor_size(out_tensor, shape);

        auto rpc_mgr = std::make_shared<RpcMemManager>();
        if (StatusCode::SUCCESS != rpc_mgr->init()) {
          QRB_ERROR("RpcMemManager init failed");
          return StatusCode::FAILURE;
        }

        if (StatusCode::SUCCESS !=
            rpc_mgr->alloc(output_tensor_size, RPCMEM_HEAP_ID_SYSTEM, RPCMEM_DEFAULT_FLAGS)) {
          QRB_ERROR("RpcMemManager alloc failed for size: ", output_tensor_size);
          return StatusCode::FAILURE;
        }

        int fd = rpc_mgr->get_fd();
        void * ptr = rpc_mgr->get_ptr();

        Qnn_MemDescriptor_t out_mem_desc = QNN_MEM_DESCRIPTOR_INIT;
        out_mem_desc.memShape = { out_tensor->v1.rank, out_tensor->v1.dimensions, nullptr };
        out_mem_desc.dataType = out_tensor->v1.dataType;
        out_mem_desc.memType = QNN_MEM_TYPE_ION;
        out_mem_desc.ionInfo.fd = fd;

        Qnn_MemHandle_t out_mem_handle = nullptr;
        auto out_rc =
            iface()->interface_.memRegister(context_, &out_mem_desc, 1u, &out_mem_handle);
        if (QNN_SUCCESS != out_rc) {
          const char * err_msg = nullptr;
          iface()->interface_.errorGetMessage(out_rc, &err_msg);
          QRB_ERROR("memRegister(output) failed: ", (err_msg ? err_msg : "unknown"));
          return StatusCode::FAILURE;
        }

        // Keep rpc_mgr alive (owns the buffer) but don't disown - we manage lifecycle
        cached_output_buffers_[out_i] = rpc_mgr;
        cached_output_handles_[out_i] = out_mem_handle;
        cached_output_fds_[out_i] = fd;
        cached_output_sizes_[out_i] = output_tensor_size;
        cached_output_ptrs_[out_i] = ptr;
      }

      dmabuf_cache_initialized_ = true;
    }

    // Set cached output handles on tensors
    for (uint32_t out_i = 0; out_i < graph_info.num_of_output_tensors; out_i++) {
      io_tensors.outputs_[out_i].v1.memType = QNN_TENSORMEMTYPE_MEMHANDLE;
      io_tensors.outputs_[out_i].v1.memHandle = cached_output_handles_[out_i];
    }

    // EXECUTE: No register/deregister overhead
    auto exec_rc = iface()->interface_.graphExecute(graph_info.graph, io_tensors.inputs_,
        io_tensors.num_of_input_tensors_, io_tensors.outputs_, io_tensors.num_of_output_tensors_,
        nullptr, nullptr);

    if (QNN_GRAPH_NO_ERROR != exec_rc) {
      QRB_ERROR("QNN graphExecute failed with code: ", exec_rc);
      return StatusCode::FAILURE;
    }

#ifndef __hexagon__
    output_tensor_.clear();
    output_tensor_.reserve(graph_info.num_of_output_tensors);

    for (uint32_t out_i = 0; out_i < graph_info.num_of_output_tensors; out_i++) {
      const auto * out_tensor = &(io_tensors.outputs_[out_i]);
      auto shape = io_tensors.get_tensor_shape(out_tensor);

      OutputTensor ot;
      ot.output_tensor_name = out_tensor->v1.name;
      ot.output_tensor_shape.reserve(shape.size());
      for (size_t i = 0; i < shape.size(); i++) {
        ot.output_tensor_shape.push_back(static_cast<uint32_t>(shape[i]));
      }
      ot.data_type = io_tensors.qnn_dtype_to_qrb_dtype(out_tensor->v1.dataType);

      // Output is in cached DMA buffer (reused across frames)
      ot.output_dmabuf_fd = cached_output_fds_[out_i];
      ot.output_dmabuf_offset = dmabuf_offset;
      ot.output_dmabuf_size = cached_output_sizes_[out_i];
      // ptr=0 signals downstream should NOT free this buffer (it's reused)
      ot.output_dmabuf_ptr = 0;

      output_tensor_.emplace_back(std::move(ot));
    }
#endif
  }

  return StatusCode::SUCCESS;
}

const std::vector<OutputTensor> QnnInference::get_output_tensors()
{
  return std::move(output_tensor_);
}

StatusCode QnnInference::initialize_backend()
{
  // Enable detailed error reporting for better diagnostics
  QnnBackend_Config_t error_config = QNN_BACKEND_CONFIG_INIT;
  error_config.option = QNN_BACKEND_CONFIG_OPTION_ERROR_REPORTING;
  error_config.errorConfig.reportingLevel = QNN_ERROR_REPORTING_LEVEL_DETAILED;
  error_config.errorConfig.storageLimit = 1024;  // 1 MB for error info

  const QnnBackend_Config_t * backend_configs[] = { &error_config, nullptr };

  auto qnn_status = qnn_interface_->interface_.backendCreate(
      nullptr, (const QnnBackend_Config_t **)backend_configs, &(backend_handle_));

  if (QNN_BACKEND_NO_ERROR != qnn_status) {
    // Retry without error reporting config in case backend doesn't support it
    qnn_status = qnn_interface_->interface_.backendCreate(
        nullptr, (const QnnBackend_Config_t **)(nullptr), &(backend_handle_));
    if (QNN_BACKEND_NO_ERROR != qnn_status) {
      QRB_ERROR("Could not initialize backend due to error = ", qnn_status);
      return StatusCode::FAILURE;
    }
  }

  QRB_INFO(backend_option_, " initialize successfully");
  return StatusCode::SUCCESS;
}

StatusCode QnnInference::create_device()
{
  auto is_device_property_supported = [this] {
    if (nullptr != qnn_interface_->interface_.propertyHasCapability) {
      auto qnn_status = qnn_interface_->interface_.propertyHasCapability(QNN_PROPERTY_GROUP_DEVICE);

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
    if (nullptr != qnn_interface_->interface_.deviceCreate) {
      auto qnn_status =
          qnn_interface_->interface_.deviceCreate(nullptr, nullptr, &(device_handle_));

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
  if (QNN_CONTEXT_NO_ERROR != qnn_interface_->interface_.contextCreate(
                                  backend_handle_, device_handle_, nullptr, &(context_))) {
    QRB_ERROR("Could not create context!");
    return StatusCode::FAILURE;
  }
  return StatusCode::SUCCESS;
}

StatusCode QnnInference::compose_graphs()
{
  if (ModelError::MODEL_NO_ERROR !=
      qnn_interface_->compose_graphs_(backend_handle_, qnn_interface_->interface_, context_,
          nullptr, 0, &(graphs_info_), &(graphs_count_), false, nullptr, QNN_LOG_LEVEL_MAX)) {
    QRB_ERROR("Failed in composeGraphs()!");
    return StatusCode::FAILURE;
  }
  return StatusCode::SUCCESS;
}

StatusCode QnnInference::finalize_graphs()
{
  for (size_t i = 0; i < graphs_count_; i++) {
    if (QNN_GRAPH_NO_ERROR !=
        qnn_interface_->interface_.graphFinalize((*graphs_info_)[i].graph, nullptr, nullptr)) {
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

  auto check_and_free = [](auto & ptr) {
    if (ptr != nullptr) {
      free((void *)ptr);
      ptr = nullptr;
    }
  };

  if (*graphs_info_ != nullptr) {
    for (uint32_t i = 0; i < graphs_count_; i++) {
      GraphInfo & graph_info = (*graphs_info_)[i];
      check_and_free(graph_info.graph_name);

      QnnTensor tensor_ops;
      tensor_ops.free_qnn_tensors(graph_info.input_tensors, graph_info.num_of_input_tensors);
      tensor_ops.free_qnn_tensors(graph_info.output_tensors, graph_info.num_of_output_tensors);
    }
    check_and_free(*graphs_info_);
  }

  check_and_free(graphs_info_);
}

void QnnInference::free_context()
{
  if (!owned_context_) {
    return;
  }
  if (QNN_CONTEXT_NO_ERROR != qnn_interface_->interface_.contextFree(context_, nullptr)) {
    QRB_ERROR("Failed to free context!");
  }

  context_ = nullptr;
}

void QnnInference::free_device()
{
  if (!owned_context_) {
    return;
  }
  if (perf_initialized_ && perf_infra_.destroyPowerConfigId != nullptr) {
    perf_infra_.destroyPowerConfigId(power_config_id_);
    perf_initialized_ = false;
  }

  if (true == support_device_) {
    if (nullptr != qnn_interface_->interface_.deviceFree) {
      auto qnn_status = qnn_interface_->interface_.deviceFree(device_handle_);
      if (QNN_SUCCESS != qnn_status && QNN_DEVICE_ERROR_UNSUPPORTED_FEATURE != qnn_status) {
        QRB_ERROR("Failed to free device!");
      }
    }
  }

  device_handle_ = nullptr;
}

void QnnInference::free_backend()
{
  if (!owned_context_) {
    return;
  }
  if (qnn_interface_->interface_.backendFree != nullptr) {
    if (QNN_BACKEND_NO_ERROR != qnn_interface_->interface_.backendFree(backend_handle_)) {
      QRB_ERROR("Could not free backend!");
    }
  }

  backend_handle_ = nullptr;
}

StatusCode QnnInference::init_performance()
{
  if (nullptr == qnn_interface_->interface_.deviceGetInfrastructure) {
    QRB_WARNING("deviceGetInfrastructure is not available, skipping performance init");
    return StatusCode::SUCCESS;
  }

  QnnDevice_Infrastructure_t device_infra = nullptr;
  if (QNN_SUCCESS != qnn_interface_->interface_.deviceGetInfrastructure(&device_infra)) {
    QRB_ERROR("Failure in deviceGetInfrastructure()");
    return StatusCode::FAILURE;
  }

  auto * htp_infra = static_cast<QnnHtpDevice_Infrastructure_t *>(device_infra);
  perf_infra_ = htp_infra->perfInfra;

  uint32_t device_id = 0;
  uint32_t core_id = 0;
  if (QNN_SUCCESS != perf_infra_.createPowerConfigId(device_id, core_id, &power_config_id_)) {
    QRB_ERROR("Failure in createPowerConfigId()");
    return StatusCode::FAILURE;
  }

  perf_initialized_ = true;

  // Determine HTP performance mode from environment variables.
  // Priority order: BURST > SUSTAINED_HIGH > BALANCED > POWER_SAVER > default
  auto get_perf_mode = []() -> std::pair<QnnHtpPerfInfrastructure_PowerMode_t, const char *> {
    auto env_is_set = [](const char * name) -> bool {
      const char * v = std::getenv(name);
      return (v != nullptr && std::string(v) == "1");
    };

    if (env_is_set("QNN_HTP_BURST")) {
      return { QNN_HTP_PERF_INFRASTRUCTURE_POWERMODE_PERFORMANCE_MODE, "burst" };
    }
    if (env_is_set("QNN_HTP_SUSTAINED_HIGH")) {
      return { QNN_HTP_PERF_INFRASTRUCTURE_POWERMODE_ADJUST_ONLY_UP, "sustained_high" };
    }
    if (env_is_set("QNN_HTP_BALANCED")) {
      return { QNN_HTP_PERF_INFRASTRUCTURE_POWERMODE_ADJUST_UP_DOWN, "balanced" };
    }
    if (env_is_set("QNN_HTP_POWER_SAVER")) {
      return { QNN_HTP_PERF_INFRASTRUCTURE_POWERMODE_POWER_SAVER_MODE, "power_saver" };
    }
    return { QNN_HTP_PERF_INFRASTRUCTURE_POWERMODE_UNKNOWN, nullptr };
  };

  auto [power_mode, mode_name] = get_perf_mode();

  if (mode_name != nullptr) {
    QnnHtpPerfInfrastructure_PowerConfig_t power_config =
        QNN_HTP_PERF_INFRASTRUCTURE_POWER_CONFIG_INIT;
    power_config.option = QNN_HTP_PERF_INFRASTRUCTURE_POWER_CONFIGOPTION_DCVS_V3;
    power_config.dcvsV3Config.contextId = power_config_id_;
    power_config.dcvsV3Config.setDcvsEnable = 1;
    power_config.dcvsV3Config.dcvsEnable = 0;  // disable DCVS to lock performance level
    power_config.dcvsV3Config.powerMode = power_mode;
    power_config.dcvsV3Config.setSleepLatency = 1;
    power_config.dcvsV3Config.sleepLatency = 40;  // low latency
    power_config.dcvsV3Config.setSleepDisable = 1;
    power_config.dcvsV3Config.sleepDisable = 1;  // disable sleep for consistent performance
    power_config.dcvsV3Config.setBusParams = 1;
    power_config.dcvsV3Config.busVoltageCornerMin = DCVS_VOLTAGE_VCORNER_TURBO;
    power_config.dcvsV3Config.busVoltageCornerTarget = DCVS_VOLTAGE_VCORNER_TURBO;
    power_config.dcvsV3Config.busVoltageCornerMax = DCVS_VOLTAGE_VCORNER_TURBO_PLUS;
    power_config.dcvsV3Config.setCoreParams = 1;
    power_config.dcvsV3Config.coreVoltageCornerMin = DCVS_VOLTAGE_VCORNER_TURBO;
    power_config.dcvsV3Config.coreVoltageCornerTarget = DCVS_VOLTAGE_VCORNER_TURBO;
    power_config.dcvsV3Config.coreVoltageCornerMax = DCVS_VOLTAGE_VCORNER_TURBO_PLUS;

    const QnnHtpPerfInfrastructure_PowerConfig_t * power_configs[] = { &power_config, nullptr };

    if (QNN_SUCCESS != perf_infra_.setPowerConfig(power_config_id_, power_configs)) {
      QRB_WARNING("setPowerConfig failed for mode: ", mode_name,
          ", continuing with default performance mode");
    } else {
      QRB_INFO("HTP performance mode set to: ", mode_name);
    }
  } else {
    QRB_INFO("HTP performance mode: default (no override set)");
  }

  return StatusCode::SUCCESS;
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
      qnn_interface_->qnn_system_interface_.systemContextCreate(&sys_context_headle)) {
    QRB_ERROR("Could not create system handle.");
    return StatusCode::FAILURE;
  }

  const QnnSystemContext_BinaryInfo_t * binary_info = nullptr;
  Qnn_ContextBinarySize_t binary_info_size = 0;
  if (QNN_SUCCESS !=
      qnn_interface_->qnn_system_interface_.systemContextGetBinaryInfo(sys_context_headle,
          static_cast<void *>(model_buf.get()), model_buf_size, &binary_info, &binary_info_size)) {
    QRB_ERROR("Failed to get context binary info.");
    return StatusCode::FAILURE;
  }

  if (StatusCode::SUCCESS != set_up_graph_info(binary_info)) {
    return StatusCode::FAILURE;
  }

  qnn_interface_->qnn_system_interface_.systemContextFree(sys_context_headle);
  sys_context_headle = nullptr;

  return StatusCode::SUCCESS;
}

// Helper: allocate and deep-copy tensor metadata from a QNN tensor array.
static StatusCode set_up_tensors_info(
    const Qnn_Tensor_t * src, const uint32_t cnt, Qnn_Tensor_t *& dst)
{
  dst = (Qnn_Tensor_t *)calloc(cnt, sizeof(Qnn_Tensor_t));
  if (nullptr == dst) {
    return StatusCode::FAILURE;
  }
  QnnTensor tensor_ops;
  for (size_t j = 0; j < cnt; j++) {
    dst[j] = QNN_TENSOR_INIT;
    if (StatusCode::SUCCESS != tensor_ops.tensor_info_deep_copy(&dst[j], &src[j])) {
      return StatusCode::FAILURE;
    }
  }
  return StatusCode::SUCCESS;
}

// GraphInfoMember is a pointer-to-member selecting graphInfoV1/V2/V3 out of
// QnnSystemContext_GraphInfo_t, so the same loop body works for every binary-info version.
template <auto GraphInfoMember>
StatusCode QnnInference::copy_graph_info(const QnnSystemContext_GraphInfo_t * graphs_array)
{
  // graphs_info_ is GraphInfo** where *graphs_info_ points to a contiguous GraphInfo array.
  // Allocate the outer pointer and the inner contiguous array.
  graphs_info_ = (GraphInfo **)calloc(1, sizeof(GraphInfo *));
  if (nullptr == graphs_info_) {
    return StatusCode::FAILURE;
  }
  *graphs_info_ = (GraphInfo *)calloc(graphs_count_, sizeof(GraphInfo));
  if (nullptr == *graphs_info_) {
    return StatusCode::FAILURE;
  }
  for (uint32_t i = 0; i < graphs_count_; i++) {
    GraphInfo & dst = (*graphs_info_)[i];
    const auto & gi_src = graphs_array[i].*GraphInfoMember;
    dst.graph_name = (char *)malloc(strlen(gi_src.graphName) + 1);
    if (nullptr == dst.graph_name) {
      return StatusCode::FAILURE;
    }
    memcpy(dst.graph_name, gi_src.graphName, strlen(gi_src.graphName) + 1);
    dst.num_of_input_tensors = gi_src.numGraphInputs;
    if (StatusCode::SUCCESS !=
        set_up_tensors_info(gi_src.graphInputs, gi_src.numGraphInputs, dst.input_tensors)) {
      return StatusCode::FAILURE;
    }
    dst.num_of_output_tensors = gi_src.numGraphOutputs;
    if (StatusCode::SUCCESS !=
        set_up_tensors_info(gi_src.graphOutputs, gi_src.numGraphOutputs, dst.output_tensors)) {
      return StatusCode::FAILURE;
    }
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

  // Log context binary metadata for diagnostics
  switch (binary_info->version) {
    case QNN_SYSTEM_CONTEXT_BINARY_INFO_VERSION_1: {
      const auto & info = binary_info->contextBinaryInfoV1;
      QRB_INFO("Context binary info (V1): SDK build=", (info.buildId ? info.buildId : "unknown"),
          ", target SoC=", (info.socVersion ? info.socVersion : "unknown"));
      graphs_count_ = info.numGraphs;
      // Pass the full QnnSystemContext_GraphInfo_t array so copy_graph_info
      // can stride correctly through all graphs.
      if (StatusCode::SUCCESS !=
          copy_graph_info<&QnnSystemContext_GraphInfo_t::graphInfoV1>(info.graphs)) {
        return StatusCode::FAILURE;
      }
      break;
    }
    case QNN_SYSTEM_CONTEXT_BINARY_INFO_VERSION_2: {
      const auto & info = binary_info->contextBinaryInfoV2;
      QRB_INFO("Context binary info (V2): SDK build=", (info.buildId ? info.buildId : "unknown"),
          ", target SoC=", (info.socVersion ? info.socVersion : "unknown"));
      graphs_count_ = info.numGraphs;
      if (StatusCode::SUCCESS !=
          copy_graph_info<&QnnSystemContext_GraphInfo_t::graphInfoV2>(info.graphs)) {
        return StatusCode::FAILURE;
      }
      break;
    }
    case QNN_SYSTEM_CONTEXT_BINARY_INFO_VERSION_3: {
      const auto & info = binary_info->contextBinaryInfoV3;
      QRB_INFO(
          "Context binary info (V3/FCB): SDK build=", (info.buildId ? info.buildId : "unknown"),
          ", target SoC=", (info.socVersion ? info.socVersion : "unknown"),
          ", SoC model ID=", info.socModel);
      if (info.socModel == 0 && info.socVersion == nullptr) {
        QRB_INFO(
            "Model appears to be a Flexible Context Binary (FCB) — "
            "compatible with multiple SoC targets");
      } else {
        QRB_INFO("Model is a standard context binary targeting a specific SoC");
      }
      graphs_count_ = info.numGraphs;
      if (StatusCode::SUCCESS !=
          copy_graph_info<&QnnSystemContext_GraphInfo_t::graphInfoV3>(info.graphs)) {
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
  auto create_rc =
      qnn_interface_->interface_.contextCreateFromBinary(backend_handle_, device_handle_, nullptr,
          static_cast<void *>(model_buf.get()), model_buf_size, &context_, nullptr);
  if (QNN_SUCCESS != create_rc) {
    QRB_ERROR("Could not create context from binary!");
    log_error_details(create_rc);
    return StatusCode::FAILURE;
  }

  for (uint32_t i = 0; i < graphs_count_; i++) {
    if (QNN_SUCCESS != qnn_interface_->interface_.graphRetrieve(
                           context_, (*graphs_info_)[i].graph_name, &((*graphs_info_)[i].graph))) {
      QRB_ERROR("Unable to retrieve graph handle for the graph");
      return StatusCode::FAILURE;
    }
  }
  return StatusCode::SUCCESS;
}

void QnnInference::log_error_details(Qnn_ErrorHandle_t error_handle)
{
  if (error_handle == QNN_SUCCESS) {
    return;
  }

  // Try verbose message first
  if (iface()->interface_.errorGetVerboseMessage != nullptr) {
    const char * verbose_msg = nullptr;
    if (QNN_SUCCESS ==
        iface()->interface_.errorGetVerboseMessage(error_handle, &verbose_msg)) {
      if (verbose_msg != nullptr) {
        QRB_ERROR("Detailed error info: ", verbose_msg);
        if (iface()->interface_.errorFreeVerboseMessage != nullptr) {
          iface()->interface_.errorFreeVerboseMessage(verbose_msg);
        }
        return;
      }
    }
  }

  // Fall back to basic error message
  if (iface()->interface_.errorGetMessage != nullptr) {
    const char * err_msg = nullptr;
    if (QNN_SUCCESS == iface()->interface_.errorGetMessage(error_handle, &err_msg)) {
      if (err_msg != nullptr) {
        QRB_ERROR("Error info: ", err_msg);
      }
    }
  }
}

}  // namespace qrb::inference_mgr
