// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_INFERENCE_MANAGER_QNN_INFERENCE_HPP_
#define QRB_INFERENCE_MANAGER_QNN_INFERENCE_HPP_

#include <memory>

#include "qnn_inference/qnn_inference_impl.hpp"
#include "qrb_inference.hpp"

namespace qrb::inference_mgr
{

class RpcMemManager
{
public:
  RpcMemManager() = default;

  // If true, destructor will NOT free(ptr). This is used when ownership is transferred
  // to downstream (e.g. post-process node) which will call rpcmem_free(ptr) itself.
  void disown() { owned_ = false; }

  ~RpcMemManager()
  {
    if (owned_ && ptr_ != nullptr && rpcmem_free_ != nullptr) {
      rpcmem_free_(ptr_);
      ptr_ = nullptr;
    }
    if (libCdspHandle_ != nullptr) {
      ::dlclose(libCdspHandle_);
      libCdspHandle_ = nullptr;
    }
  }

  // Delete copy constructor and assignment
  RpcMemManager(const RpcMemManager &) = delete;
  RpcMemManager & operator=(const RpcMemManager &) = delete;

  StatusCode init()
  {
    libCdspHandle_ = ::dlopen("libcdsprpc.so", RTLD_NOW | RTLD_LOCAL);
    if (nullptr == libCdspHandle_) {
      QRB_ERROR("dlopen(libcdsprpc.so) failed");
      return StatusCode::FAILURE;
    }

    rpcmem_alloc_ = (RpcMemAllocFn_t)::dlsym(libCdspHandle_, "rpcmem_alloc");
    rpcmem_to_fd_ = (RpcMemToFdFn_t)::dlsym(libCdspHandle_, "rpcmem_to_fd");
    rpcmem_free_ = (RpcMemFreeFn_t)::dlsym(libCdspHandle_, "rpcmem_free");

    if (nullptr == rpcmem_alloc_ || nullptr == rpcmem_to_fd_ || nullptr == rpcmem_free_) {
      QRB_ERROR("Failed to resolve rpcmem symbols");
      ::dlclose(libCdspHandle_);
      libCdspHandle_ = nullptr;
      return StatusCode::FAILURE;
    }

    return StatusCode::SUCCESS;
  }

  StatusCode alloc(size_t size, int heap_id = 25, uint32_t flags = 1)
  {
    if (rpcmem_alloc_ == nullptr) {
      QRB_ERROR("RpcMemManager not initialized");
      return StatusCode::FAILURE;
    }

    ptr_ = rpcmem_alloc_(heap_id, flags, static_cast<int>(size));
    if (nullptr == ptr_) {
      QRB_ERROR("rpcmem_alloc failed for size: ", size);
      return StatusCode::FAILURE;
    }

    fd_ = rpcmem_to_fd_(ptr_);
    if (fd_ < 0) {
      QRB_ERROR("rpcmem_to_fd failed");
      rpcmem_free_(ptr_);
      ptr_ = nullptr;
      return StatusCode::FAILURE;
    }

    size_ = size;
    return StatusCode::SUCCESS;
  }

  StatusCode free_mem_ptr(void * ptr)
  {
    if (ptr != nullptr && rpcmem_free_ != nullptr) {
      rpcmem_free_(ptr);
      ptr = nullptr;
    } else
      return StatusCode::FAILURE;
    return StatusCode::SUCCESS;
  }

  void * get_ptr() const { return ptr_; }
  int get_fd() const { return fd_; }
  size_t get_size() const { return size_; }

private:
  using RpcMemAllocFn_t = void * (*)(int, uint32_t, int);
  using RpcMemToFdFn_t = int (*)(void *);
  using RpcMemFreeFn_t = void (*)(void *);

  void * libCdspHandle_ = nullptr;
  RpcMemAllocFn_t rpcmem_alloc_ = nullptr;
  RpcMemToFdFn_t rpcmem_to_fd_ = nullptr;
  RpcMemFreeFn_t rpcmem_free_ = nullptr;

  bool owned_ = true;
  void * ptr_ = nullptr;
  int fd_ = -1;
  size_t size_ = 0;
};

class QnnInference : public QrbInference
{
public:
  // Standard constructor: loads model_path, creates its own backend/device/context.
  QnnInference(const std::string & model_path, const std::string & backend_option);

  // Shared-context constructor: borrows an already-created context and interface.
  // Calls only graphRetrieve for graph_name; does NOT own the context lifetime.
  // owner_graph_info: tensor metadata from the owner's GraphInfo for this graph (may be nullptr).
  QnnInference(Qnn_ContextHandle_t shared_context,
      QnnInterface * shared_interface,
      const std::string & graph_name,
      const GraphInfo * owner_graph_info = nullptr);

  ~QnnInference();
  StatusCode inference_init() override;
  StatusCode inference_graph_init() override;
  StatusCode inference_execute(const std::vector<uint8_t> & input_tensor_data) override;
  // Zero-copy variant: passes src pointer directly to QNN without memcpy.
  StatusCode inference_execute_ptr(const void * src, size_t src_size);
  // Hybrid: regular input copy + rpcmem output buffers (zero-copy output).
  StatusCode inference_execute_ptr_dmabuf_out(const void * src, size_t src_size);
  StatusCode inference_execute_dmabuf(int dmabuf_fd, uint32_t dmabuf_size, uint64_t dmabuf_offset);
  const std::vector<OutputTensor> get_output_tensors() override;

  // Accessors used by QrbSharedContextLoader to share context/interface with graph instances.
  Qnn_ContextHandle_t get_context() const { return context_; }
  QnnInterface * get_interface() const { return qnn_interface_.get(); }

  // Returns the GraphInfo for the named graph, or nullptr if not found.
  // Used by QrbSharedContextLoader to copy tensor metadata into shared-context instances.
  const GraphInfo * get_graph_info_for(const std::string & graph_name) const
  {
    if (graphs_info_ == nullptr) return nullptr;
    for (uint32_t i = 0; i < graphs_count_; ++i) {
      const GraphInfo & gi = (*graphs_info_)[i];
      if (gi.graph_name != nullptr && graph_name == gi.graph_name) {
        return &gi;
      }
    }
    return nullptr;
  }

private:
  const std::string model_path_;
  const std::string backend_option_;
  const std::string qnn_syslib_path_ = "libQnnSystem.so";
  bool load_model_from_binary = false;
  // When false this instance borrowed its context from QrbSharedContextLoader
  // and must not free the context, device, or backend on destruction.
  bool owned_context_ = true;
  void * backend_lib_handle = nullptr;
  void * sys_lib_handle_ = nullptr;
  void * backend_handle_ = nullptr;
  void * model_handle_ = nullptr;
  Qnn_DeviceHandle_t device_handle_ = nullptr;
  Qnn_ContextHandle_t context_ = nullptr;
  GraphInfo ** graphs_info_ = nullptr;
  uint32_t graphs_count_ = 0;
  bool support_device_ = false;
  QnnHtpDevice_PerfInfrastructure_t perf_infra_{};
  uint32_t power_config_id_ = 0;
  bool perf_initialized_ = false;
  std::vector<OutputTensor> output_tensor_;
  std::unique_ptr<QnnInterface> qnn_interface_{ nullptr };
  // When owned_context_ is false, this holds the borrowed interface pointer (not owned).
  QnnInterface * borrowed_interface_{ nullptr };

  // Returns the active interface regardless of ownership mode.
  QnnInterface * iface() const
  {
    return owned_context_ ? qnn_interface_.get() : borrowed_interface_;
  }

  StatusCode initialize_backend();
  StatusCode create_device();
  StatusCode create_context();
  StatusCode compose_graphs();
  StatusCode finalize_graphs();
  void free_graphs_info();
  void free_context();
  void free_device();
  void free_backend();
  StatusCode init_performance();
  void log_error_details(Qnn_ErrorHandle_t error_handle);

private:
  StatusCode init_graph_from_binary();
  std::tuple<std::shared_ptr<uint8_t[]>, uint64_t> read_binary_model();
  StatusCode get_and_set_graph_info_from_binary(const std::shared_ptr<uint8_t[]> model_buf,
      const uint64_t model_buf_size);
  // GraphInfoMember selects which versioned member (graphInfoV1/V2/V3) of
  // QnnSystemContext_GraphInfo_t to read from for a given binary-info version.
  template <auto GraphInfoMember>
  StatusCode copy_graph_info(const QnnSystemContext_GraphInfo_t * graphs_array);
  StatusCode set_up_graph_info(const QnnSystemContext_BinaryInfo_t * binary_info);
  StatusCode create_context_from_binary(const std::shared_ptr<uint8_t[]> model_buf,
      const uint64_t model_buf_size);

  // DMA buffer pool: cached handles for zero-overhead frame-to-frame reuse (dmabuf path)
  int cached_input_fd_{ -1 };
  Qnn_MemHandle_t cached_input_handle_{ nullptr };
  std::vector<std::shared_ptr<RpcMemManager>> cached_output_buffers_;
  std::vector<Qnn_MemHandle_t> cached_output_handles_;
  std::vector<int> cached_output_fds_;
  std::vector<uint32_t> cached_output_sizes_;
  std::vector<void *> cached_output_ptrs_;
  bool dmabuf_cache_initialized_{ false };
};  // class QnnInference

}  // namespace qrb::inference_mgr

#endif
