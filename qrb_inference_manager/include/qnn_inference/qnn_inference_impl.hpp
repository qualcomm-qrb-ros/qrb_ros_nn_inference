// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_INFERENCE_MANAGER_QNN_INFERENCE_IMPL_HPP_
#define QRB_INFERENCE_MANAGER_QNN_INFERENCE_IMPL_HPP_

#include <dlfcn.h>
#include <string.h>

#include <cstdint>
#include <fstream>
#include <functional>
#include <iostream>
#include <numeric>
#include <vector>

#include "QnnInterface.h"
#include "System/QnnSystemInterface.h"
#include "qrb_inference.hpp"

namespace qrb::inference_mgr
{

enum class ModelError
{
  MODEL_NO_ERROR = 0,
  MODEL_TENSOR_ERROR = 1,
  MODEL_PARAMS_ERROR = 2,
  MODEL_NODES_ERROR = 3,
  MODEL_GRAPH_ERROR = 4,
  MODEL_CONTEXT_ERROR = 5,
  MODEL_GENERATION_ERROR = 6,
  MODEL_SETUP_ERROR = 7,
  MODEL_INVALID_ARGUMENT_ERROR = 8,
  MODEL_FILE_ERROR = 9,
  MODEL_MEMORY_ALLOCATE_ERROR = 10,
  // Value selected to ensure 32 bits.
  MODEL_UNKNOWN_ERROR = 0x7FFFFFFF
};

struct GraphInfo
{
  Qnn_GraphHandle_t graph = nullptr;
  char * graph_name = nullptr;
  Qnn_Tensor_t * input_tensors = nullptr;
  uint32_t num_of_input_tensors = 0;
  Qnn_Tensor_t * output_tensors = nullptr;
  uint32_t num_of_output_tensors = 0;
};

struct GraphConfigInfo
{
  char * graph_name = nullptr;
  const QnnGraph_Config_t ** graph_configs = nullptr;
};

/// for getting QNN interface from model and backend
class QnnInterface
{
private:
  using compose_graphs_func = ModelError (*)(Qnn_BackendHandle_t,
      QNN_INTERFACE_VER_TYPE,
      Qnn_ContextHandle_t,
      GraphConfigInfo **,
      const uint32_t,
      GraphInfo ***,
      uint32_t *,
      bool,
      QnnLog_Callback_t,
      QnnLog_Level_t);

  using free_graph_info_func = ModelError (*)(GraphInfo ***, uint32_t);

  using qnn_interface_providers_func = Qnn_ErrorHandle_t (*)(const QnnInterface_t ***, uint32_t *);

  using qnn_sys_interface_providers_func = Qnn_ErrorHandle_t (*)(const QnnSystemInterface_t ***,
      uint32_t *);

public:
  QnnInterface(const std::string & model_path,
      const std::string & backend_option,
      void ** backend_handle,
      void ** model_handle);

  QnnInterface(const std::string & backend_option,
      void ** backend_handle,
      const std::string & qnn_syslib_path,
      void ** sys_lib_handle);

  ~QnnInterface() = default;

  compose_graphs_func compose_graphs;
  free_graph_info_func free_graph_info;
  QNN_INTERFACE_VER_TYPE interface;
  QNN_SYSTEM_INTERFACE_VER_TYPE qnn_system_interface;

private:
  template <class T>
  inline T get_function_from_lib(const std::string & lib_name,
      const int open_flag,
      void ** lib_handle,
      const char * func_name);
  bool init_qnn_backend_interface(const std::string & backend_option, void ** backend_handle);
  bool init_qnn_graph_interface(const std::string & model_path, void ** model_handle);
  bool init_qnn_system_interface(const std::string & qnn_syslib_path, void ** sys_lib_handle);
};  // class QnnInterface

/// wrapper of QNN tensor and corresponding operations
class QnnTensor
{
public:
  QnnTensor() = default;
  QnnTensor(uint32_t num_of_input_tensors, uint32_t num_of_output_tensors);
  ~QnnTensor();
  StatusCode setup_tensors(Qnn_Tensor_t *& tensor,
      const uint32_t tensor_cnt,
      const Qnn_Tensor_t * tensor_src);
  StatusCode write_input_tensors(const std::vector<uint8_t> & input_data);
  std::vector<OutputTensor> read_output_tensors(const uint32_t number_of_outputs);
  void free_qnn_tensors(Qnn_Tensor_t * tensors, uint32_t tensors_cnt);
  StatusCode tensor_info_deep_copy(Qnn_Tensor_t * dst, const Qnn_Tensor_t * src);

  Qnn_Tensor_t * inputs = nullptr;
  Qnn_Tensor_t * outputs = nullptr;
  uint32_t num_of_input_tensors = 0;
  uint32_t num_of_output_tensors = 0;

private:
  std::vector<size_t> get_tensor_shape(const Qnn_Tensor_t * tensor);
  uint32_t get_tensor_size(const Qnn_Tensor_t * tensor, const std::vector<size_t> shape);
  StatusCode allocate_tensor_buf(void *& data, const std::vector<size_t> shape);
  void check_tensor_version(const Qnn_Tensor_t * tensor);
  int32_t qnn_dtype_to_qrb_dtype(const Qnn_DataType_t & data_type);

  inline void set_tensor_id(Qnn_Tensor_t * tensor, const uint32_t id);
  inline void set_tensor_type(Qnn_Tensor_t * tensor, const Qnn_TensorType_t type);
  inline void set_tensor_data_format(Qnn_Tensor_t * tensor, const Qnn_TensorDataFormat_t format);
  inline void set_tensor_data_type(Qnn_Tensor_t * tensor, const Qnn_DataType_t data_type);
  inline void set_tensor_quant_params(Qnn_Tensor_t * tensor, const Qnn_QuantizeParams_t params);
  inline void set_tensor_rank(Qnn_Tensor_t * tensor, const uint32_t rank);
  inline void set_tensor_dimensions(Qnn_Tensor_t * tensor, uint32_t * dims);
  inline void set_tensor_name(Qnn_Tensor_t * tensor, const char * name);
  inline void set_tensor_mem_type(Qnn_Tensor_t * tensor, const Qnn_TensorMemType_t mem_type);
  inline void set_tensor_client_buf(Qnn_Tensor_t * tensor, const Qnn_ClientBuffer_t client_buf);

  inline uint32_t get_tensor_id(const Qnn_Tensor_t * tensor);
  inline uint32_t * get_tensor_dimensions(const Qnn_Tensor_t * tensor);
  inline uint32_t get_tensor_rank(const Qnn_Tensor_t * tensor);
  inline const char * get_tensor_name(const Qnn_Tensor_t * tensor);
  inline Qnn_TensorType_t get_tensor_type(const Qnn_Tensor_t * tensor);
  inline Qnn_TensorDataFormat_t get_tensor_data_format(const Qnn_Tensor_t * tensor);
  inline Qnn_DataType_t get_tensor_data_type(const Qnn_Tensor_t * tensor);
  inline Qnn_QuantizeParams_t get_tensor_quant_params(const Qnn_Tensor_t * tensor);
  inline Qnn_ClientBuffer_t get_tensor_client_buf(const Qnn_Tensor_t * tensor);

};  // class QnnTensor

}  // namespace qrb::inference_mgr

#endif
