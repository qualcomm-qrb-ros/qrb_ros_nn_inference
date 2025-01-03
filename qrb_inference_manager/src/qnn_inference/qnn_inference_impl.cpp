// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qnn_inference/qnn_inference_impl.hpp"

namespace qrb::inference_mgr
{

QnnInterface::QnnInterface(const std::string & model_path,
    const std::string & backend_option,
    void ** backend_handle,
    void ** model_handle)
{
  if (false == init_qnn_backend_interface(backend_option, backend_handle)) {
    throw std::logic_error("ERROR: Get QNN backend interface failed!");
  }

  if (false == init_qnn_graph_interface(model_path, model_handle)) {
    throw std::logic_error("ERROR: Get QNN graph interface failed!");
  }
}

QnnInterface::QnnInterface(const std::string & backend_option,
    void ** backend_handle,
    const std::string & qnn_syslib_path,
    void ** sys_lib_handle)
{
  if (false == init_qnn_backend_interface(backend_option, backend_handle)) {
    throw std::logic_error("ERROR: Get QNN backend interface failed!");
  }

  if (false == init_qnn_system_interface(qnn_syslib_path, sys_lib_handle)) {
    throw std::logic_error("ERROR: Get QNN system interface failed!");
  }
}

template <class T>
inline T QnnInterface::get_function_from_lib(const std::string & lib_name,
    const int open_flag,
    void ** lib_handle,
    const char * func_name)
{
  if (nullptr == *lib_handle) {
    *lib_handle = ::dlopen(lib_name.c_str(), open_flag);
    if (nullptr == *lib_handle) {
      QRB_ERROR("Unable to open lib!");
      return nullptr;
    }
  }

  auto temp_lib_handle = *lib_handle;
  auto func_ptr = ::dlsym(temp_lib_handle, func_name);
  if (nullptr == func_ptr) {
    QRB_ERROR("Unable to get function: ", func_name);
    return nullptr;
  }

  return reinterpret_cast<T>(func_ptr);
}

/// @brief get the qnn interface from backend library
/// @param backend_option file path of libQnnBackend.so
/// @param backend_handle pointer point to qnn interface
/// @return true of false
bool QnnInterface::init_qnn_backend_interface(const std::string & backend_option,
    void ** backend_handle)
{
  auto get_interface_providers = get_function_from_lib<qnn_interface_providers_func>(
      backend_option, RTLD_NOW | RTLD_GLOBAL, backend_handle, "QnnInterface_getProviders");

  QnnInterface_t ** interface_providers = nullptr;
  uint32_t num_providers = 0;

  if (QNN_SUCCESS !=
      get_interface_providers((const QnnInterface_t ***)&interface_providers, &num_providers)) {
    QRB_ERROR("Failed to get interface providers!");
    return false;
  }

  if (interface_providers == nullptr || num_providers == 0) {
    QRB_ERROR("Invalid interface providers retrieved!");
    return false;
  }

  for (size_t i = 0; i < num_providers; i++) {
    if (QNN_API_VERSION_MAJOR == interface_providers[i]->apiVersion.coreApiVersion.major &&
        QNN_API_VERSION_MINOR <= interface_providers[i]->apiVersion.coreApiVersion.minor) {
      this->interface = interface_providers[i]->QNN_INTERFACE_VER_NAME;
      return true;
    }
  }

  QRB_ERROR("Unable to find a valid interface!");
  return false;
}

/// @brief get the qnn interface from model lib
/// @param model_path
/// @param model_handle pointer point to qnn interface
/// @return true or false
bool QnnInterface::init_qnn_graph_interface(const std::string & model_path, void ** model_handle)
{
  this->compose_graphs = get_function_from_lib<compose_graphs_func>(
      model_path, RTLD_NOW | RTLD_LOCAL, model_handle, "QnnModel_composeGraphs");

  this->free_graph_info = get_function_from_lib<free_graph_info_func>(
      model_path, RTLD_NOW | RTLD_LOCAL, model_handle, "QnnModel_freeGraphsInfo");

  return true;
}

/// @brief get qnn system interfaces for inference of binary model
/// @param qnn_syslib_path file path of libQnnSystem.so
/// @param sys_lib_handle pointer point to qnn system interface
/// @return true of false
bool QnnInterface::init_qnn_system_interface(const std::string & qnn_syslib_path,
    void ** sys_lib_handle)
{
  auto get_sys_interface_providers = get_function_from_lib<qnn_sys_interface_providers_func>(
      qnn_syslib_path, RTLD_NOW | RTLD_LOCAL, sys_lib_handle, "QnnSystemInterface_getProviders");

  QnnSystemInterface_t ** sys_interface_providers = nullptr;
  uint32_t number_of_providers = 0;

  if (QNN_SUCCESS !=
      get_sys_interface_providers(
          (const QnnSystemInterface_t ***)&sys_interface_providers, &number_of_providers)) {
    QRB_ERROR("Failed to get system interface providers.");
    return false;
  }

  if (nullptr == sys_interface_providers || number_of_providers == 0) {
    QRB_ERROR("Failed to get system interface providers: null interface providers received.");
    return false;
  }

  for (size_t i = 0; i < number_of_providers; i++) {
    if (QNN_SYSTEM_API_VERSION_MAJOR == sys_interface_providers[i]->systemApiVersion.major &&
        QNN_SYSTEM_API_VERSION_MINOR <= sys_interface_providers[i]->systemApiVersion.minor) {
      qnn_system_interface = sys_interface_providers[i]->QNN_SYSTEM_INTERFACE_VER_NAME;
      return true;
    }
  }

  QRB_ERROR("Unable to find a valid system interface.");
  return false;
}

QnnTensor::QnnTensor(uint32_t num_of_input_tensors, uint32_t num_of_output_tensors)
  : num_of_input_tensors(num_of_input_tensors), num_of_output_tensors(num_of_output_tensors)
{
}

QnnTensor::~QnnTensor()
{
  free_qnn_tensors(inputs, num_of_input_tensors);
  free_qnn_tensors(outputs, num_of_output_tensors);
}

/// @brief setup tensor based on tensor_src
/// @param tensor
/// @param tensor_cnt number of tensor
/// @param tensor_src
/// @return SUCCESS or FAILURE
StatusCode QnnTensor::setup_tensors(Qnn_Tensor_t *& tensor,
    const uint32_t tensor_cnt,
    const Qnn_Tensor_t * tensor_src)
{
  if (tensor_src == nullptr || tensor_cnt == 0) {
    return StatusCode::FAILURE;
  }

  tensor = (Qnn_Tensor_t *)calloc(1, tensor_cnt * sizeof(Qnn_Tensor_t));
  if (tensor == nullptr) {
    QRB_ERROR("calloc tensor failed!");
    return StatusCode::FAILURE;
  }

  for (size_t i = 0; i < tensor_cnt; i++) {
    const Qnn_Tensor_t & tensor_src_tmp = tensor_src[i];

    std::vector<size_t> tensor_shape = get_tensor_shape(&tensor_src_tmp);
    if (tensor_shape.size() == 0) {
      return StatusCode::FAILURE;
    }

    tensor[i] = QNN_TENSOR_INIT;
    if (StatusCode::SUCCESS != tensor_info_deep_copy(tensor + i, &tensor_src_tmp)) {
      return StatusCode::FAILURE;
    }

    set_tensor_mem_type(tensor + i, QNN_TENSORMEMTYPE_RAW);

    Qnn_ClientBuffer_t tensor_buf = QNN_CLIENT_BUFFER_INIT;

    if (StatusCode::SUCCESS != allocate_tensor_buf(tensor_buf.data, tensor_shape)) {
      return StatusCode::FAILURE;
    }
    tensor_buf.dataSize = get_tensor_size(tensor + i, tensor_shape);

    set_tensor_client_buf(tensor + i, tensor_buf);
  }

  return StatusCode::SUCCESS;
}

/// @brief fill input_data into input tensor for inference
/// @param input_data
/// @return SUCCESS or FAILURE
StatusCode QnnTensor::write_input_tensors(const std::vector<uint8_t> & input_data)
{
  if ((inputs == nullptr) || (get_tensor_dimensions(inputs) == nullptr)) {
    return StatusCode::FAILURE;
  }

  std::vector<size_t> shape;
  for (size_t i = 0; i < get_tensor_rank(inputs); i++) {
    shape.emplace_back(get_tensor_dimensions(inputs)[i]);
  }

  if (get_tensor_data_type(inputs) == QNN_DATATYPE_FLOAT_32) {  // now only support FLOAT32
    size_t tensor_size = get_tensor_size(inputs, shape);

    if (tensor_size != input_data.size()) {
      QRB_ERROR("The size of input tensor should be %u byte, but receive %u byte", tensor_size,
          input_data.size());
      return StatusCode::FAILURE;
    }

    // get_tensor_client_buf(input).data is const and can not point to input_tensor_data directly
    memcpy(static_cast<char *>(get_tensor_client_buf(inputs).data), input_data.data(),
        input_data.size());
  } else {
    QRB_ERROR("Input tensor data type not support!");
    return StatusCode::FAILURE;
  }

  return StatusCode::SUCCESS;
}

/// @brief get information of output tensors, incluing name, shape, data
/// @param number_of_outputs number of output tensors
/// @return all information of output tensors
std::vector<OutputTensor> QnnTensor::read_output_tensors(const uint32_t number_of_outputs)
{
  auto res = std::vector<OutputTensor>(number_of_outputs);

  if (nullptr == outputs) {
    return std::vector<OutputTensor>();
  }

  for (uint32_t i = 0; i < number_of_outputs; i++) {
    const auto output = &(outputs[i]);

    std::vector<size_t> shape;
    for (size_t i = 0; i < get_tensor_rank(output); i++) {
      shape.emplace_back(get_tensor_dimensions(output)[i]);
    }

    if (get_tensor_data_type(output) == QNN_DATATYPE_FLOAT_32) {
      size_t tensor_size = get_tensor_size(output, shape);

      auto output_buf = static_cast<uint8_t *>(get_tensor_client_buf(output).data);
      res[i].output_tensor_data = std::vector<uint8_t>(output_buf, output_buf + tensor_size);
      res[i].output_tensor_name = get_tensor_name(output);
      res[i].output_tensor_shape = std::vector<uint32_t>(shape.begin(), shape.end());
    } else {
      QRB_ERROR("Input tensor data type not support!");
      return std::vector<OutputTensor>();
    }
  }

  return res;
}

/// @brief free qnn tensor: Qnn_Tensor_t
/// @param tensors
/// @param tensors_cnt number of tensors
void QnnTensor::free_qnn_tensors(Qnn_Tensor_t * tensors, uint32_t tensors_cnt)
{
  if (nullptr == tensors) {
    return;
  }

  auto check_and_free = [](auto & ptr) {
    if (ptr != nullptr) {
      free((void *)ptr);
      ptr = nullptr;
    }
  };

  for (size_t i = 0; i < tensors_cnt; i++) {
    if (tensors[i].version == QNN_TENSOR_VERSION_1) {
      check_and_free(tensors[i].v1.dimensions);
      check_and_free(tensors[i].v1.name);
      check_and_free(tensors[i].v1.clientBuf.data);
      check_and_free(tensors[i].v1.quantizeParams.axisScaleOffsetEncoding.scaleOffset);
    }
  }

  check_and_free(tensors);
}

/// @brief copy all information of src to dst
/// @param dst
/// @param src
/// @return SUCCESS or FAILURE
StatusCode QnnTensor::tensor_info_deep_copy(Qnn_Tensor_t * dst, const Qnn_Tensor_t * src)
{
  if (nullptr == dst || nullptr == src) {
    return StatusCode::FAILURE;
  }

  // set tensor.version before using QNN_TENSOR_SET macros, as they require the version to be set
  // to correctly assign values
  dst->version = src->version;

  if (const char * src_tensor_name = get_tensor_name(src); nullptr == src_tensor_name) {
    QRB_ERROR("Tensor name is nullptr, fail to set tensor name!");
    return StatusCode::FAILURE;
  } else {
    char * temp_name = (char *)malloc(sizeof(char) * (1 + strlen(src_tensor_name)));
    if (nullptr == temp_name) {
      return StatusCode::FAILURE;
    }
    memcpy(temp_name, src_tensor_name, strlen(src_tensor_name));
    temp_name[strlen(src_tensor_name)] = '\0';

    set_tensor_name(dst, temp_name);
  }

  set_tensor_id(dst, get_tensor_id(src));
  set_tensor_type(dst, get_tensor_type(src));
  set_tensor_data_format(dst, get_tensor_data_format(src));
  set_tensor_data_type(dst, get_tensor_data_type(src));

  Qnn_QuantizeParams_t dst_param = QNN_QUANTIZE_PARAMS_INIT;
  Qnn_QuantizeParams_t src_param = get_tensor_quant_params(src);
  dst_param.encodingDefinition = src_param.encodingDefinition;
  dst_param.quantizationEncoding = QNN_QUANTIZATION_ENCODING_UNDEFINED;

  if (src_param.quantizationEncoding == QNN_QUANTIZATION_ENCODING_SCALE_OFFSET) {
    dst_param.quantizationEncoding = src_param.quantizationEncoding;
    dst_param.scaleOffsetEncoding = src_param.scaleOffsetEncoding;
  } else if (src_param.quantizationEncoding == QNN_QUANTIZATION_ENCODING_AXIS_SCALE_OFFSET) {
    dst_param.quantizationEncoding = src_param.quantizationEncoding;
    dst_param.axisScaleOffsetEncoding.axis = src_param.axisScaleOffsetEncoding.axis;
    dst_param.axisScaleOffsetEncoding.numScaleOffsets =
        src_param.axisScaleOffsetEncoding.numScaleOffsets;

    if (src_param.axisScaleOffsetEncoding.numScaleOffsets > 0) {
      dst_param.axisScaleOffsetEncoding.scaleOffset = (Qnn_ScaleOffset_t *)malloc(
          src_param.axisScaleOffsetEncoding.numScaleOffsets * sizeof(Qnn_ScaleOffset_t));

      if (dst_param.axisScaleOffsetEncoding.scaleOffset) {
        for (size_t idx = 0; idx < src_param.axisScaleOffsetEncoding.numScaleOffsets; idx++) {
          dst_param.axisScaleOffsetEncoding.scaleOffset[idx].scale =
              src_param.axisScaleOffsetEncoding.scaleOffset[idx].scale;
          dst_param.axisScaleOffsetEncoding.scaleOffset[idx].offset =
              src_param.axisScaleOffsetEncoding.scaleOffset[idx].offset;
        }
      }
    }
  }

  set_tensor_quant_params(dst, dst_param);
  set_tensor_rank(dst, get_tensor_rank(src));
  set_tensor_dimensions(dst, nullptr);

  if (get_tensor_rank(src) > 0) {
    set_tensor_dimensions(dst, (uint32_t *)malloc(get_tensor_rank(src) * sizeof(uint32_t)));
    if (get_tensor_dimensions(dst)) {
      if (get_tensor_dimensions(dst) == nullptr || get_tensor_dimensions(src) == nullptr) {
        return StatusCode::FAILURE;
      }
      memcpy(get_tensor_dimensions(dst), get_tensor_dimensions(src),
          get_tensor_rank(src) * sizeof(uint32_t));
    } else {
      return StatusCode::FAILURE;
    }
  } else {
    return StatusCode::FAILURE;
  }

  return StatusCode::SUCCESS;
}

/// @brief get the shape of tensor
/// @param tensor
/// @return shape of tensor
std::vector<size_t> QnnTensor::get_tensor_shape(const Qnn_Tensor_t * tensor)
{
  uint32_t * dimensions = get_tensor_dimensions(tensor);
  uint32_t rank = get_tensor_rank(tensor);

  if (dimensions == nullptr) {
    return std::vector<size_t>();
  }

  std::vector<size_t> shape;
  for (size_t i = 0; i < rank; i++) {
    shape.emplace_back(dimensions[i]);
  }

  return shape;
}

/// @brief get the data size of tensor, shape * sizeof(data_type)
/// @param tensor
/// @param shape shape of tensor
/// @return data size of tensor
uint32_t QnnTensor::get_tensor_size(const Qnn_Tensor_t * tensor, const std::vector<size_t> shape)
{
  size_t element_cnt = std::accumulate(shape.begin(), shape.end(), 1, std::multiplies<size_t>());

  if (get_tensor_data_type(tensor) != QNN_DATATYPE_FLOAT_32) {
    QRB_WARNING("Input data type is not suppport!");
  }

  return sizeof(float) * element_cnt;
}

/// @brief allocate buffer for tensor data
/// @param data pointer point to tensor data buffer
/// @param shape shape of tensor
/// @return SUCCESS or FAILURE
StatusCode QnnTensor::allocate_tensor_buf(void *& data, const std::vector<size_t> shape)
{
  size_t element_cnt = std::accumulate(shape.begin(), shape.end(), 1, std::multiplies<size_t>());
  data = (float *)malloc(element_cnt * sizeof(float));  // default is float

  if (data == nullptr) {
    return StatusCode::FAILURE;
  }
  return StatusCode::SUCCESS;
}

void QnnTensor::check_tensor_version(const Qnn_Tensor_t * tensor)
{
  if (tensor->version != QNN_TENSOR_VERSION_1) {
    QRB_WARNING("Version of tensor is NOT QNN_TENSOR_VERSION_1");
  }
}

inline void QnnTensor::set_tensor_id(Qnn_Tensor_t * tensor, const uint32_t id)
{
  check_tensor_version(tensor);
  tensor->v1.id = id;
}

inline void QnnTensor::set_tensor_type(Qnn_Tensor_t * tensor, const Qnn_TensorType_t type)
{
  check_tensor_version(tensor);
  tensor->v1.type = type;
}

inline void QnnTensor::set_tensor_data_format(Qnn_Tensor_t * tensor,
    const Qnn_TensorDataFormat_t format)
{
  check_tensor_version(tensor);
  tensor->v1.dataFormat = format;
}

inline void QnnTensor::set_tensor_data_type(Qnn_Tensor_t * tensor, const Qnn_DataType_t data_type)
{
  check_tensor_version(tensor);
  tensor->v1.dataType = data_type;
}

inline void QnnTensor::set_tensor_quant_params(Qnn_Tensor_t * tensor,
    const Qnn_QuantizeParams_t params)
{
  check_tensor_version(tensor);
  tensor->v1.quantizeParams = params;
}

inline void QnnTensor::set_tensor_rank(Qnn_Tensor_t * tensor, const uint32_t rank)
{
  check_tensor_version(tensor);
  tensor->v1.rank = rank;
}

inline void QnnTensor::set_tensor_dimensions(Qnn_Tensor_t * tensor, uint32_t * dims)
{
  check_tensor_version(tensor);
  tensor->v1.dimensions = dims;
}

inline void QnnTensor::set_tensor_name(Qnn_Tensor_t * tensor, const char * name)
{
  check_tensor_version(tensor);
  tensor->v1.name = name;
}

inline void QnnTensor::set_tensor_mem_type(Qnn_Tensor_t * tensor,
    const Qnn_TensorMemType_t mem_type)
{
  check_tensor_version(tensor);
  tensor->v1.memType = mem_type;
}

inline void QnnTensor::set_tensor_client_buf(Qnn_Tensor_t * tensor,
    const Qnn_ClientBuffer_t client_buf)
{
  check_tensor_version(tensor);
  tensor->v1.clientBuf = client_buf;
}

inline uint32_t QnnTensor::get_tensor_id(const Qnn_Tensor_t * tensor)
{
  check_tensor_version(tensor);
  return tensor->v1.id;
}

inline uint32_t * QnnTensor::get_tensor_dimensions(const Qnn_Tensor_t * tensor)
{
  check_tensor_version(tensor);
  return tensor->v1.dimensions;
}

inline uint32_t QnnTensor::get_tensor_rank(const Qnn_Tensor_t * tensor)
{
  check_tensor_version(tensor);
  return tensor->v1.rank;
}

inline const char * QnnTensor::get_tensor_name(const Qnn_Tensor_t * tensor)
{
  check_tensor_version(tensor);
  return tensor->v1.name;
}

inline Qnn_TensorType_t QnnTensor::get_tensor_type(const Qnn_Tensor_t * tensor)
{
  check_tensor_version(tensor);
  return tensor->v1.type;
}

inline Qnn_TensorDataFormat_t QnnTensor::get_tensor_data_format(const Qnn_Tensor_t * tensor)
{
  check_tensor_version(tensor);
  return tensor->v1.dataFormat;
}

inline Qnn_DataType_t QnnTensor::get_tensor_data_type(const Qnn_Tensor_t * tensor)
{
  check_tensor_version(tensor);
  return tensor->v1.dataType;
}

inline Qnn_QuantizeParams_t QnnTensor::get_tensor_quant_params(const Qnn_Tensor_t * tensor)
{
  check_tensor_version(tensor);
  return tensor->v1.quantizeParams;
}

inline Qnn_ClientBuffer_t QnnTensor::get_tensor_client_buf(const Qnn_Tensor_t * tensor)
{
  check_tensor_version(tensor);
  return tensor->v1.clientBuf;
}

}  // namespace qrb::inference_mgr