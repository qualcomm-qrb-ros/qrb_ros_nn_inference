// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <iostream>
#include <filesystem>

#include "qnn_inference/qnn_inference_impl.hpp"
#include "DynamicLoadUtil.hpp"
#include "QnnSampleAppUtils.hpp"
#include "Logger.hpp"
#include "IOTensor.hpp"
#include "PAL/DynamicLoading.hpp"
#include "QnnTypeMacros.hpp"
#include "DataUtil.hpp"

namespace qrb::inference_mgr
{
using namespace qnn::tools;

//*---------------------------------------------QnnInferenceCommon---------------------------------------------*//
QnnInferenceCommon::QnnInferenceCommon(const std::string &backend_option, const std::string &model_path):
backend_option_(backend_option),
model_path_(model_path)
{

}

QnnInferenceCommon::~QnnInferenceCommon()
{
  qnn_wrapper_api::freeGraphsInfo(&(this->graphs_info_), this->graphs_count_);
  this->graphs_info_ = nullptr;

  this->free_context();
  this->free_device();

  if (this->qnn_function_pointers_.qnnInterface.backendFree != nullptr) {
    QNN_DEBUG("Freeing backend");
    if (QNN_BACKEND_NO_ERROR != this->qnn_function_pointers_.qnnInterface.backendFree(this->backend_handle_)) {
      QNN_ERROR("Could not free backend");
    }
  }

  if (this->qnn_function_pointers_.qnnInterface.logFree != nullptr && this->log_handle_ != nullptr) {
    QNN_DEBUG("Freeing logging in the backend");
    if (QNN_SUCCESS != this->qnn_function_pointers_.qnnInterface.logFree(this->log_handle_)) {
      QNN_WARN("Unable to terminate logging in the backend.");
    }
  }

  if (this->backend_handle_) {
    // pal::dynamicloading::dlClose(this->backend_handle_);
    this->backend_handle_ = nullptr;
  }
  if (this->model_handle_) {
    // pal::dynamicloading::dlClose(this->model_handle_);
    this->model_handle_ = nullptr;
  }
}

/**
 * \brief initialize qnn_function_pointers_ for loading backend and model.so
*/
StatusCode QnnInferenceCommon::initialize()
{
  auto status_code = dynamicloadutil::getQnnFunctionPointers(this->backend_option_,
                                                            this->model_path_,
                                                            &(this->qnn_function_pointers_),
                                                            &(this->backend_handle_),
                                                            true,
                                                            &(this->model_handle_));
  if (dynamicloadutil::StatusCode::SUCCESS != status_code) {
    QNN_ERROR("Get qnn interface fail!");
    return StatusCode::FAILURE;
  }

  // initialize logging in the backend
  qnn::log::initializeLogging();
  if (qnn::log::isLogInitialized()) {
    auto logCallback = qnn::log::getLogCallback();
    auto logLevel = qnn::log::getLogLevel();
    QNN_INFO("Initializing logging in the backend. Callback: [%p], Log Level: [%d]",
            logCallback,
            logLevel);
    if (QNN_SUCCESS !=
        this->qnn_function_pointers_.qnnInterface.logCreate(logCallback, logLevel, &(this->log_handle_))) {
      QNN_WARN("Unable to initialize logging in the backend.");
    }
  } else {
    QNN_WARN("Logging not available in the backend.");
  }

  return StatusCode::SUCCESS;
}

StatusCode QnnInferenceCommon::initialize_backend()
{
  {
    auto qnnStatus = this->qnn_function_pointers_.qnnInterface.backendCreate(
      this->log_handle_, (const QnnBackend_Config_t**)(nullptr), &(this->backend_handle_));
    if (QNN_BACKEND_NO_ERROR != qnnStatus) {
      QNN_ERROR("Could not initialize backend due to error = %d", qnnStatus);
      return StatusCode::FAILURE;
    }
    QNN_INFO("Initialize Backend Returned Status = %d", qnnStatus);
  }
  return StatusCode::SUCCESS;
}

StatusCode QnnInferenceCommon::create_device()
{
  auto isDevicePropertySupported = [this] {
    if (nullptr != this->qnn_function_pointers_.qnnInterface.propertyHasCapability) {
      auto qnnStatus =
          this->qnn_function_pointers_.qnnInterface.propertyHasCapability(QNN_PROPERTY_GROUP_DEVICE);
      if (QNN_PROPERTY_NOT_SUPPORTED == qnnStatus) {
        QNN_WARN("Device property is not supported");
      }
      if (QNN_PROPERTY_ERROR_UNKNOWN_KEY == qnnStatus) {
        QNN_ERROR("Device property is not known to backend");
        return StatusCode::FAILURE;
      }
    }
    return StatusCode::SUCCESS;
  };

  if (StatusCode::FAILURE != isDevicePropertySupported()) {
    if (nullptr != this->qnn_function_pointers_.qnnInterface.deviceCreate) {
      auto qnnStatus =
          this->qnn_function_pointers_.qnnInterface.deviceCreate(this->log_handle_, nullptr, &(this->device_handle_));
      if (QNN_SUCCESS != qnnStatus && QNN_DEVICE_ERROR_UNSUPPORTED_FEATURE != qnnStatus) {
        QNN_ERROR("Failed to create device");
        return StatusCode::FAILURE;
      }
    }
    this->support_device = true;
  }
  return StatusCode::SUCCESS;
}

StatusCode QnnInferenceCommon::create_context()
{
  if (QNN_CONTEXT_NO_ERROR !=
      this->qnn_function_pointers_.qnnInterface.contextCreate(this->backend_handle_,
                                                              this->device_handle_,
                                                              (const QnnContext_Config_t**)(nullptr),
                                                              &(this->context_))) {
    QNN_ERROR("Could not create context");
    return StatusCode::FAILURE;
  }
  return StatusCode::SUCCESS;
}

StatusCode QnnInferenceCommon::compose_graphs()
{
  if (qnn_wrapper_api::ModelError_t::MODEL_NO_ERROR !=
      this->qnn_function_pointers_.composeGraphsFnHandle(this->backend_handle_,
                                                          this->qnn_function_pointers_.qnnInterface,
                                                          this->context_,
                                                          (const qnn_wrapper_api::GraphConfigInfo_t**)nullptr,
                                                          0,
                                                          &(this->graphs_info_),
                                                          &(this->graphs_count_),
                                                          false,
                                                          qnn::log::getLogCallback(),
                                                          qnn::log::getLogLevel())) {
    QNN_ERROR("Failed in composeGraphs()");
    return StatusCode::FAILURE;
  }
  return StatusCode::SUCCESS;
}

StatusCode QnnInferenceCommon::finalize_graphs()
{
  for (size_t graphIdx = 0; graphIdx < this->graphs_count_; graphIdx++) {
    if (QNN_GRAPH_NO_ERROR != this->qnn_function_pointers_.qnnInterface.graphFinalize(
            (*this->graphs_info_)[graphIdx].graph, nullptr, nullptr)) {
      return StatusCode::FAILURE;
    }
  }

  return StatusCode::SUCCESS;
}

void QnnInferenceCommon::free_context()
{
  if (QNN_CONTEXT_NO_ERROR !=
      this->qnn_function_pointers_.qnnInterface.contextFree(this->context_, nullptr)) {
    QNN_ERROR("Could not free context");
  }
}

void QnnInferenceCommon::free_device()
{
  if(true == this->support_device) {
    if (nullptr != this->qnn_function_pointers_.qnnInterface.deviceFree) {
      auto qnnStatus = this->qnn_function_pointers_.qnnInterface.deviceFree(this->device_handle_);
      if (QNN_SUCCESS != qnnStatus && QNN_DEVICE_ERROR_UNSUPPORTED_FEATURE != qnnStatus) {
        QNN_ERROR("Failed to free device");
      }
    }
  }
}

//*---------------------------------------------QnnInferenceFromFileImpl---------------------------------------------*//

QnnInferenceFromFileImpl::QnnInferenceFromFileImpl(const std::string &backend_option, const std::string &model_path)
{
  this->qnn_inference_common_ = std::make_unique<QnnInferenceCommon>(backend_option, model_path);
}

StatusCode QnnInferenceFromFileImpl::initialize()
{
  auto status_code = this->qnn_inference_common_->initialize();
  std::string home_path = std::getenv("HOME");
  this->inputfile_path_= home_path + "/tmp/qrb_input/input_data.txt";
  this->outputfile_path_= home_path + "/tmp/qrb_output";

  #ifndef __hexagon__
  if (!std::filesystem::exists(this->outputfile_path_)) {
    std::filesystem::create_directory(this->outputfile_path_);
  }
  #endif
  std::vector<std::string> input_list_path;
  sample_app::split(input_list_path, this->inputfile_path_, ',');
  bool readSuccess;
  std::tie(this->inputfile_lists_, this->inputname_to_index_, readSuccess) = sample_app::readInputLists(input_list_path);

  if(!readSuccess || status_code != StatusCode::SUCCESS) {
    return StatusCode::FAILURE;
  }

  return StatusCode::SUCCESS;
}

StatusCode QnnInferenceFromFileImpl::initialize_backend()
{
  return this->qnn_inference_common_->initialize_backend();
}

StatusCode QnnInferenceFromFileImpl::create_device()
{
  return this->qnn_inference_common_->create_device();
}

StatusCode QnnInferenceFromFileImpl::create_context()
{
  return this->qnn_inference_common_->create_context();
}

StatusCode QnnInferenceFromFileImpl::compose_graphs()
{
  return this->qnn_inference_common_->compose_graphs();
}

StatusCode QnnInferenceFromFileImpl::finalize_graphs()
{
  return this->qnn_inference_common_->finalize_graphs();
}

StatusCode QnnInferenceFromFileImpl::execute_graphs()
{
  auto status_code = StatusCode::SUCCESS;
  for (size_t graphIdx = 0; graphIdx < this->qnn_inference_common_->graphs_count_; graphIdx++) {
    Qnn_Tensor_t* inputs  = nullptr;
    Qnn_Tensor_t* outputs = nullptr;
    iotensor::IOTensor io_tensors;

    if (iotensor::StatusCode::SUCCESS !=
        io_tensors.setupInputAndOutputTensors(&inputs, &outputs, (*(this->qnn_inference_common_->graphs_info_))[graphIdx])) {
      QNN_ERROR("Error in setting up Input and output Tensors for graphIdx: %d", graphIdx);
      status_code = StatusCode::FAILURE;
      break;
    }

    auto inputFileList = this->inputfile_lists_[graphIdx];
    auto graphInfo     = (*(this->qnn_inference_common_->graphs_info_))[graphIdx];

    if (!inputFileList.empty()) {
      size_t totalCount = inputFileList[0].size();
      size_t inputFileIndexOffset = 0;

      while (inputFileIndexOffset < totalCount) {
        iotensor::StatusCode iotReturnStatus;
        size_t numInputFilesPopulated;
        size_t batchSize;
        std::tie(iotReturnStatus, numInputFilesPopulated, batchSize) =
            io_tensors.populateInputTensors(graphIdx,
                                            inputFileList,
                                            inputFileIndexOffset,
                                            false,
                                            this->inputname_to_index_[graphIdx],
                                            inputs,
                                            graphInfo,
                                            iotensor::InputDataType::FLOAT);

        if (iotensor::StatusCode::SUCCESS != iotReturnStatus) {
          status_code = StatusCode::FAILURE;
        }

        if (StatusCode::SUCCESS == status_code) {
          Qnn_ErrorHandle_t executeStatus = QNN_GRAPH_NO_ERROR;
          executeStatus =
              this->qnn_inference_common_->qnn_function_pointers_.qnnInterface.graphExecute(graphInfo.graph,
                                                              inputs,
                                                              graphInfo.numInputTensors,
                                                              outputs,
                                                              graphInfo.numOutputTensors,
                                                              nullptr,
                                                              nullptr);
          if (QNN_GRAPH_NO_ERROR != executeStatus) {
            status_code = StatusCode::FAILURE;
          }

          if (StatusCode::SUCCESS == status_code) {
            #ifndef __hexagon__
            if (iotensor::StatusCode::SUCCESS != io_tensors.writeOutputTensors(graphIdx,
                                                                              inputFileIndexOffset,
                                                                              graphInfo.graphName,
                                                                              outputs,
                                                                              graphInfo.numOutputTensors,
                                                                              iotensor::OutputDataType::FLOAT_ONLY,
                                                                              this->qnn_inference_common_->graphs_count_,
                                                                              this->outputfile_path_,
                                                                              numInputFilesPopulated,
                                                                              batchSize)) {
              status_code = StatusCode::FAILURE;
            }
            #endif
          }
          inputFileIndexOffset += numInputFilesPopulated;
        }
        if (StatusCode::SUCCESS != status_code) {
          QNN_ERROR("Execution of Graph: %d failed!", graphIdx);
          break;
        }
      } // while
    } // if !inputFileList.empty()

    io_tensors.tearDownInputAndOutputTensors(
        inputs, outputs, graphInfo.numInputTensors, graphInfo.numOutputTensors);
    inputs  = nullptr;
    outputs = nullptr;
    if (StatusCode::SUCCESS != status_code) {
      break;
    }
  } // for

  qnn_wrapper_api::freeGraphsInfo(&(this->qnn_inference_common_->graphs_info_), this->qnn_inference_common_->graphs_count_);
  this->qnn_inference_common_->graphs_info_ = nullptr;
  return StatusCode::SUCCESS;
}

//*---------------------------------------------QnnInferenceImpl---------------------------------------------*//
QnnInferenceImpl::QnnInferenceImpl(const std::string &backend_option, const std::string &model_path)
{
  this->qnn_inference_common_ = std::make_unique<QnnInferenceCommon>(backend_option, model_path);
}

StatusCode QnnInferenceImpl::initialize()
{
  QRB_DEBUG("****attention: QnnInferenceImpl::initialize() init1****");
  return this->qnn_inference_common_->initialize();
}

StatusCode QnnInferenceImpl::initialize_backend()
{
  return this->qnn_inference_common_->initialize_backend();
}

StatusCode QnnInferenceImpl::create_device()
{
  return this->qnn_inference_common_->create_device();
}

StatusCode QnnInferenceImpl::create_context()
{
  return this->qnn_inference_common_->create_context();
}

StatusCode QnnInferenceImpl::compose_graphs()
{
  return this->qnn_inference_common_->compose_graphs();
}

StatusCode QnnInferenceImpl::finalize_graphs()
{
  return this->qnn_inference_common_->finalize_graphs();
}

StatusCode QnnInferenceImpl::populateInputTensorsFromBuffer
(
  Qnn_Tensor_t* input,
  const std::vector<uint8_t> &input_tensor_data
)
{
  if((input == nullptr) || (QNN_TENSOR_GET_DIMENSIONS(input) == nullptr)) {
    return StatusCode::FAILURE;
  }
  std::vector<size_t> dims;
  for (size_t r = 0; r < QNN_TENSOR_GET_RANK(input); r++) {
    dims.emplace_back(QNN_TENSOR_GET_DIMENSIONS(input)[r]);
  }

  if(QNN_TENSOR_GET_DATA_TYPE(input) == QNN_DATATYPE_FLOAT_32) {
    auto [status_tmp, tensor_size] = datautil::calculateLength(dims, QNN_TENSOR_GET_DATA_TYPE(input));
    if (datautil::StatusCode::SUCCESS != status_tmp) {
      return StatusCode::FAILURE;
    }

    if(tensor_size != input_tensor_data.size()) {
      QNN_ERROR("The size of input tensor should be %u byte, but receive %u byte", tensor_size, input_tensor_data.size());
      return StatusCode::FAILURE;
    }

    // * QNN_TENSOR_GET_CLIENT_BUF(input).data is const and can not point to input_tensor_data directly
    memcpy(static_cast<char*>(QNN_TENSOR_GET_CLIENT_BUF(input).data), input_tensor_data.data(), input_tensor_data.size());
  }
  else {
    QNN_ERROR("Input tensor data type not support!");
    return StatusCode::FAILURE;
  }

  return StatusCode::SUCCESS;
}

StatusCode QnnInferenceImpl::writeOutputTensorsToBuffer(Qnn_Tensor_t* outputs, uint32_t number_of_outputs)
{
  if((outputs == nullptr)) {
    return StatusCode::FAILURE;
  }

  this->output_tensor_ = std::vector<OutputTensor>(number_of_outputs);
  for (uint32_t i = 0; i < number_of_outputs; i++) {
    auto output = &(outputs[i]);

    std::vector<size_t> dims;
    for (size_t r = 0; r < QNN_TENSOR_GET_RANK(output); r++) {
      dims.emplace_back(QNN_TENSOR_GET_DIMENSIONS(output)[r]);
    }

    if(QNN_TENSOR_GET_DATA_TYPE(output) == QNN_DATATYPE_FLOAT_32) {
      auto [status_tmp, tensor_size] = datautil::calculateLength(dims, QNN_TENSOR_GET_DATA_TYPE(output));
      if (datautil::StatusCode::SUCCESS != status_tmp) {
        return StatusCode::FAILURE;
      }

      auto qnn_output_tensor_buf = static_cast<uint8_t*>(QNN_TENSOR_GET_CLIENT_BUF(output).data);
      this->output_tensor_[i].output_tensor_data_ = std::vector<uint8_t>(qnn_output_tensor_buf, qnn_output_tensor_buf + tensor_size);
      this->output_tensor_[i].output_tensor_name_ = QNN_TENSOR_GET_NAME(output);
      this->output_tensor_[i].output_tensor_shape_ = std::vector<uint32_t>(dims.begin(), dims.end());
    }
    else {
      QNN_ERROR("Input tensor data type not support!");
      return StatusCode::FAILURE;
    }
  }

  return StatusCode::SUCCESS;
}

StatusCode QnnInferenceImpl::execute_graphs(const std::vector<uint8_t> &input_tensor_data)
{
  auto status_code = StatusCode::SUCCESS;

  Qnn_Tensor_t* inputs  = nullptr;
  Qnn_Tensor_t* outputs = nullptr;
  iotensor::IOTensor io_tensors;

  io_tensors.setupInputAndOutputTensors(&inputs, &outputs, (*(this->qnn_inference_common_->graphs_info_))[0]);

  if (StatusCode::SUCCESS == status_code) {
    if (iotensor::StatusCode::SUCCESS
        != io_tensors.setupInputAndOutputTensors(&inputs, &outputs, (*(this->qnn_inference_common_->graphs_info_))[0])) {
      QNN_ERROR("QNN set up input and output tensor failed!");
      status_code = StatusCode::FAILURE;
    }
  }

  if (StatusCode::SUCCESS == status_code) {
    if (StatusCode::SUCCESS != populateInputTensorsFromBuffer(&(inputs[0]), input_tensor_data)) {
      QNN_ERROR("QNN populate input tensor failed!");
      status_code = StatusCode::FAILURE;
    }
  }

  auto graphInfo = (*(this->qnn_inference_common_->graphs_info_))[0];
  if (StatusCode::SUCCESS == status_code) {
    if (QNN_GRAPH_NO_ERROR
        != this->qnn_inference_common_->qnn_function_pointers_.qnnInterface.graphExecute(graphInfo.graph,
                                                                                        inputs,
                                                                                        graphInfo.numInputTensors,
                                                                                        outputs,
                                                                                        graphInfo.numOutputTensors,
                                                                                        nullptr,
                                                                                        nullptr)) {
      QNN_ERROR("QNN graphExecute failed!");
      status_code = StatusCode::FAILURE;
    }
  }

#ifndef __hexagon__
  if (StatusCode::SUCCESS == status_code) {
    if (StatusCode::SUCCESS != writeOutputTensorsToBuffer(outputs, graphInfo.numOutputTensors)) {
      QNN_ERROR("QNN write output tensor to buffer failed!");
      status_code = StatusCode::FAILURE;
    }
  }
#endif

  if (iotensor::StatusCode::SUCCESS
      != io_tensors.tearDownInputAndOutputTensors(inputs, outputs, graphInfo.numInputTensors, graphInfo.numOutputTensors)) {
    QNN_ERROR("QNN free input and output tensor failed!");
    inputs  = nullptr;
    outputs = nullptr;
    status_code = StatusCode::FAILURE;
  }

  return status_code;
}

const std::vector<OutputTensor> QnnInferenceImpl::get_output_tensors()
{
  return std::move(this->output_tensor_);
}

} // namespace qrb::inference_mgr