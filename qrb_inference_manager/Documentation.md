* [Introduction](#0)
* [Head files]()
  * [qrb_inference_manager.hpp](./include/qrb_inference_manager.hpp)
* [Struct](#1)
  * [OutputTensor](#1.1)
* [Functions](#2)
  * [QrbInferenceManager](#2.1)
  * [inference_execute](#2.2)
  * [get_output_tensors](#2.3)

<h1 id="0">Introduction</h1>

This is the API documentation of qrb_inference_manager, you can access [example](./test/example.cpp) to see the minimal example.

<h1 id="1">Struct</h1>

<h3 id="1.1">OutputTensor</h1>

```cpp
struct OutputTensor {
  std::vector<uint8_t> output_tensor_data;
  std::string output_tensor_name;
  std::vector<uint32_t> output_tensor_shape;
};
```

<h1 id="2">Functions</h1>

<h3 id="2.1">QrbInferenceManager - <a href="https://github.com/quic-qrb-ros/qrb_ros_nn_inference/blob/main/qrb_inference_manager/src/qrb_inference_manager.cpp#L19">[source]</a></h3>

```cpp
/**
 * \brief initialize qrb_inference_ to QnnInference or QnnDelegateInference
 * \param backend_option backend lib of QNN
 * \param model_path path of model
 * \throw std::logic_error, if param not meet requirement
 */
QrbInferenceManager(const std::string &model_path,
                    const std::string &backend_option = "");
```

Warning⚠️: Models in different formats need different backend option, make sure your parameters are vaild

|   model_path   | backend_option |         Desctiption         |
| :------------: | :------------: | :-------------------------: |
| "model.tflite" |       ""       | inference with no delegate  |
| "model.tflite" |     "gpu"      | inference with GPU delegate |
| "model.tflite" |     "htp"      | inference with HTP delegate |
| "model.tflite" |     "dsp"      | inference with DSP delegate |
|   "model.so"   | "libQnnCpu.so" |     inference with CPU      |
|   "model.so"   | "libQnnGpu.so" |     inference with GPU      |
|   "model.so"   | "libQnnHtp.so" |     inference with HTP      |
|   "model.so"   | "libQnnDsp.so" |     inference with DSP      |
|  "model.bin"   | "libQnnHtp.so" |     inference with HTP      |

<h3 id="2.2">inference_execute - <a href="https://github.com/quic-qrb-ros/qrb_ros_nn_inference/blob/main/qrb_inference_manager/src/qrb_inference_manager.cpp#L45">[source]</a></h1>

```cpp
/**
 * \brief execute model inference
 * \param input_tensor_data input data for model inference
 * \return true of false
 */
bool inference_execute(const std::vector<uint8_t> &input_tensor_data);
```

<h3 id="2.3">get_output_tensors - <a href="https://github.com/quic-qrb-ros/qrb_ros_nn_inference/blob/main/qrb_inference_manager/src/qrb_inference_manager.cpp#L53">[source]</a></h1>

```cpp
/**
 * \brief get the result of inference
 * \return result
 */
std::vector<OutputTensor> get_output_tensors();
```
