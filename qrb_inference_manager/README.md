# Introduction

![Static Badge](https://img.shields.io/badge/language-Cpp-green)
![Static Badge](https://img.shields.io/badge/executor-CPU_GPU_HTP-orange)

qrb_inference_manager is a wrapper library of the QNN SDK and QNN Delegate SDK.<br>

qrb_inference_manager provides a simple and user-friendly interface, supporting 🚀**real-time** inference for models in so, tflite, and bin formats.

# API Documentation
[Documentation](./Documentation.md)

# Dependency

### Cpp version

qrb_inference_manager use the feature of C17, please ensure your Cpp environment meets requirements.

### QNN SDK

> the version of QNN SDK should ideally be greater than or equal to 2.24.1.240626

download QNN SDK:

```bash
export QNN_SDK_VER="2.27.0.240926" && wget -P /opt/qcom https://softwarecenter.qualcomm.com/api/download/software/qualcomm_neural_processing_sdk/v${QNN_SDK_VER}.zip
```

copy the dependency:

> QNN libs are differentiated based on the build toolchains version, please refer to this [wiki](https://docs.qualcomm.com/bundle/publicresource/topics/80-63442-50/overview.html#supported-snapdragon-devices) to select correct version

```bash
export QNN_GCC_VER="aarch64-oe-linux-gcc11.2"
cd /opt/qcom && unzip v${QNN_SDK_VER}.zip -d /opt/qcom/qnn_sdk_v${QNN_SDK_VER} && rm -rf v${QNN_SDK_VER}.zip && \
cp -R /opt/qcom/qnn_sdk_v${QNN_SDK_VER}/qairt/${QNN_SDK_VER}/lib/${QNN_GCC_VER}/* /usr/local/lib && \
cp -R /opt/qcom/qnn_sdk_v${QNN_SDK_VER}/qairt/${QNN_SDK_VER}/include/QNN/* /usr/local/include && \
rm -rf /opt/qcom/qnn_sdk_v${QNN_SDK_VER} && \
```

## TensorFlow

> the version of TensorFlow should ideally be greater than or equal to 2.15.0

download and build TensorFlow source code:

```bash
git clone --branch v2.16.1 https://github.com/tensorflow/tensorflow.git /opt/tensorflow && \
export TENSORFLOW_SOURCE_DIR="/opt/tensorflow" && cd $TENSORFLOW_SOURCE_DIR && \
mkdir tflite-build && cd tflite-build && \
cmake ../tensorflow/lite/c && \
cmake --build . -j8 && \
cp ./libtensorflowlite_c.so /usr/local/lib
```

# Use qrb_inference_manager in your C++ project

add these lines in your CMakeLists.txt

```cmake
# set the env of qrb_inference_manager source code
if(NOT DEFINED ENV{QRB_INFERENCE_MANAGER_SOURCE_DIR})
  set(ENV{QRB_INFERENCE_MANAGER_SOURCE_DIR} "/path/to/qrb_inference_manager")
endif()

add_subdirectory(
  "$ENV{QRB_INFERENCE_MANAGER_SOURCE_DIR}"
  "${CMAKE_CURRENT_BINARY_DIR}/qrb_inference_manager" EXCLUDE_FROM_ALL)

include_directories(
  /path/to/qrb_inference_manager/include
)
```
