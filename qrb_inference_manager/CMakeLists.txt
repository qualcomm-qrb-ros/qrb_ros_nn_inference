# this file is for building libqrb_inference_manager.so
cmake_minimum_required(VERSION 3.8)
project(qrb_inference_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(QNN_SDK_ROOT /opt/QnnSDK/2.22.10.240618)
set(QNN_SDK ${QNN_SDK_ROOT}/include/QNN/)
set(QNN_SampleApp ${QNN_SDK_ROOT}/examples/QNN/SampleApp/src)

include_directories(
  ${QNN_SDK}
  ${QNN_SampleApp}
  ${QNN_SampleApp}/PAL/include/
  ${QNN_SampleApp}/Log
  ${QNN_SampleApp}/Utils
  ${QNN_SampleApp}/WrapperUtils
  include
)

set(QNN_HELPER_SOURCES
  ${QNN_SampleApp}/PAL/src/common/GetOpt.cpp
  ${QNN_SampleApp}/PAL/src/common/StringOp.cpp
  ${QNN_SampleApp}/PAL/src/linux/Directory.cpp
  ${QNN_SampleApp}/PAL/src/linux/DynamicLoading.cpp
  ${QNN_SampleApp}/PAL/src/linux/FileOp.cpp
  ${QNN_SampleApp}/PAL/src/linux/Path.cpp
  ${QNN_SampleApp}/Log/Logger.cpp
  ${QNN_SampleApp}/Log/LogUtils.cpp
  ${QNN_SampleApp}/Utils/DataUtil.cpp
  ${QNN_SampleApp}/Utils/DynamicLoadUtil.cpp
  ${QNN_SampleApp}/Utils/IOTensor.cpp
  ${QNN_SampleApp}/Utils/QnnSampleAppUtils.cpp
  ${QNN_SampleApp}/WrapperUtils/QnnWrapperUtils.cpp
)

set(SOURCES
  src/qrb_inference_manager.cpp
  src/qnn_inference/qnn_inference.cpp
  src/qnn_inference/qnn_inference_impl.cpp
  src/qnn_delegate_inference/qnn_delegate_inference.cpp
)

link_directories($ENV{SDK_TOP_DIR}/toolchain/install_dir/sysroots/armv8-2a-qcom-linux/lib/aarch64-oe-linux-gcc11.2)
ament_auto_add_library(qrb_inference_manager SHARED ${QNN_HELPER_SOURCES} ${SOURCES})
target_link_libraries(qrb_inference_manager tensorflowlite_c QnnTFLiteDelegate)

ament_auto_package()