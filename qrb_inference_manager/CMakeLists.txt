cmake_minimum_required(VERSION 3.8)
project(qrb_inference_manager VERSION 2.0.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(TENSORFLOW_SOURCE_DIR "/path/to/tensorflow_source_code" CACHE PATH)
if(NOT TENSORFLOW_SOURCE_DIR)
  get_filename_component(TENSORFLOW_SOURCE_DIR
    "${CMAKE_CURRENT_LIST_DIR}/../../../../" ABSOLUTE)
endif()

add_subdirectory(
  "${TENSORFLOW_SOURCE_DIR}/tensorflow/lite"
  "${CMAKE_CURRENT_BINARY_DIR}/tensorflow-lite" EXCLUDE_FROM_ALL)

include_directories(
  include
)

set(SOURCES
  src/qrb_inference_manager.cpp
  src/qnn_inference/qnn_inference.cpp
  src/qnn_inference/qnn_inference_impl.cpp
  src/qnn_delegate_inference/qnn_delegate_inference.cpp
)

ament_auto_add_library(qrb_inference_manager SHARED ${SOURCES})
target_link_libraries(qrb_inference_manager tensorflow-lite QnnTFLiteDelegate)

set_target_properties(
  qrb_inference_manager PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION 2
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
