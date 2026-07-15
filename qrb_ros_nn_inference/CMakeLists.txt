cmake_minimum_required(VERSION 3.8)
project(qrb_ros_nn_inference)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  ./include
  /usr/include/QNN
)

ament_auto_add_library(qrb_ros_inference_node SHARED
  src/qrb_ros_inference_node.cpp
  src/qrb_ros_shared_inference_node.cpp)

# Detect whether the installed qrb_ros_tensor_list_msgs Tensor message has dmabuf fields.
# Older versions (1.0.0) do not; newer versions do.
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_INCLUDES ${qrb_ros_tensor_list_msgs_INCLUDE_DIRS})
set(CMAKE_REQUIRED_FLAGS "-std=c++17")
check_cxx_source_compiles("
  #include <qrb_ros_tensor_list_msgs/msg/tensor.hpp>
  int main() {
    qrb_ros_tensor_list_msgs::msg::Tensor t;
    (void)t.dmabuf_fd;
    return 0;
  }
" QRB_TENSOR_HAS_DMABUF_FIELD)
if(QRB_TENSOR_HAS_DMABUF_FIELD)
  target_compile_definitions(qrb_ros_inference_node PRIVATE QRB_TENSOR_HAS_DMABUF)
endif()

rclcpp_components_register_nodes(
  qrb_ros_inference_node
  PLUGIN "qrb_ros::nn_inference::QrbRosInferenceNode")

rclcpp_components_register_nodes(
  qrb_ros_inference_node
  PLUGIN "qrb_ros::nn_inference::QrbRosSharedInferenceNode")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
