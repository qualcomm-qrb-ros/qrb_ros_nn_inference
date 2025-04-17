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
)

ament_auto_add_library(qrb_ros_inference_node SHARED src/qrb_ros_inference_node.cpp)

rclcpp_components_register_nodes(
  qrb_ros_inference_node
  PLUGIN "qrb_ros::nn_inference::QrbRosInferenceNode")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
