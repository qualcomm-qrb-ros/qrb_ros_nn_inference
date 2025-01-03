cmake_minimum_required(VERSION 3.8)
project(qrb_ros_post_process)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  ./include
)

ament_auto_add_library(qrb_ros_post_process_node SHARED src/qrb_ros_post_process_node.cpp)
target_link_libraries(qrb_ros_post_process_node)

rclcpp_components_register_nodes(
  qrb_ros_post_process_node
  PLUGIN "qrb_ros::post_process::QrbRosPostProcessNode")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
