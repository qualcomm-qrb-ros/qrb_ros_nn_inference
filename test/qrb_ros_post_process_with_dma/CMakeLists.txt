cmake_minimum_required(VERSION 3.8)
project(qrb_ros_post_process_with_dma)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(qrb_ros_tensor_list_msgs REQUIRED)

add_library(qrb_ros_post_process_with_dma_node SHARED
  src/qrb_ros_post_process_with_dma_node.cpp
)
target_compile_features(qrb_ros_post_process_with_dma_node PUBLIC cxx_std_17)
target_include_directories(qrb_ros_post_process_with_dma_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(qrb_ros_post_process_with_dma_node
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  cv_bridge
  qrb_ros_tensor_list_msgs
)

rclcpp_components_register_nodes(qrb_ros_post_process_with_dma_node "qrb_ros_post_process_with_dma::QrbRosPostProcessWithDmaNode")

add_executable(qrb_ros_post_process_with_dma_node_exe
  src/qrb_ros_post_process_with_dma_node_main.cpp
)
target_compile_features(qrb_ros_post_process_with_dma_node_exe PUBLIC cxx_std_17)
target_include_directories(qrb_ros_post_process_with_dma_node_exe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(qrb_ros_post_process_with_dma_node_exe qrb_ros_post_process_with_dma_node)
ament_target_dependencies(qrb_ros_post_process_with_dma_node_exe
  rclcpp
  rclcpp_components
)

install(TARGETS qrb_ros_post_process_with_dma_node qrb_ros_post_process_with_dma_node_exe
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
