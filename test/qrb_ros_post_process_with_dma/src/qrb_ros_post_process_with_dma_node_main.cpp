#include "qrb_ros_post_process_with_dma/qrb_ros_post_process_with_dma_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<qrb_ros_post_process_with_dma::QrbRosPostProcessWithDmaNode>(
      rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
