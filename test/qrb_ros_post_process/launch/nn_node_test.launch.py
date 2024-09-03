import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
  pre_process_node = ComposableNode(
    package = "qrb_ros_pre_process",
    plugin = "qrb_ros::pre_process::QrbRosPreProcessNode",
    name = "pre_process_node",
  )

  nn_inference_node = ComposableNode(
    package = "qrb_ros_nn_inference",
    plugin = "qrb_ros::nn_inference::QrbRosInferenceNode",
    name = "nn_inference_node",
    parameters=[
      {
        "backend_option": "",
        "model_path": "/path/to/model"
      }
    ]
  )

  post_precess_node = ComposableNode(
    package = "qrb_ros_post_process",
    plugin = "qrb_ros::post_process::QrbRosPreProcessNode",
    name = "post_process_node"
  )

  container = ComposableNodeContainer(
    name = "container",
    namespace = "nasong_test",
    package = "rclcpp_components",
    executable='component_container',
    output = "screen",
    composable_node_descriptions=[pre_process_node, nn_inference_node, post_precess_node]
  )

  return launch.LaunchDescription([container])