# QRB ROS NN Inference

## Overview

qrb_ros_nn_inference is a ros2 package for performing neural network model, providing AI-based perception for robotics applications. qrb_ros_nn_inference
offers hardware acceleration based on Qualcomm platforms, utilizing pre-trained models from the Qualcomm AI Hub to receive input tensor and output the prediction to output tensor.

## System Requirements

- Version of TFLite should be higher than 2.11.1
- ROS 2 Humble and later.

## QuickStart

### 1. Create your ROS2 workspace for QRB ROS

```bash
adb root;adb shell "mkdir -p /home/qrb_ros_ws/src"
```

### 2. Download the qrb_ros_docker and push to your device

```bash
git clone https://github.com/quic-qrb-ros/qrb_ros_nn_inference
```

```bash
adb push qrb_ros_nn_inference /home/qrb_ros_ws/src/
```

### 3. Build and Launch the Docker container on your device

```bash
adb shell
```

```bash
cd /home/qrb_ros_ws/src/qrb_ros_nn_inference/docker/scripts && \
bash docker_build.sh && \
bash docker_run.sh
```

### 4. Clone the QRB_ROS repository in docker container

```bash
export QRB_ROS_WS=/workspace/qrb_ros_ws && \
cd ${QRB_ROS_WS}/src && \
git clone https://github.com/quic-qrb-ros/qrb_ros_nn_inference && \
git clone https://github.com/quic-qrb-ros/qrb_ros_tensor_list_msgs
```

### 5. Build qrb_ros_nn_inference

```bash
cd ${QRB_ROS_WS}/ && \
colcon build --packages-up-to qrb_ros_nn_inference
```

### 6. Test qrb_ros_nn_inference with YOLOv8 detection model

1. [download model](https://aihub.qualcomm.com/iot/models/yolov8_det?domain=Computer+Vision&useCase=Object+Detection)

2. download the test image for object detecion

   ```
   wget -O \
   ${QRB_ROS_WS}/src/qrb_ros_nn_inference/test/qrb_ros_pre_process/image/image.jpg \
   https://ultralytics.com/images/bus.jpg
   ```

3. point out the image path and model path in ${QRB_ROS_WS}/src/qrb_ros_nn_inference/test/qrb_ros_post_process/launch/nn_node_test.launch.py

   ```python
       pre_process_node = ComposableNode(
           package = "qrb_ros_pre_process",
           plugin = "qrb_ros::pre_process::QrbRosPreProcessNode",
           name = "pre_process_node",
           parameters=[
             {
               "image_path": "/path/to/image" # point out the path to image
             }
           ]
       )

       nn_inference_node = ComposableNode(
           package = "qrb_ros_nn_inference",
           plugin = "qrb_ros::nn_inference::QrbRosInferenceNode",
           name = "nn_inference_node",
           parameters=[
             {
               "backend_option": "",
               "model_path": "/path/to/model" # point out the path to model
             }
           ]
       )
   ```

4. build the pre and post process packages

   ```bash
   cd ${QRB_ROS_WS}/ && \
   colcon build --symlink-install --packages-select qrb_ros_pre_process qrb_ros_post_process
   ```

5. execute the inference

   ```bash
   cd ${QRB_ROS_WS}/ && \
   source install/local_setup.bash && \
   ros2 launch qrb_ros_post_process nn_node_test.launch.py
   ```

6. visualize the detection result

   ```bash
   python3 ./src/qrb_ros_nn_inference/test/qrb_ros_post_process/scripts/qrb_ros_yolo_detection_visualizer.py \
   --original_image ${QRB_ROS_WS}/src/qrb_ros_nn_inference/test/qrb_ros_pre_process/image/image.jpg
   ```

   reulst image will be stroed in `${QRB_ROS_WS}/src/qrb_ros_nn_inference/test/qrb_ros_post_process/inference_result`

## Resources

- [Qualcomm AI Engine Direct](https://docs.qualcomm.com/bundle/publicresource/topics/80-63442-50/introduction.html)
- [Qualcomm AI Hub](https://aihub.qualcomm.com/)

## Contributions

Thanks for your interest in contributing to qrb_ros_nn_inference! Please read our [Contributions Page](CONTRIBUTING.md) for more information on contributing features or bug fixes. We look forward to your participation!

## License

qrb_ros_nn_inference is licensed under the BSD 3-clause "New" or "Revised" License.

Check out the [LICENSE](LICENSE) for more details.
