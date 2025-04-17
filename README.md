# QRB ROS NN Inference
![Static Badge](https://img.shields.io/badge/language-Cpp-green)
![Static Badge](https://img.shields.io/badge/executor-CPU_GPU_HTP-orange)
![Static Badge](https://img.shields.io/badge/release-v0.2.0-blue)

🎉 Explore your AI robot applications on Qualcomm devices!

## 🙋‍♂️ Overview

**qrb_ros_nn_inference** is a ROS2 package for performing neural network model, providing 🤖AI-based perception for robotics applications.<br>

qrb_ros_nn_inference support:
- 🚀hardware acceleration based on Qualcomm platforms
- ✨three kinds of model format: **.tflite**, **.so**, **.bin**

## 🎬 Quick Start

> Quick start shown here is always latest.

We provide two ways for running the QRB ROS packages on QCOM Linux platform.

<details>
<summary>On-Device Compilation with Docker</summary>

1. please follow [steps](https://github.com/quic-qrb-ros/qrb_ros_docker?tab=readme-ov-file#quickstart) to setup docker env.

2. download the qrb_ros_nn_inference and dependencies

```bash
    cd ${QRB_ROS_WS}/src && \
    git clone https://github.com/quic-qrb-ros/qrb_ros_tensor_list_msgs.git && \
    git clone https://github.com/quic-qrb-ros/qrb_ros_nn_inference.git
```

3. build qrb_ros_nn_inference

```bash
    cd ${QRB_ROS_WS} && \
    colcon build --packages-up-to qrb_ros_nn_inference
```

4. test qrb_ros_nn_inference with YOLOv8 detection model

    4.1 download model from [QC AI hub](https://aihub.qualcomm.com/iot/models/yolov8_det?domain=Computer+Vision&useCase=Object+Detection).

    4.2 download the test image for object detecion

    ```bash
    wget -P \
    ${QRB_ROS_WS}/src/qrb_ros_nn_inference/test/qrb_ros_pre_process/image/ \
    https://ultralytics.com/images/bus.jpg
    ```

    4.3 point out the image path and model path in `${QRB_ROS_WS}/src/qrb_ros_nn_inference/test/qrb_ros_post_process/launch/nn_node_test.launch.py`

    ```python
    pre_process_node = ComposableNode(
       package = "qrb_ros_pre_process",
       plugin = "qrb_ros::pre_process::QrbRosPreProcessNode",
       name = "pre_process_node",
       parameters=[
         {
           "image_path": os.environ['QRB_ROS_WS']+"/src/qrb_ros_nn_inference/test/qrb_ros_pre_process/image/bus.jpg"
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
           "model_path": "/path/to/model"
         }
       ]
    )
    ```

    4.4 build the pre and post process packages

    ```bash
      cd ${QRB_ROS_WS}/ && \
      rm ./src/qrb_ros_nn_inference/test/qrb_ros_post_process/COLCON_IGNORE && \
      rm ./src/qrb_ros_nn_inference/test/qrb_ros_pre_process/COLCON_IGNORE && \
      colcon build --symlink-install --packages-select qrb_ros_pre_process qrb_ros_post_process
    ```

    4.5 execute the inference

    ```bash
      cd ${QRB_ROS_WS}/ && \
      source install/local_setup.bash && \
      ros2 launch qrb_ros_post_process nn_node_test.launch.py
    ```

    4.6 visualize the detection result

    ```bash
      python3 ./src/qrb_ros_nn_inference/test/qrb_ros_post_process/scripts/qrb_ros_yolo_detection_visualizer.py \
      --original_image ${QRB_ROS_WS}/src/qrb_ros_nn_inference/test/qrb_ros_pre_process/image/image.jpg
    ```

    reulst image will be stroed in `${QRB_ROS_WS}/src/qrb_ros_nn_inference/test/qrb_ros_post_process/inference_result`

</details>


<details>
<summary>Cross Compilation with QIRP SDK</summary>

1. please follow [steps](https://quic-qrb-ros.github.io/getting_started/index.html) to setup qirp-sdk env.

2. clone this repository and dependencies

    ```bash
        mdkir -p <qirp_decompressed_workspace>/qirp-sdk/ros_ws/src && \
        cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws/src && \
        git clone https://github.com/quic-qrb-ros/qrb_ros_tensor_list_msgs.git && \
        git clone https://github.com/quic-qrb-ros/qrb_ros_nn_inference.git
    ```

3. prepare your pre and post process node

4. colcon build your pipeline:

    ```bash
      cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws && \
      colcon build --cmake-args \
        -DPYTHON_EXECUTABLE=${OECORE_NATIVE_SYSROOT}/usr/bin/python3 \
        -DPython3_NumPy_INCLUDE_DIR=${OECORE_NATIVE_SYSROOT}/usr/lib/python3.12/site-packages/numpy/core/include \
        -DPYTHON_SOABI=cpython-312-aarch64-linux-gnu \
        -DCMAKE_MAKE_PROGRAM=/usr/bin/make \
        -DCMAKE_LIBRARY_PATH=${OECORE_TARGET_SYSROOT}/usr/lib \
        -DBUILD_TESTING=OFF \
        -DCMAKE_TOOLCHAIN_FILE=${OE_CMAKE_TOOLCHAIN_FILE}
    ```

5. source this file to set up the environment on your device:

    ```bash
        ssh root@[ip-addr]
        (ssh) export HOME=/opt
        (ssh) source /opt/qcom/qirp-sdk/qirp-setup.sh
        (ssh) export ROS_DOMAIN_ID=xx
        (ssh) source /usr/bin/ros_setup.bash
    ```

6. launch your inference pipeline

    ```bash
        (ssh) ros2 launch ${package_name} ${launch-file}
    ```

</details>

You can get more details from [QRB ROS Documentation](https://quic-qrb-ros.github.io/main/packages/qrb_ros_nn_inference/index.html).

## 🙏 Contributing

We so much would love to have you as a part of the QRB ROS community. Whether you are helping us fix bugs, proposing new features, improving our documentation, or spreading the word, please refer to our [contribution guidelines](./CONTRIBUTING.md) and [code of conduct](./CODE_OF_CONDUCT.md).

- Bug report: If you see an error message or encounter failures, please create a [bug report](../../issues)
- Feature Request: If you have an idea or if there is a capability that is missing and would make development easier and more robust, please submit a [feature request](../../issues)

<Update link with template>

## 👨‍💻 Authors

* **Na Song** - *Initial work* - [@nasongCool](https://github.com/nasongCool)

See also the list of [contributors](https://github.com/quic-qrb-ros/qrb_ros_nn_inference/graphs/contributors) who participated in this project.

## 📃 License

Project is licensed under the [BSD-3-clause License](https://spdx.org/licenses/BSD-3-Clause.html). See [LICENSE](./LICENSE) for the full license text.
