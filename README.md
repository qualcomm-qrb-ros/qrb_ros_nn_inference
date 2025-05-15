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

1. please follow [steps](https://github.com/qualcomm-qrb-ros/qrb_ros_docker?tab=readme-ov-file#quickstart) to setup docker env.

2. download the qrb_ros_nn_inference and dependencies

```bash
    cd ${QRB_ROS_WS}/src && \
    git clone https://github.com/qualcomm-qrb-ros/qrb_ros_interfaces && \
    git clone https://github.com/qualcomm-qrb-ros/qrb_ros_nn_inference
```

3. build qrb_ros_nn_inference

```bash
    cd ${QRB_ROS_WS} && \
    colcon build --packages-up-to qrb_ros_nn_inference
```

4. test qrb_ros_nn_inference with YOLOv8 detection model

    4.1 download yolov8.tflite model by following [QC AI hub Getting Started](https://app.aihub.qualcomm.com/docs/hub/getting_started.html).

    4.2 download the test image for object detecion

    ```bash
    wget -P ${QRB_ROS_WS}/src/qrb_ros_nn_inference/test/qrb_ros_pre_process/image/ \
    https://ultralytics.com/images/bus.jpg && \
    python3 ${QRB_ROS_WS}/src/qrb_ros_nn_inference/test/qrb_ros_post_process/scripts/yolov8_input_pre_process.py
    ```

    4.3 point out the raw image path and model path in `${QRB_ROS_WS}/src/qrb_ros_nn_inference/test/qrb_ros_post_process/launch/nn_node_test.launch.py`

    ```python
    pre_process_node = ComposableNode(
       package = "qrb_ros_pre_process",
       plugin = "qrb_ros::pre_process::QrbRosPreProcessNode",
       name = "pre_process_node",
       parameters=[
         {
           "image_path": os.environ['QRB_ROS_WS']+"/src/qrb_ros_nn_inference/test/qrb_ros_pre_process/image/bus.raw"
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
      colcon build --packages-select qrb_ros_pre_process qrb_ros_post_process
    ```

    4.5 execute the inference

    ```bash
      cd ${QRB_ROS_WS}/ && \
      source install/local_setup.bash && \
      ros2 launch qrb_ros_post_process nn_node_test.launch.py
    ```

    4.6 visualize the detection result

    ```bash
      python3 ${QRB_ROS_WS}/src/qrb_ros_nn_inference/test/qrb_ros_post_process/scripts/qrb_ros_yolo_detection_visualizer.py \
      --original_image ${QRB_ROS_WS}/src/qrb_ros_nn_inference/test/qrb_ros_pre_process/image/image.jpg
    ```

    reulst image will be stroed in `${QRB_ROS_WS}/src/qrb_ros_nn_inference/test/qrb_ros_post_process/inference_result`

</details>


<details>
<summary>Cross Compilation with QIRP SDK</summary>

1. please follow [steps](https://qualcomm-qrb-ros.github.io/getting_started/index.html) to setup qirp-sdk env.

2. clone this repository and dependencies

    ```bash
        mdkir -p <qirp_decompressed_workspace>/qirp-sdk/ros_ws/src && \
        cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws/src && \
        git clone https://github.com/qualcomm-qrb-ros/qrb_ros_interfaces && \
        git clone https://github.com/qualcomm-qrb-ros/qrb_ros_nn_inference
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

5. install the qrb_ros_nn_inference package on your device:

    ```bash
        cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws/install/qrb_ros_nn_inference && \
        tar -czvf qrb_ros_nn_inference.tar.gz include lib share && \
        scp qrb_ros_nn_inference.tar.gz root@[ip-addr]:/opt && \
        cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws/install/qrb_inference_manager && \
        tar -czvf qrb_inference_manager.tar.gz include lib share && \
        scp qrb_inference_manager.tar.gz root@[ip-addr]:/opt && \
        cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws/install/qrb_ros_tensor_list_msgs && \
        tar -czvf qrb_ros_tensor_list_msgs.tar.gz include lib share && \
        scp qrb_ros_tensor_list_msgs.tar.gz root@[ip-addr]:/opt
    ```

    ```bash
        ssh root@[ip-addr]
        (ssh) mount -o remount rw /usr
        (ssh) tar --no-overwrite-dir --no-same-owner -zxf /opt/qrb_ros_tensor_list_msgs.tar.gz -C /usr/
        (ssh) tar --no-overwrite-dir --no-same-owner -zxf /opt/qrb_inference_manager.tar.gz -C /usr/
        (ssh) tar --no-overwrite-dir --no-same-owner -zxf /opt/qrb_ros_nn_inference.tar.gz -C /usr/
    ```

5. source this file to set up the environment on your device:

    ```bash
        ssh root@[ip-addr]
        (ssh) export HOME=/opt
        (ssh) source /usr/bin/ros_setup.bash
        (ssh) export ROS_DOMAIN_ID=xx
    ```

6. launch your inference pipeline

    ```bash
        (ssh) ros2 launch ${package_name} ${launch-file}
    ```

</details>

You can get more details from [QRB ROS Documentation](https://qualcomm-qrb-ros.github.io/main/packages/qrb_ros_nn_inference/index.html).

## 🙏 Contributing

We so much would love to have you as a part of the QRB ROS community. Whether you are helping us fix bugs, proposing new features, improving our documentation, or spreading the word, please refer to our [contribution guidelines](./CONTRIBUTING.md) and [code of conduct](./CODE_OF_CONDUCT.md).

- Bug report: If you see an error message or encounter failures, please create a [bug report](../../issues)
- Feature Request: If you have an idea or if there is a capability that is missing and would make development easier and more robust, please submit a [feature request](../../issues)

<Update link with template>

## 👨‍💻 Authors

* **Na Song** - *Initial work* - [@nasongCool](https://github.com/nasongCool)

See also the list of [contributors](https://github.com/qualcomm-qrb-ros/qrb_ros_nn_inference/graphs/contributors) who participated in this project.

## 📃 License

Project is licensed under the [BSD-3-clause License](https://spdx.org/licenses/BSD-3-Clause.html). See [LICENSE](./LICENSE) for the full license text.
