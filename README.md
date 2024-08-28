# QRB ROS NN Inference

## Overview

qrb_ros_nn_inference is a ros2 package for performing neural network model, providing AI-based perception for robotics applications. qrb_ros_nn_inference
offers hardware acceleration based on Qualcomm platforms, utilizing pre-trained models from the Qualcomm AI Hub to receive input tensor and output the prediction to output tensor.

## System Requirements

- Version of TFLite should be higher than 2.11.1
- ROS 2 Humble and later.

## Quickstart

> qrb_ros_nn_inference node is for model inference, it can not run alone
>
> User need to prepare pre-process node for provideing input data for model inference and post-process node for prase the model output

### Build

1. Setup environments follow this document 's [Set up the cross-compile environment.](https://docs.qualcomm.com/bundle/publicresource/topics/80-65220-2/develop-your-first-application_6.html?product=1601111740013072&facet=Qualcomm%20Intelligent%20Robotics%20(QIRP)%20Product%20SDK&state=releasecandidate) part

2. Create `ros_ws` directory in `<qirp_decompressed_workspace>/qirp-sdk/`

3. Clone your pre-process node and post-process node under `<qirp_decompressed_workspace>/qirp-sdk/ros_ws`

   ```bash
   cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws && \
   git clone https://github.com/my_github/pre-process_node.git && \
   git clone https://github.com/my_github/post-process_node.git
   ```

4. Clone this repository and dependencies under `<qirp_decompressed_workspace>/qirp-sdk/ros_ws`

   ```bash
   cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws && \
   git clone https://github.com/quic-qrb-ros/qrb_ros_tensor_list_msgs.git && \
   git clone https://github.com/quic-qrb-ros/qrb_ros_nn_inference.git
   ```

5. download the qnn sdk

   ```bash
   mkdir -p /opt/QnnSDK && \
   wget -P /opt/QnnSDK https://softwarecenter.qualcomm.com/api/download/software/qualcomm_neural_processing_sdk/v2.22.10.240618.zip && \
   unzip v2.22.10.240618.zip -d /opt/QnnSDK
   ```

6. build your pipeline

   ```bash
   export AMENT_PREFIX_PATH="${OECORE_TARGET_SYSROOT}/usr;${OECORE_NATIVE_SYSROOT}/usr"
   export PYTHONPATH=${PYTHONPATH}:${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages

   colcon build --merge-install --cmake-args \
     -DPython3_ROOT_DIR=${OECORE_TARGET_SYSROOT}/usr \
     -DPython3_NumPy_INCLUDE_DIR=${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages/numpy/core/include \
     -DPYTHON_SOABI=cpython-310-aarch64-linux-gnu -DCMAKE_STAGING_PREFIX=$(pwd)/install \
     -DCMAKE_PREFIX_PATH=$(pwd)/install/share \
     -DBUILD_TESTING=OFF
   ```

7. Push to the device & Install

   ```bash
   cd `<qirp_decompressed_workspace>/qirp-sdk/ros_ws/install`
   tar czvf qrb_ros_nn_inference.tar.gz lib share
   scp qrb_ros_nn_inferenceu.tar.gz root@[ip-addr]:/opt/
   ssh root@[ip-addr]
   (ssh) tar -zxf /opt/qrb_ros_nn_inference.tar.gz -C /opt/qcom/qirp-sdk/usr/
   ```

### Run

1. Source this file to set up the environment on your device:

   ```bash
   ssh root@[ip-addr]
   (ssh) export HOME=/opt
   (ssh) source /opt/qcom/qirp-sdk/qirp-setup.sh
   (ssh) export ROS_DOMAIN_ID=xx
   (ssh) source /usr/bin/ros_setup.bash
   ```

2. use launch file to run your inference pipeline

   ```bash
   (ssh) ros2 launch ${package_name} ${launch-file}
   ```

## Resources

- [Qualcomm AI Engine Direct](https://docs.qualcomm.com/bundle/publicresource/topics/80-63442-50/introduction.html)
- [Qualcomm AI Hub](https://aihub.qualcomm.com/)

## Contributions

Thanks for your interest in contributing to qrb_ros_nn_inference! Please read our [Contributions Page](CONTRIBUTING.md) for more information on contributing features or bug fixes. We look forward to your participation!

## License

qrb_ros_nn_inference is licensed under the BSD 3-clause "New" or "Revised" License.

Check out the [LICENSE](LICENSE) for more details.
