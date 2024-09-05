FROM ros:humble

LABEL maintainer="Na Song <quic_nasong@quicinc.com>"
LABEL version="1.0"
LABEL description="this docker file is for testing QUIC_QRB_ROS on RB3 gen2 LE, more details: https://github.com/quic-qrb-ros"

ENV QNN_SDK_VER="2.22.10.240618"
ENV QNN_HEXAGON_VER="hexagon-v68"
ENV OPENCV_VER="4.9.0"
ENV TensorFlow_VER="2.15.0"
ENV QNN_GCC_VER="aarch64-oe-linux-gcc11.2"
ENV LD_LIBRARY_PATH=/usr/local/lib

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"]

RUN --mount=type=cache,target=/var/cache/apt \
		apt-get update && \
		apt-get install -y \
		git \
		gcc \
		g++ \
		cmake \
		wget \
		unzip

# download qnn sdk
RUN wget -P /opt/qcom https://softwarecenter.qualcomm.com/api/download/software/qualcomm_neural_processing_sdk/v${QNN_SDK_VER}.zip
RUN cd /opt/qcom && unzip v${QNN_SDK_VER}.zip -d /opt/qcom/qnn_sdk_v${QNN_SDK_VER} && rm -rf v${QNN_SDK_VER}.zip && \
		cp -R /opt/qcom/qnn_sdk_v${QNN_SDK_VER}/qairt/${QNN_SDK_VER}/lib/${QNN_GCC_VER}/* /usr/local/lib && \
		cp -R /opt/qcom/qnn_sdk_v${QNN_SDK_VER}/qairt/${QNN_SDK_VER}/bin/${QNN_GCC_VER}/* /usr/local/bin && \
		cp -R /opt/qcom/qnn_sdk_v${QNN_SDK_VER}/qairt/${QNN_SDK_VER}/lib/${QNN_HEXAGON_VER}/unsigned/* /usr/local/lib && \
		cp -R /opt/qcom/qnn_sdk_v${QNN_SDK_VER}/qairt/${QNN_SDK_VER}/include/QNN/* /usr/local/include && \
    rm -rf /opt/qcom/qnn_sdk_v${QNN_SDK_VER}

# download TensorFlow
RUN git clone --branch v${TensorFlow_VER} https://github.com/tensorflow/tensorflow.git /opt/tensorflow-${TensorFlow_VER}

# download OpenCV and build
RUN git clone --depth=1 --branch ${OPENCV_VER} https://github.com/opencv/opencv.git /opt/opencv-${OPENCV_VER} && \
		mkdir -p /opt/opencv-${OPENCV_VER}/build && cd /opt/opencv-${OPENCV_VER}/build && \
		cmake ..; cmake --build . -j8; cmake --install . && \
		mv /usr/local/include/opencv4/opencv2/ /usr/local/include/; rm -rf /usr/local/include/opencv4/ && \
		rm -rf /opt/opencv-${OPENCV_VER}

# install qrb_ros fundamentals
RUN --mount=type=cache,target=/var/cache/apt \
		apt-get install -y \
		python3-pip \
		python3-rosdep

# install the dependency of qrb_ros_nn_inference
RUN python3 -m pip install -U \
		opencv-python

WORKDIR /home/qrb_ros_ws/
