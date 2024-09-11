#!/bin/bash

HOST_LIBS=()

HOST_LIB_PATH="/usr/lib"
CONTAINER_LIB_PATH="/usr/local/lib"

# libs for QNN TFLite GPU delegate
HOST_LIBS+=("-v ${HOST_LIB_PATH}/libadreno_utils.so:${CONTAINER_LIB_PATH}/libadreno_utils.so")
HOST_LIBS+=("-v ${HOST_LIB_PATH}/libCB.so:${CONTAINER_LIB_PATH}/libCB.so")
HOST_LIBS+=("-v ${HOST_LIB_PATH}/libOpenCL.so:${CONTAINER_LIB_PATH}/libOpenCL.so")
HOST_LIBS+=("-v ${HOST_LIB_PATH}/libOpenCL_adreno.so:${CONTAINER_LIB_PATH}/libOpenCL_adreno.so")
HOST_LIBS+=("-v ${HOST_LIB_PATH}/libdmabufheap.so.0:${CONTAINER_LIB_PATH}/libdmabufheap.so.0")
HOST_LIBS+=("-v ${HOST_LIB_PATH}/libdmabufheap.so.0.0.0:${CONTAINER_LIB_PATH}/libdmabufheap.so.0.0.0")
HOST_LIBS+=("-v ${HOST_LIB_PATH}/libgsl.so:${CONTAINER_LIB_PATH}/libgsl.so")
HOST_LIBS+=("-v ${HOST_LIB_PATH}/libpropertyvault.so.0:${CONTAINER_LIB_PATH}/libpropertyvault.so.0")
HOST_LIBS+=("-v ${HOST_LIB_PATH}/libpropertyvault.so.0.0.0:${CONTAINER_LIB_PATH}/libpropertyvault.so.0.0.0")
HOST_LIBS+=("-v ${HOST_LIB_PATH}/libllvm-qcom.so:${CONTAINER_LIB_PATH}/libllvm-qcom.so")

docker run -it --rm \
	--privileged \
  --network host \
  -v /home/qrb_ros_ws/:/workspace/qrb_ros_ws \
  ${HOST_LIBS[@]} \
  --name "qrb_ros_container" \
  --hostname "qrb_ros" \
  --workdir /workspace/qrb_ros_ws \
  qrb_ros:latest