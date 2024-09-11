#!/bin/bash

export DOCKER_BUILDKIT=1
export COMPOSE_DOCKER_CLI_BUILD=1

# dependency version
QNN_SDK_VER="2.22.10.240618"
OPENCV_VER="4.9.0"
TensorFlow_VER="2.15.0"

BUILD_ARG=()

BUILD_ARG+=("--build-arg QNN_SDK_VER=${QNN_SDK_VER}")
BUILD_ARG+=("--build-arg OPENCV_VER=${OPENCV_VER}")
BUILD_ARG+=("--build-arg TensorFlow_VER=${TensorFlow_VER}")

docker build ${BUILD_ARG[@]} \
  -t qrb_ros:latest \
  -f ../dockerfile/qrb_ros.dockerfile .