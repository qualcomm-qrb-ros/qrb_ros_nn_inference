#!/bin/bash

export DOCKER_BUILDKIT=1
export COMPOSE_DOCKER_CLI_BUILD=1
export QRB_ROS_WS=/home/qrb_ros_ws

docker build \
  -t qrb_ros:latest \
  -f ./qrb_ros.dockerfile .

docker run -it \
	--privileged \
  --network host \
  -v /home/qrb_ros_ws/:/workspace/qrb_ros_ws \
  --name "qrb_ros_container" \
  --hostname "qrb_ros" \
  --workdir /workspace/qrb_ros_ws \
  qrb_ros:latest