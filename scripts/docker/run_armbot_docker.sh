#!/bin/bash

ROS_MASTER_URI=$1
ROS_IP=$2

if [[ -z "$ROS_MASTER_URI" ]]; then
   ROS_MASTER_URI='http://127.0.0.1:11311'
fi

if [[ -z "$ROS_IP" ]]; then
   ROS_IP='127.0.0.1'
fi

echo ROS_MASTER_URI=$ROS_MASTER_URI
echo ROS_IP=$ROS_IP

xhost +local:docker || true
docker run  -it --rm \
        -e "DISPLAY" \
        -e "QT_X11_NO_MITSHM=1" \
        -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        -e XAUTHORITY \
        -e ARMBOT_PATH='/workspace' \
        -e ROS_MASTER_URI="$ROS_MASTER_URI" \
        -e ROS_IP="$ROS_IP" \
        -v /dev:/dev \
        -v "$(pwd)":/workspace \
        -v ~:/home \
       --net=host \
       --privileged \
       --name trajopt trajopt-img

# wstool init /workspace/src/libs/ /workspace/src/libs/tesseract_planning/dependencies.rosinstall