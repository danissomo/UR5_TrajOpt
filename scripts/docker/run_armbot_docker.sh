#!/bin/bash

echo ROS_MASTER_URI=$1
echo ROS_IP=$2

xhost +local:docker || true
docker run  -it --rm \
        -e "DISPLAY" \
        -e "QT_X11_NO_MITSHM=1" \
        -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        -e XAUTHORITY \
        -e ARMBOT_PATH='/workspace' \
        -e ROS_MASTER_URI="$1" \
        -e ROS_IP="$2" \
        -v /dev:/dev \
        -v "$(pwd)":/workspace \
        -v ~:/home \
       --net=host \
       --privileged \
       --name trajopt trajopt-img

# wstool init /workspace/src/libs/ /workspace/src/libs/tesseract_planning/dependencies.rosinstall