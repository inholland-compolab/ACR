#!/bin/bash

SCRIPTPATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

if [[ $# -lt 2 ]] ; then
    echo "Usage: ./execute.sh [arduino-port] [lidar-port]"
    exit 1
fi
xhost +local:docker
docker run -p 11311:11311 -it --device=${1}://dev/ttyACM0 --device=${2}://dev/ttyACM1 --gpus all -v //opt/tmp/:$SCRIPTPATH/ros -v //tmp/.X11-unix://tmp/.X11-unix -e DISPLAY=$DISPLAY ros-acr

