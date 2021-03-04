#!/bin/bash
echo "Entrypoint entered"

source /opt/ros/melodic/setup.sh
source /opt/acr/devel/setup.sh

roslaunch acr_setup acr_setup.launch &
rosrun teleop_twist_keyboard teleop_twist_keyboard.py


# /bin/bash "$@"
