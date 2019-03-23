#!/bin/bash

# Do not compile Lisp messages
# XXX: not sure if this is the place to put this.
export PYTHONPATH=$(pwd - P)/catkin_ws/src/utm
export ROS_LANG_DISABLE=gennodejs:geneus:genlisp
GAZEBO_MODEL_PATH=$(pwd - P)/catkin_ws/src/gazebo/model
shell=`basename $SHELL`
echo "Activating ROS with shell: $SHELL"
source /opt/ros/kinetic/setup.$shell
export GAZEBO_MODEL_PATH
export HOSTNAME=$HOSTNAME
export ROS_HOSTNAME=$HOSTNAME.local
echo "Set ROS_HOSTNAME to: $ROS_HOSTNAME"
echo "Set GAZEBO_MODEL_PATH to: $GAZEBO_MODEL_PATH"
if [ 2015 -ge $(date +%Y) ];
then
    >&2 echo "Error! Time travel detected. System time is: $(date)"
fi

exec "$@" #Passes arguments. Need this for ROS remote launching to work.

source catkin_ws/devel/setup.bash
