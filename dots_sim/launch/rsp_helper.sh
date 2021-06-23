#!/bin/bash

# First parameter is the robot urdf filename.
# All parameters passed on to robot_state_publisher once file is available.
#   

echo 'Running rsp with:' "$@"
while [ ! -f $1 ]; do 
    echo 'Waiting for urdf..' 
    sleep 1 
done 
echo 'Got urdf..'
ros2 run robot_state_publisher robot_state_publisher "$@"
