#!/bin/bash

source ~/ros2_foxy/install/setup.bash

cd ~/ROS2-OPCUA-MIDDLEWARE/ros2
colcon build --symlink-install
source install/setup.bash 
ros2 launch kmr_communication kmr.launch.py

exit 0
