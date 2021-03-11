#!/bin/bash

# test: bash build.sh binary test 
# prod: bash build.sh binary prod

build_type=$1
run_type=$2

# Prod parameters as default 
opc_ua_domain="andrcar-master.ivt.ntnu.no"

if [ $build_type = 'source' ]
then
    source ~/ros2_foxy/install/setup.bash
elif [ $build_type = 'binary' ]
then
    source /opt/ros/foxy/setup.bash
fi

if [ $run_type = 'test' ]
then 
    echo "Running in test mode"
    opc_ua_domain=0.0.0.0
elif [ $run_type = 'prod' ]
then 
    echo "Running in production mode"
fi

sed -i "/^\([[:space:]]*domain: \).*/s//\1\'$opc_ua_domain\'/" kmr_communication/kmr_communication/config/bringup.yaml

colcon build --symlink-install
source install/setup.bash 
ros2 launch kmr_communication hybrid.launch.py

exit 0