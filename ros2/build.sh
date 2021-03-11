#!/bin/bash

# test: bash build.sh binary TCP test 
# prod: bash build.sh binary TCP  

build_type=$1
connection_type=$2
run_type=$3

# Prod parameters as default 
lbr_port=30005
kmp_port=30002
kmr_ip="172.31.1.69'"
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
    lbr_port=50007
    kmp_port=50008
    kmr_ip=127.0.0.1
    opc_ua_domain=0.0.0.0
elif [ $run_type = 'prod' ]
then 
    echo "Running in production mode"
fi

yq -y '.connection_params.ros__parameters.connection_type = ${connection_type}' \
    kmr_communication/kmr_communication/config/bringup_test.yaml \
    | sponge kmr_communication/kmr_communication/config/bringup_test.yaml

# cat kmr_communication/kmr_communication/config/bringup_test.yaml \
#     | yq -y '.connection_params.ros__parameters.connection_type = $v' --arg v $connection_type \
#     | sponge kmr_communication/kmr_communication/config/bringup_test.yaml

# cat kmr_communication/kmr_communication/config/bringup_test.yaml \
#     | yq -y '.lbr_command_node.ros__parameters.port = $v' --arg v $lbr_port \
#     | sponge kmr_communication/kmr_communication/config/bringup_test.yaml

# cat kmr_communication/kmr_communication/config/bringup_test.yaml \
#     | yq -y '.lbr_command_node.ros__parameters.kmr_ip = $v' --arg v $kmr_ip \
#     | sponge kmr_communication/kmr_communication/config/bringup_test.yaml

# cat kmr_communication/kmr_communication/config/bringup_test.yaml \
#     | yq -y '.kmp_command_node.ros__parameters.port = $v' --arg v $kmp_port \
#     | sponge kmr_communication/kmr_communication/config/bringup_test.yaml

# cat kmr_communication/kmr_communication/config/bringup_test.yaml \
#     | yq -y '.kmp_command_node.ros__parameters.kmr_ip = $v' --arg v $kmr_ip \
#     | sponge kmr_communication/kmr_communication/config/bringup_test.yaml

# cat kmr_communication/kmr_communication/config/bringup_test.yaml \
#     | yq -y '.hybrid_node.ros__parameters.domain = $v' --arg v $opc_ua_domain \
#     | sponge kmr_communication/kmr_communication/config/bringup_test.yaml

colcon build --symlink-install
source install/setup.bash 
ros2 launch kmr_communication kmr.launch.py

exit 0
