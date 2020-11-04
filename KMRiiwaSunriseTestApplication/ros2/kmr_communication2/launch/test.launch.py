import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
#import sys
#from rclpy.utilities import remove_ros_args
#import argparse


def generate_launch_description():

    connection_type_TCP='TCP'

    robot="KMR"

    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('kmr_communication2'),
            'param',
            'bringup.yaml'))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'param_dir',
                default_value=param_dir,
                description='Full path to parameter file to load'
            ),

            # launch_ros.actions.Node(
            #     package="kmr_communication2",
            #     node_executable="lbr_commands_node.py",
            #     node_name="lbr_commands_node",
            #     output="screen",
            #     emulate_tty=True,
            #     arguments=['-c', connection_type_TCP, '-ro', robot],
            #     parameters=[param_dir]
            # )

            launch_ros.actions.Node(
                package="kmr_communication2",
                executable="test_node.py",
                name="test_node",
                output="screen",
                emulate_tty=True,
            )
        ]
    )