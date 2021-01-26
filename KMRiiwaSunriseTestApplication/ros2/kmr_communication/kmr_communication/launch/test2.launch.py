import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description(argv=sys.argv[1:]):
    pkg_name = 'kmr_communication'
    connection_type='TCP'
    robot="KMR"

    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory(pkg_name),
            'config',
            'bringup.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
                'param_dir',
                default_value=param_dir,
                description='Full path to parameter file to load'
            ),

        Node(
            package=pkg_name,
            executable="lbr",
            name="lbr_command_node2",
            output="screen",
            emulate_tty=True,
            arguments=['-c', connection_type, '-ro', robot],
            parameters=[param_dir]
        ), 

        Node(
            package=pkg_name,
            executable="kmp",
            name="kmp_command_node2",
            output="screen",
            emulate_tty=True,
            arguments=['-c', connection_type, '-ro', robot],
            parameters=[param_dir]
        ),

        Node(
            package=pkg_name,
            executable="opcua_ros2_hybrid",
            name="hybrid_node2"
        )
    ])