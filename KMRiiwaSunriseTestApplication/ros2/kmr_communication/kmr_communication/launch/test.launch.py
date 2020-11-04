import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'kmr_communication'
    connection_type_TCP='TCP'
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
            name="lbr_commands_node",
            output="screen",
            emulate_tty=True,
            arguments=['-c', connection_type_TCP, '-ro', robot],
            parameters=[param_dir]
        ), 

        Node(
            package=pkg_name,
            executable="test",
            name="test"
        )
    ])