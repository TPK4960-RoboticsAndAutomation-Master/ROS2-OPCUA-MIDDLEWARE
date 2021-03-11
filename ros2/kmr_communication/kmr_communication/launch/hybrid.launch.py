import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description(argv=sys.argv[1:]):
    pkg_name = 'kmr_communication'

    config_file_path = os.path.join(
            get_package_share_directory(pkg_name),
            'config',
            'bringup_test.yaml')
    config_file = open(config_file_path)
    parsed_config_file = yaml.load(config_file, Loader=yaml.FullLoader)
    
    hybrid_node_params = parsed_config_file['hybrid_node']['ros__parameters']

    param_dir = LaunchConfiguration(
        'param_dir',
        default=config_file_path
    )

    return LaunchDescription([
        DeclareLaunchArgument(
                'param_dir',
                default_value=param_dir,
                description="Full path to parameter file to load"
            ),

        Node(
            package=pkg_name,
            executable='opcua_ros2_hybrid',
            name='hybrid_node',
            output='screen',
            emulate_tty=True,
            arguments=['-d', hybrid_node_params['domain']]
        )
    ])