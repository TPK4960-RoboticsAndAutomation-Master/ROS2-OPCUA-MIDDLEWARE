import os
from glob import glob
from setuptools import setup

package_name = 'kmr_communication'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(package_name + '/launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob(package_name + '/config/*.yaml'))
    ],
    scripts=[
        package_name + '/script/tcp_socket2.py', 
        package_name + '/script/udp_socket.py',
        package_name + '/script/sockets.py'
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mathias',
    maintainer_email='mathias.neslow96@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lbr = kmr_communication.nodes.lbr_command_node2:main',
            'kmp = kmr_communication.nodes.kmp_command_node2:main',
            'opcua_ros2_hybrid = kmr_communication.nodes.opcua_ros2_pubsub2:main',
        ],
    },
)
