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
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(package_name + '/launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob(package_name + '/config/*.yaml'))
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
            'test = kmr_communication.nodes.test:main',
            'lbr = kmr_communication.nodes.lbr_commands_node:main',
            'opcua_ros2_hybrid = kmr_communication.nodes.opcua_ros2_pubsub:main'
        ],
    },
)
