"""Install script for the reactive_tb4 package (ament_python)."""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'reactive_tb4'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jayce Easter',
    maintainer_email='jayce.easter@ou.edu',
    description='Reactive TurtleBot 4 controller and occupancy grid mapper for simulation and hardware.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reactive_controller = reactive_tb4.behavior_controller:main',
            'occupancy_mapper = reactive_tb4.occupancy_mapper:main',
        ],
    },
)
