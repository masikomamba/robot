"""Start Gazebo with the room-and-hall world and spawn a TurtleBot 4.

This mirrors what ``turtlebot4_gz_bringup/turtlebot4_gz.launch.py`` does, with
one change: the world file comes from this package instead of the stock
TurtleBot 4 worlds folder. (The stock sim launch file overwrites
GZ_SIM_RESOURCE_PATH, so it cannot find worlds stored anywhere else.)
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description."""
    pkg_reactive = get_package_share_directory('reactive_tb4')
    pkg_tb4_gz = get_package_share_directory('turtlebot4_gz_bringup')
    pkg_tb4_gui = get_package_share_directory('turtlebot4_gz_gui_plugins')
    pkg_tb4_desc = get_package_share_directory('turtlebot4_description')
    pkg_create_desc = get_package_share_directory('irobot_create_description')
    pkg_create_gz = get_package_share_directory('irobot_create_gz_bringup')
    pkg_create_plugins = get_package_share_directory('irobot_create_gz_plugins')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    arguments = [
        # The world NAME must match <world name='...'> inside the .sdf file,
        # because the lidar/bumper bridges use it to build Gazebo topic names.
        DeclareLaunchArgument('world', default_value='room_hall',
                              description='World name (file worlds/<world>.sdf)'),
        DeclareLaunchArgument('model', default_value='standard',
                              choices=['standard', 'lite'], description='TurtleBot 4 model'),
        DeclareLaunchArgument('x', default_value='2.286', description='Spawn x (m)'),
        DeclareLaunchArgument('y', default_value='1.524', description='Spawn y (m)'),
        DeclareLaunchArgument('z', default_value='0.0', description='Spawn z (m)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn heading (rad)'),
    ]

    # Where Gazebo looks for meshes and models: our worlds plus the robot's.
    resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(pkg_reactive, 'worlds'),
            os.path.join(pkg_tb4_gz, 'worlds'),
            os.path.join(pkg_create_gz, 'worlds'),
            str(Path(pkg_tb4_desc).parent.resolve()),
            str(Path(pkg_create_desc).parent.resolve()),
        ]))
    gui_plugin_path = SetEnvironmentVariable(
        name='GZ_GUI_PLUGIN_PATH',
        value=':'.join([os.path.join(pkg_tb4_gui, 'lib'),
                        os.path.join(pkg_create_plugins, 'lib')]))

    # Gazebo Harmonic itself. "-r" starts the simulation running right away.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])]),
        launch_arguments=[('gz_args', [
            PathJoinSubstitution([pkg_reactive, 'worlds', LaunchConfiguration('world')]),
            '.sdf -r -v 3 --gui-config ',
            PathJoinSubstitution([pkg_tb4_gz, 'gui', LaunchConfiguration('model'),
                                  'gui.config']),
        ])])

    # Gazebo's clock -> ROS /clock so every node can use simulation time.
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        output='screen', arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])

    # Robot + dock + all TurtleBot 4 / Create 3 nodes and sensor bridges.
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_tb4_gz, 'launch', 'turtlebot4_spawn.launch.py'])]),
        launch_arguments=[
            ('model', LaunchConfiguration('model')),
            ('rviz', 'false'),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw')),
        ])

    return LaunchDescription(
        arguments + [resource_path, gui_plugin_path, gazebo, clock_bridge, spawn])
