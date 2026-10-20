"""Run the controller and mapper against an already-running physical TurtleBot 4.

The robot's standard TurtleBot 4 bringup must already be running and reachable
on the same ROS 2 network. This launch never starts Gazebo or robot bringup.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Create the hardware-connected nodes using system time."""
    pkg = get_package_share_directory('reactive_tb4')
    params = PathJoinSubstitution([pkg, 'config', 'reactive_params.yaml'])
    namespace = LaunchConfiguration('robot_namespace')

    arguments = [
        DeclareLaunchArgument(
            'robot_namespace', default_value='',
            description='ROS namespace used by the robot, if any'),
        DeclareLaunchArgument(
            'rviz', default_value='false', choices=['true', 'false'],
            description='Open RViz on this computer'),
    ]

    controller = Node(
        package='reactive_tb4', executable='reactive_controller',
        name='reactive_controller', namespace=namespace, output='screen',
        parameters=[params, {'use_sim_time': False, 'undock_on_start': False}])
    mapper = Node(
        package='reactive_tb4', executable='occupancy_mapper',
        name='occupancy_mapper', namespace=namespace, output='screen',
        parameters=[params, {'use_sim_time': False}])
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', namespace=namespace,
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'reactive_tb4.rviz'])],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription(arguments + [controller, mapper, rviz])
