"""Run the project with Gazebo or connect the nodes to a running robot.

Usage::

    ros2 launch reactive_tb4 reactive_tb4.launch.py
    ros2 launch reactive_tb4 reactive_tb4.launch.py rviz:=false
    ros2 launch reactive_tb4 reactive_tb4.launch.py sim:=false use_sim_time:=false
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description."""
    pkg = get_package_share_directory('reactive_tb4')
    params = PathJoinSubstitution([pkg, 'config', 'reactive_params.yaml'])
    use_sim_time = LaunchConfiguration('use_sim_time')

    arguments = [
        DeclareLaunchArgument('sim', default_value='true', choices=['true', 'false'],
                              description='Also start Gazebo and the robot'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=PythonExpression([
                "'true' if '", LaunchConfiguration('sim'), "' == 'true' else 'false'" ]),
            choices=['true', 'false'],
            description='Use /clock (true for simulation; false for a physical robot)'),
        DeclareLaunchArgument('robot_namespace', default_value='',
                              description='ROS namespace of the robot, if any'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false'],
                              description='Open RViz showing the map and scan'),
        DeclareLaunchArgument('x', default_value='2.286', description='Spawn x (m)'),
        DeclareLaunchArgument('y', default_value='1.524', description='Spawn y (m)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn heading (rad)'),
    ]

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg, 'launch', 'room_hall_sim.launch.py'])]),
        launch_arguments=[('x', LaunchConfiguration('x')),
                          ('y', LaunchConfiguration('y')),
                          ('yaw', LaunchConfiguration('yaw'))],
        condition=IfCondition(LaunchConfiguration('sim')))

    node_namespace = LaunchConfiguration('robot_namespace')
    controller = Node(
        package='reactive_tb4', executable='reactive_controller',
        name='reactive_controller', namespace=node_namespace, output='screen',
        parameters=[params, {'use_sim_time': use_sim_time,
                             'undock_on_start': LaunchConfiguration('sim')}])

    mapper = Node(
        package='reactive_tb4', executable='occupancy_mapper',
        name='occupancy_mapper', namespace=node_namespace, output='screen',
        parameters=[params, {'use_sim_time': use_sim_time}])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'reactive_tb4.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription(arguments + [simulation, controller, mapper, rviz])
