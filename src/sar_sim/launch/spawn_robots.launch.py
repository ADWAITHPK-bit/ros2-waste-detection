#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

tb3_description = get_package_share_directory('turtlebot3_description')
urdf_path = os.path.join(tb3_description, 'urdf', 'turtlebot3_waffle_pi.urdf')

robot_state_pub1 = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    namespace='robot1',
    parameters=[{
        'robot_description': Command(['xacro ', urdf_path]),
        'use_sim_time': True,
        'frame_prefix': 'robot1/'
        }],
    output='screen'
    )

robot_state_pub2 = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    namespace='robot2',
    parameters=[{
        'robot_description': Command(['xacro ', urdf_path]),
        'use_sim_time': True,
        'frame_prefix': 'robot2/'
        }],
    output='screen'
    )
def generate_launch_description():

    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    sim_dir = get_package_share_directory('sar_sim')
    world_path = os.path.join(sim_dir, 'worlds', 'Maze', 'warehouse.world')

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen',
        additional_env={'TURTLEBOT3_MODEL': 'waffle_pi'}
    )

    spawn_robot1 = TimerAction(
        period=5.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'robot1',
                '-file', os.path.join(
                    get_package_share_directory('turtlebot3_gazebo'),
                    'models', 'turtlebot3_waffle_pi', 'model.sdf'
                ),
                '-x', '0.0', '-y', '0.0', '-z', '0.01',
                '-robot_namespace', 'robot1'
            ],
            output='screen'
        )]
    )

    spawn_robot2 = TimerAction(
        period=5.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'robot2',
                '-file', os.path.join(
                    get_package_share_directory('turtlebot3_gazebo'),
                    'models', 'turtlebot3_waffle_pi', 'model.sdf'
                ),
                '-x', '2.0', '-y', '0.0', '-z', '0.01',
                '-robot_namespace', 'robot2'
            ],
            output='screen'
        )]
    )

    return LaunchDescription([
        gazebo,
        robot_state_pub1,
        robot_state_pub2,
        spawn_robot1,
        spawn_robot2,
    ])