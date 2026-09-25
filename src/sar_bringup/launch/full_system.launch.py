#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    sim_dir    = get_package_share_directory('sar_sim')
    bringup_dir = get_package_share_directory('sar_bringup')
    nav2_dir   = get_package_share_directory('nav2_bringup')

    map_path     = os.path.join(bringup_dir, 'config', 'map.yaml')
    robot1_params = os.path.join(bringup_dir, 'config', 'robot1_nav2.yaml')
    robot2_params = os.path.join(bringup_dir, 'config', 'robot2_nav2.yaml')

    # Include spawn_robots (Gazebo + RSP + spawn)
    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_dir, 'launch', 'spawn_robots.launch.py')
        )
    )

    # Nav2 for robot1
    nav2_robot1 = TimerAction(
        period=15.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'namespace':      'robot1',
                'use_namespace':  'True',
                'map':            map_path,
                'params_file':    robot1_params,
                'use_sim_time':   'True'
            }.items()
        )]
    )

    # Nav2 for robot2
    nav2_robot2 = TimerAction(
        period=15.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'namespace':      'robot2',
                'use_namespace':  'True',
                'map':            map_path,
                'params_file':    robot2_params,
                'use_sim_time':   'True'
            }.items()
        )]
    )

    # Perception nodes
    perception_robot1 = TimerAction(
        period=20.0,
        actions=[Node(
            package='sar_perception',
            executable='aruco_detector',
            name='aruco_detector_robot1',
            parameters=[{'robot_namespace': 'robot1'}],
            output='screen'
        )]
    )

    perception_robot2 = TimerAction(
        period=20.0,
        actions=[Node(
            package='sar_perception',
            executable='aruco_detector',
            name='aruco_detector_robot2',
            parameters=[{'robot_namespace': 'robot2'}],
            output='screen'
        )]
    )

    # Database and allocator
    database_node = TimerAction(
        period=20.0,
        actions=[Node(
            package='sar_database',
            executable='database_node',
            name='database_node',
            output='screen'
        )]
    )

    allocator_node = TimerAction(
        period=22.0,
        actions=[Node(
            package='sar_allocation',
            executable='allocator_node',
            name='allocator_node',
            output='screen'
        )]
    )

    return LaunchDescription([
        spawn_launch,
        nav2_robot1,
        nav2_robot2,
        perception_robot1,
        perception_robot2,
        database_node,
        allocator_node,
    ])