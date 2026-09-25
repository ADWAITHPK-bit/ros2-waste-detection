#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    map_yaml = LaunchConfiguration('map')

    declare_map = DeclareLaunchArgument(
        'map',
        description='Full path to map yaml file to load'
    )

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch_path = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

    turtlebot3_nav2_dir = get_package_share_directory('turtlebot3_navigation2')
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi')
    param_dir = os.path.join(turtlebot3_nav2_dir, 'param', 'humble', turtlebot3_model + '.yaml')

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_path),
        launch_arguments={
            'namespace': 'robot2',
            'use_namespace': 'True',
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': param_dir,
            'autostart': 'True',
        }.items(),
    )

    return LaunchDescription([
        declare_map,
        bringup_cmd,
    ])
