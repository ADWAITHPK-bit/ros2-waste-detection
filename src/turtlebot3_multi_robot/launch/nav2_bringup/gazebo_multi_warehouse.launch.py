#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition


def generate_launch_description():
    ld = LaunchDescription()

    # Names and poses of the robots
    robots = [
        {'name': 'robot1', 'x_pose': '0.0', 'y_pose': '0.0', 'z_pose': 0.01},
        {'name': 'robot2', 'x_pose': '2.0', 'y_pose': '0.0', 'z_pose': 0.01},
    ]

    TURTLEBOT3_MODEL = 'waffle'

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )

    enable_drive = LaunchConfiguration('enable_drive', default='false')
    declare_enable_drive = DeclareLaunchArgument(
        name='enable_drive', default_value=enable_drive, description='Enable robot drive node'
    )

    turtlebot3_multi_robot = get_package_share_directory('turtlebot3_multi_robot')

    urdf = os.path.join(
        turtlebot3_multi_robot, 'urdf', 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    )

    # Point at YOUR warehouse world instead of the package's default
    sar_sim_dir = get_package_share_directory('sar_sim')
    world = os.path.join(sar_sim_dir, 'worlds', 'Maze', 'warehouse.world')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ),
    )

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_drive)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    # Remapping is required for state publisher otherwise /tf and /tf_static
    # will get published on the root '/' namespace
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    last_action = None
    # Spawn turtlebot3 instances in gazebo
    for robot in robots:

        namespace = ['/' + robot['name']]

        # Create state publisher node for that instance
        turtlebot_state_publisher = Node(
            package='robot_state_publisher',
            namespace=namespace,
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'publish_frequency': 10.0}],
            remappings=remappings,
            arguments=[urdf],
        )

        # Create spawn call
        spawn_turtlebot3 = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', os.path.join(turtlebot3_multi_robot, 'models',
                                       'turtlebot3_' + TURTLEBOT3_MODEL, 'model.sdf'),
                '-entity', robot['name'],
                '-robot_namespace', namespace,
                '-x', robot['x_pose'], '-y', robot['y_pose'],
                '-z', '0.01', '-Y', '0.0',
                '-unpause',
            ],
            output='screen',
        )

        drive_turtlebot3 = Node(
            package='turtlebot3_gazebo', executable='turtlebot3_drive',
            namespace=namespace, output='screen',
            condition=IfCondition(enable_drive),
        )

        if last_action is None:
            # First robot: add directly
            ld.add_action(turtlebot_state_publisher)
            ld.add_action(spawn_turtlebot3)
            ld.add_action(drive_turtlebot3)
        else:
            # Subsequent robots: wait for previous spawn to finish first
            spawn_turtlebot3_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[turtlebot_state_publisher,
                             spawn_turtlebot3,
                             drive_turtlebot3],
                )
            )
            ld.add_action(spawn_turtlebot3_event)

        last_action = spawn_turtlebot3

    return ld