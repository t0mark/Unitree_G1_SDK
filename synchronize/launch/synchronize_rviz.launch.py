#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    # Get package share directories
    pkg_share = get_package_share_directory('synchronize')
    g1_utils_share = get_package_share_directory('g1_utils')

    # Default rviz config
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'synchronize.rviz')

    # Declare launch arguments
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='g1',
        description='Robot model to synchronize'
    )

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='world',
        description='Base frame for TF'
    )

    robot_frame_arg = DeclareLaunchArgument(
        'robot_frame',
        default_value='pelvis',
        description='Robot base link frame'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='50.0',
        description='Publishing rate in Hz'
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz automatically'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to RViz config file'
    )

    # Create synchronize_rviz node
    synchronize_rviz_node = Node(
        package='synchronize',
        executable='synchronize_rviz_node',
        name='rviz_synchronizer',
        output='screen',
        parameters=[{
            'robot_model': LaunchConfiguration('robot_model'),
            'base_frame': LaunchConfiguration('base_frame'),
            'robot_frame': LaunchConfiguration('robot_frame'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }],
        emulate_tty=True,
    )

    # Include g1_description launch file (handles robot_state_publisher only)
    g1_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(g1_utils_share, 'launch', 'g1_description.launch.py')
        ),
        launch_arguments={
            'launch_rviz': 'false',
            'joint_state_publisher_gui': 'false',
        }.items(),
    )

    # RViz node with config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription([
        robot_model_arg,
        base_frame_arg,
        robot_frame_arg,
        publish_rate_arg,
        launch_rviz_arg,
        rviz_config_arg,
        synchronize_rviz_node,
        g1_description_launch,
        rviz_node,
    ])
