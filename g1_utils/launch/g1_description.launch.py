#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    # Get package share directory
    pkg_share = get_package_share_directory('g1_utils')

    # Default URDF path
    default_urdf = os.path.join(pkg_share, 'models', 'urdf', 'g1_29dof.urdf')

    # Declare launch arguments
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=default_urdf,
        description='Path to robot URDF file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz automatically'
    )

    launch_joint_state_publisher_gui_arg = DeclareLaunchArgument(
        'joint_state_publisher_gui',
        default_value='true',
        description='Launch joint_state_publisher_gui for manual joint control'
    )

    # Read URDF file
    urdf_file = LaunchConfiguration('urdf_file')

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['cat ', urdf_file]),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('robot_description', 'g1_description'),
        ],
    )

    # Joint state publisher GUI (for manual joint control)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('joint_state_publisher_gui')),
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )

    return LaunchDescription([
        urdf_file_arg,
        use_sim_time_arg,
        launch_rviz_arg,
        launch_joint_state_publisher_gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
