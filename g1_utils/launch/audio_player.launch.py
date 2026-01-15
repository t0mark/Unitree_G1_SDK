#!/usr/bin/env python3
"""
Launch file for Audio Player Node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    network_interface_arg = DeclareLaunchArgument(
        'network_interface',
        default_value='eth0',
        description='Network interface for Unitree SDK communication'
    )

    default_volume_arg = DeclareLaunchArgument(
        'default_volume',
        default_value='50',
        description='Default audio volume (0-100)'
    )

    app_name_arg = DeclareLaunchArgument(
        'app_name',
        default_value='audio_player',
        description='Application name for audio stream identification'
    )

    sound_file_arg = DeclareLaunchArgument(
        'sound_file',
        default_value='hello.mp3',
        description='Audio file to play on startup (e.g., "hello.mp3")'
    )

    # Audio Player Node
    audio_player_node = Node(
        package='g1_utils',
        executable='audio_player_node.py',
        name='audio_player_node',
        output='screen',
        parameters=[{
            'network_interface': LaunchConfiguration('network_interface'),
            'default_volume': LaunchConfiguration('default_volume'),
            'app_name': LaunchConfiguration('app_name'),
            'sound_file': LaunchConfiguration('sound_file'),
        }]
    )

    return LaunchDescription([
        network_interface_arg,
        default_volume_arg,
        app_name_arg,
        sound_file_arg,
        audio_player_node,
    ])
