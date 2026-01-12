#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('g1_utils'),
        'config',
        'domain_bridge.yaml'
    )

    return LaunchDescription([
        Node(
            package='g1_utils',
            executable='domain_bridge_node',
            name='domain_bridge',
            output='screen',
            parameters=[config_file]
        )
    ])
