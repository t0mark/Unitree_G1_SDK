
#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    
    # Get package share directory
    pkg_share = get_package_share_directory('synchronize')
    models_dir = os.path.join(pkg_share, 'models', 'g1')

    # Declare launch arguments
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='g1',
        description='Robot model to synchronize (g1, h1, go2, etc.)'
    )

    scene_file_arg = DeclareLaunchArgument(
        'scene_file',
        default_value='scene_empty.xml',
        description='Scene XML filename (located in models/g1/ directory)'
    )

    viewer_fps_arg = DeclareLaunchArgument(
        'viewer_fps',
        default_value='50.0',
        description='Viewer refresh rate in FPS'
    )

    # Create synchronize node
    synchronize_node = Node(
        package='synchronize',
        executable='synchronize_node.py',
        name='mujoco_synchronizer',
        output='screen',
        parameters=[{
            'robot_model': LaunchConfiguration('robot_model'),
            'scene_file': PathJoinSubstitution([models_dir, LaunchConfiguration('scene_file')]),
            'viewer_fps': LaunchConfiguration('viewer_fps'),
        }],
        emulate_tty=True,
    )

    return LaunchDescription([
        robot_model_arg,
        scene_file_arg,
        viewer_fps_arg,
        synchronize_node,
    ])
