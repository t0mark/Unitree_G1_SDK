
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

    # Camera follow arguments
    camera_follow_arg = DeclareLaunchArgument(
        'camera_follow',
        default_value='true',
        description='Enable camera follow mode'
    )

    camera_distance_arg = DeclareLaunchArgument(
        'camera_distance',
        default_value='3.0',
        description='Initial camera distance from robot'
    )

    camera_azimuth_arg = DeclareLaunchArgument(
        'camera_azimuth',
        default_value='0.0',
        description='Initial camera azimuth angle (degrees)'
    )

    camera_elevation_arg = DeclareLaunchArgument(
        'camera_elevation',
        default_value='-20.0',
        description='Initial camera elevation angle (degrees)'
    )

    camera_lookat_offset_x_arg = DeclareLaunchArgument(
        'camera_lookat_offset_x',
        default_value='0.0',
        description='Camera lookat X offset from robot'
    )

    camera_lookat_offset_y_arg = DeclareLaunchArgument(
        'camera_lookat_offset_y',
        default_value='0.0',
        description='Camera lookat Y offset from robot'
    )

    camera_lookat_offset_z_arg = DeclareLaunchArgument(
        'camera_lookat_offset_z',
        default_value='0.1',
        description='Camera lookat Z offset from robot'
    )

    # Create synchronize node
    synchronize_node = Node(
        package='synchronize',
        executable='synchronize_node',
        name='mujoco_synchronizer',
        output='screen',
        parameters=[{
            'robot_model': LaunchConfiguration('robot_model'),
            'scene_file': PathJoinSubstitution([models_dir, LaunchConfiguration('scene_file')]),
            'viewer_fps': LaunchConfiguration('viewer_fps'),
            'camera_follow': LaunchConfiguration('camera_follow'),
            'camera_distance': LaunchConfiguration('camera_distance'),
            'camera_azimuth': LaunchConfiguration('camera_azimuth'),
            'camera_elevation': LaunchConfiguration('camera_elevation'),
            'camera_lookat_offset_x': LaunchConfiguration('camera_lookat_offset_x'),
            'camera_lookat_offset_y': LaunchConfiguration('camera_lookat_offset_y'),
            'camera_lookat_offset_z': LaunchConfiguration('camera_lookat_offset_z'),
        }],
        emulate_tty=True,
    )

    return LaunchDescription([
        robot_model_arg,
        scene_file_arg,
        viewer_fps_arg,
        camera_follow_arg,
        camera_distance_arg,
        camera_azimuth_arg,
        camera_elevation_arg,
        camera_lookat_offset_x_arg,
        camera_lookat_offset_y_arg,
        camera_lookat_offset_z_arg,
        synchronize_node,
    ])
