
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for G1 robot synchronization to Mujoco simulation

    Usage:
        ros2 launch synchronize synchronize.launch.py
        ros2 launch synchronize synchronize.launch.py robot_model:=h1
        ros2 launch synchronize synchronize.launch.py scene_file:=/path/to/scene.xml
    """

    # Declare launch arguments
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='g1',
        description='Robot model to synchronize (g1, h1, go2, etc.)'
    )

    scene_file_arg = DeclareLaunchArgument(
        'scene_file',
        default_value='/home/unitree/demo_ws/src/synchronize/models/g1/scene_empty.xml',
        description='Path to Mujoco scene XML file'
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
            'scene_file': LaunchConfiguration('scene_file'),
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
