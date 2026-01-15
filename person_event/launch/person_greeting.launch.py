from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'network_interface',
            default_value='eth0',
            description='Network interface for Unitree SDK'
        ),
        DeclareLaunchArgument(
            'detection_duration',
            default_value='3.0',
            description='Seconds person must be detected before greeting (float)'
        ),
        DeclareLaunchArgument(
            'greeting_count',
            default_value='-1',
            description='Number of greetings to perform (-1 for unlimited)'
        ),
        DeclareLaunchArgument(
            'greeting_interval',
            default_value='10.0',
            description='Minimum seconds between greetings (float)'
        ),
        DeclareLaunchArgument(
            'greeting_sound',
            default_value='hello.mp3',
            description='Greeting sound file name'
        ),
        DeclareLaunchArgument(
            'audio_volume',
            default_value='100',
            description='Audio volume (0-100)'
        ),

        Node(
            package='person_event',
            executable='person_event',
            name='person_event',
            output='screen',
            parameters=[{
                'network_interface': LaunchConfiguration('network_interface'),
                'detection_duration': LaunchConfiguration('detection_duration'),
                'greeting_count': LaunchConfiguration('greeting_count'),
                'greeting_interval': LaunchConfiguration('greeting_interval'),
                'greeting_sound': LaunchConfiguration('greeting_sound'),
                'audio_volume': LaunchConfiguration('audio_volume'),
            }]
        ),
    ])
