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
            description='트리거: 사람 인식 지속 시간'
        ),
        DeclareLaunchArgument(
            'greeting_count',
            default_value='-1',
            description='반복 횟수 설정 (-1 : inf)'
        ),
        DeclareLaunchArgument(
            'greeting_interval',
            default_value='15.0',
            description='인사 사이의 간격'
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
