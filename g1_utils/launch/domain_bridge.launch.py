import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('g1_utils')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='0to3.yaml',
        description='Configuration filename (relative to config/domain_bridge/)'
    )

    # Get launch configurations
    config_file = LaunchConfiguration('config_file')

    # Build full config path
    config_path = [os.path.join(pkg_dir, 'config', 'domain_bridge', ''), config_file]

    # Domain bridge node
    domain_bridge_node = Node(
        package='g1_utils',
        executable='g1_utils',
        name='domain_bridge',
        output='both',
        arguments=[config_path]
    )

    return LaunchDescription([
        config_file_arg,
        domain_bridge_node,
    ])
