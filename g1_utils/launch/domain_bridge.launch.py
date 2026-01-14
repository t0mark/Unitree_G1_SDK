import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('g1_utils')

    # Config file paths
    config_0to3_path = os.path.join(pkg_dir, 'config', 'domain_bridge', '0to3.yaml')
    config_3to0_path = os.path.join(pkg_dir, 'config', 'domain_bridge', '3to0.yaml')

    # Domain bridge node: 0 to 3 (Robot -> Developer PC)
    bridge_0to3_node = Node(
        package='g1_utils',
        executable='g1_utils',
        name='bridge_0to3',
        output='both',
        arguments=[config_0to3_path]
    )

    # Domain bridge node: 3 to 0 (Developer PC -> Robot)
    bridge_3to0_node = Node(
        package='g1_utils',
        executable='g1_utils',
        name='bridge_3to0',
        output='both',
        arguments=[config_3to0_path]
    )

    return LaunchDescription([
        bridge_0to3_node,
        bridge_3to0_node,
    ])