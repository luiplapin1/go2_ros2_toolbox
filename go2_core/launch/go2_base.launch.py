import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('go2_core')
    rviz_config_path = os.path.join(pkg_dir, 'config', 'default.rviz')

    go2_base_node = Node(
        package='go2_core',
        executable='go2_base',
        name='go2_base',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        go2_base_node,
        rviz_node
    ])
