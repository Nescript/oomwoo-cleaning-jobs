from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config = Path(get_package_share_directory('oomwoo_rose2')) / 'config' / 'rose2.yaml'
    return LaunchDescription([
        Node(
            package='oomwoo_rose2',
            executable='oomwoo-rose2-server',
            name='oomwoo_rose2',
            output='screen',
            parameters=[str(config)],
        ),
    ])
