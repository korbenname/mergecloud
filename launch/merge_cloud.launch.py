from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch_ros.actions import Node


def generate_launch_description():
    config_file = Path(
        get_package_share_directory('merge_cloud')) / 'config' / 'merge_cloud.yaml'

    return LaunchDescription([
        Node(
            package='merge_cloud',
            executable='merge_cloud_node',
            name='merge_cloud_node',
            output='screen',
            parameters=[str(config_file)],
        ),
    ])