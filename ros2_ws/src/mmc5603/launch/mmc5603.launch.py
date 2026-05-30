from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('mmc5603'),
        'config',
        'mmc5603.yaml'
    )

    return LaunchDescription([
        Node(
            package='mmc5603',
            executable='mmc5603_node',
            name='mmc5603_node',
            output='screen',
            parameters=[config_file]
        )
    ])
