from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('corner_manager'),
        'config',
        'corner_manager.yaml'
    )

    corner_manager_node = Node(
        package='corner_manager',
        executable='corner_manager_node',
        name='corner_manager_node',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([
        corner_manager_node
    ])