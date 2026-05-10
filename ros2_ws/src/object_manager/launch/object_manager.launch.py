import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("object_manager"),
        "config",
        "object_manager.yaml"
    )

    return LaunchDescription([
        Node(
            package="object_manager",
            executable="object_manager_node",
            name="object_manager_node",
            output="screen",
            parameters=[config_file]
        )
    ])