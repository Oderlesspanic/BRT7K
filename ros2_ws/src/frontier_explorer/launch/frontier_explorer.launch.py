from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("frontier_explorer"),
        "config",
        "frontier_explorer.yaml"
    )

    return LaunchDescription([
        Node(
            package="frontier_explorer",
            executable="frontier_explorer_node",
            name="frontier_explorer_node",
            output="screen",
            parameters=[config]
        )
    ])