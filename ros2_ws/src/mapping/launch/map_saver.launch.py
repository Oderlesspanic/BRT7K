from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("mapping"),
        "config",
        "map_saver.yaml"
    )

    return LaunchDescription([
        Node(
            package="mapping",
            executable="map_saver_node",
            name="map_saver_node",
            output="screen",
            parameters=[config_file]
        )
    ])