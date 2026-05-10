import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("object_localizer"),
        "config",
        "object_localizer.yaml"
    )

    return LaunchDescription([
        Node(
            package="object_localizer",
            executable="object_localizer_node",
            name="object_localizer_node",
            output="screen",
            parameters=[config_file]
        )
    ])