from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("hardware_supervisor"),
        "config",
        "hardware_supervisor.yaml"
    )

    return LaunchDescription([
        Node(
            package="hardware_supervisor",
            executable="hardware_supervisor_node",
            name="hardware_supervisor",
            output="screen",
            parameters=[config_file],
        )
    ])