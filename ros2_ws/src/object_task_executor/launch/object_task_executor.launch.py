import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("object_task_executor"),
        "config",
        "object_task_executor.yaml",
    )

    return LaunchDescription([
        Node(
            package="object_task_executor",
            executable="object_task_executor_node",
            name="object_task_executor_node",
            output="screen",
            parameters=[config_file],
        )
    ])
