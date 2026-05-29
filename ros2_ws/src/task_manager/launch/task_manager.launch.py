from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("task_manager"),
        "config",
        "task_manager.yaml",
    )

    return LaunchDescription([
        Node(
            package="task_manager",
            executable="task_manager_node.py",
            name="task_manager",
            output="screen",
            parameters=[config_file],
        )
    ])
