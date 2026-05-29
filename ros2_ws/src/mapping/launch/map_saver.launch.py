import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("mapping"),
        "config",
        "map_saver.yaml"
    )
    map_path = LaunchConfiguration("map_path")
    default_map_dir = os.environ.get(
        "BRT7K_MAP_DIR",
        os.path.join(os.path.expanduser("~"), "BRT7K", "ros2_ws", "maps", "map"),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map_path",
            default_value=os.path.join(default_map_dir, "arena_map"),
            description="Map basename without .yaml/.pgm suffix",
        ),
        Node(
            package="mapping",
            executable="map_saver_node",
            name="map_saver_node",
            output="screen",
            parameters=[
                config_file,
                {"map_path": map_path},
            ]
        )
    ])
