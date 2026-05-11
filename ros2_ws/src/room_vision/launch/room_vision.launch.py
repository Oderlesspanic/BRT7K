from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("room_vision")
    detector_cfg = os.path.join(pkg_share, "config", "detector.yaml")

    return LaunchDescription([
        Node(
            package="room_vision",
            executable="detector",
            name="room_vision_detector",
            output="screen",
            parameters=[detector_cfg],
        ),
    ])
