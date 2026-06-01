from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    mapping_dir = get_package_share_directory("mapping")
    navigation_dir = get_package_share_directory("navigation")
    mapping_launch = os.path.join(
        mapping_dir,
        "launch",
        "mapping.launch.py"
    )

    navigation_slam_launch = os.path.join(
        navigation_dir,
        "launch",
        "navigation_slam.launch.py"
    )

    frontier_explorer_dir = get_package_share_directory("frontier_explorer")
    frontier_explorer_launch = os.path.join(
        frontier_explorer_dir,
        "launch",
        "frontier_explorer.launch.py"

    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch)
        ),

        TimerAction(
            period=12.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(navigation_slam_launch)
                )
            ]
        ),

        TimerAction(
            period=16.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(frontier_explorer_launch)
                )
            ]
        )
    ])
