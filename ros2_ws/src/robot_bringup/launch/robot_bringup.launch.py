from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory("robot_bringup")

    description_launch = os.path.join(
        bringup_dir,
        "launch",
        "description.launch.py"
    )

    web_launch = os.path.join(
        bringup_dir,
        "launch",
        "web.launch.py"
    )

    hardware_launch = os.path.join(
        bringup_dir,
        "launch",
        "hardware.launch.py"
    )

    vision_launch = os.path.join(
        bringup_dir,
        "launch",
        "vision.launch.py"
    )

    odometry_launch = os.path.join(
        bringup_dir,
        "launch",
        "odometry.launch.py"
    )


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(web_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vision_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(odometry_launch)
        )
    ])