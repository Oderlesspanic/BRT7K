from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    hardware_supervisor_dir = get_package_share_directory("hardware_supervisor")
    hardware_supervisor_launch = os.path.join(
        hardware_supervisor_dir,
        "launch",
        "hardware_supervisor.launch.py"
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_supervisor_launch)
        ),
    ])
