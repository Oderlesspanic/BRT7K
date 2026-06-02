from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    sllidar_dir = get_package_share_directory("sllidar_ros2")
    sllidar_launch = os.path.join(
        sllidar_dir,
        "launch",
        "sllidar_c1_launch.py"
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch),
            launch_arguments={
                "serial_port": "/dev/lidar",
                "serial_baudrate": "460800",
                "frame_id": "laser_link",
                "inverted": "true",
                "scan_mode": "",
                "scan_frequency": "5"
            }.items()
        ),
    ])
