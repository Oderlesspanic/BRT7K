from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    wheel_odometry_dir = get_package_share_directory("odometry")
    wheel_odometry_launch = os.path.join(
        wheel_odometry_dir,
        "launch",
        "wheel_odometry.launch.py"
    )

    ekf_dir = get_package_share_directory("ekf")
    ekf_launch = os.path.join(
        ekf_dir,
        "launch",
        "ekf.launch.py"
    )


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(wheel_odometry_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_launch)
        ),
    ])
