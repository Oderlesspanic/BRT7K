from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    imu_dir = get_package_share_directory("imu")
    imu_launch = os.path.join(
        imu_dir,
        "launch",
        "imu.launch.py"
    )

    mmc5603_dir = get_package_share_directory("mmc5603")
    mmc5603_launch = os.path.join(
        mmc5603_dir,
        "launch",
        "mmc5603.launch.py"
    )

    ekf_dir = get_package_share_directory("ekf")
    ekf_launch = os.path.join(
        ekf_dir,
        "launch",
        "ekf.launch.py"
    )


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mmc5603_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_launch)
        ),
    ])
