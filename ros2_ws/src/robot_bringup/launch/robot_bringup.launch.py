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

    imu_launch = os.path.join(
        get_package_share_directory("imu"),
        "launch",
        "imu.launch.py"
    )

    magnetometer_launch = os.path.join(
        get_package_share_directory("mmc5603"),
        "launch",
        "mmc5603.launch.py"
    )

    imu_fusion_launch = os.path.join(
        get_package_share_directory("imu_mag_fusion"),
        "launch",
        "madgwick_with_cov_fix.launch.py"
    )

    ekf_launch = os.path.join(
        get_package_share_directory("ekf"),
        "launch",
        "ekf.launch.py"
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
            PythonLaunchDescriptionSource(imu_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(magnetometer_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_fusion_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_launch)
        )
    ])
