from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():


    robot_description_pkg = get_package_share_directory('robot_description')
    imu_pkg = get_package_share_directory('imu')
    lidar_pkg = get_package_share_directory('sllidar_ros2')


    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_description_pkg, 'launch', 'rsp.launch.py')
        )
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imu_pkg, 'launch', 'imu.launch.py')
        )
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_pkg, 'launch', 'sllidar_c1_launch.py')
        )
    )


    return LaunchDescription([
        robot_launch,
        imu_launch,
        lidar_launch,
    ])