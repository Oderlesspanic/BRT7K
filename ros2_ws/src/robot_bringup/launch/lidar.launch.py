from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
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
        GroupAction(
            [
                SetRemap(src="scan", dst="scan_raw"),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(sllidar_launch),
                    launch_arguments={
                        "serial_port": "/dev/lidar",
                        "serial_baudrate": "460800",
                        "frame_id": "laser_link",
                        "inverted": "true",
                        "scan_mode": "",
                        "scan_frequency": "5",
                    }.items()
                ),
            ]
        ),
        Node(
            package="robot_bringup",
            executable="scan_timestamp_republisher.py",
            name="scan_timestamp_republisher",
            output="screen",
            parameters=[{
                "input_topic": "/scan_raw",
                "output_topic": "/scan",
            }],
        ),
    ])
