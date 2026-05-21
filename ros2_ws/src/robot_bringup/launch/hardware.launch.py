from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    sllidar_dir = get_package_share_directory("sllidar_ros2")

    sllidar_launch = os.path.join(
        sllidar_dir,
        "launch",
        "sllidar_a1_launch.py"
    )

    hardware_supervisor_dir = get_package_share_directory("hardware_supervisor")

    hardware_supervisor_launch = os.path.join(
        hardware_supervisor_dir,
        "launch",
        "hardware_supervisor.launch.py"
    )

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch),
            launch_arguments={
                "serial_port": "/dev/ttyUSB0",
                "serial_baudrate": "115200",
                "frame_id": "laser"
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_supervisor_launch)
        ),

        ExecuteProcess(
            cmd=[
                "ros2", "run", "micro_ros_agent", "micro_ros_agent",
                "serial",
                "--dev", "/dev/esp_drive",
                "-b", "115200"
            ],
            output="screen"
        ),

        ExecuteProcess(
            cmd=[
                "ros2", "run", "micro_ros_agent", "micro_ros_agent",
                "serial",
                "--dev", "/dev/esp_gripper",
                "-b", "115200"
            ],
            output="screen"
        ),

        ExecuteProcess(
            cmd=[
                "ros2", "run", "micro_ros_agent", "micro_ros_agent",
                "serial",
                "--dev", "/dev/esp_monitor",
                "-b", "115200"
            ],
            output="screen"
        ),
    ])