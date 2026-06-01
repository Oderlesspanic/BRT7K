from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    robot_bringup_dir = get_package_share_directory("robot_bringup")
    lidar_launch = os.path.join(robot_bringup_dir, "launch", "lidar.launch.py")
    hardware_supervisor_launch = os.path.join(
        robot_bringup_dir,
        "launch",
        "hardware_supervisor.launch.py"
    )
    esp_drive_agent_launch = os.path.join(robot_bringup_dir, "launch", "esp_drive_agent.launch.py")
    esp_gripper_agent_launch = os.path.join(robot_bringup_dir, "launch", "esp_gripper_agent.launch.py")

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_supervisor_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(esp_drive_agent_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(esp_gripper_agent_launch)
        ),
    ])
