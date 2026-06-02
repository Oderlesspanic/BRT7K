from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def include_optional_gripper(context, *args, **kwargs):
    if not os.path.exists("/dev/esp_gripper"):
        return [LogInfo(msg="Skipping gripper micro-ROS agent: /dev/esp_gripper not present")]

    robot_bringup_dir = get_package_share_directory("robot_bringup")
    esp_gripper_agent_launch = os.path.join(
        robot_bringup_dir,
        "launch",
        "esp_gripper_agent.launch.py"
    )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(esp_gripper_agent_launch)
        )
    ]


def generate_launch_description():

    robot_bringup_dir = get_package_share_directory("robot_bringup")
    lidar_launch = os.path.join(robot_bringup_dir, "launch", "lidar.launch.py")
    hardware_supervisor_launch = os.path.join(
        robot_bringup_dir,
        "launch",
        "hardware_supervisor.launch.py"
    )
    esp_drive_agent_launch = os.path.join(robot_bringup_dir, "launch", "esp_drive_agent.launch.py")

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

        OpaqueFunction(function=include_optional_gripper),
    ])
