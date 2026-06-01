from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                "ros2", "run", "micro_ros_agent", "micro_ros_agent",
                "serial",
                "--dev", "/dev/esp_gripper",
                "-b", "115200"
            ],
            output="screen"
        ),
    ])
