from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    web_dir = os.path.join(
        get_package_share_directory("robot_web"),
        "web"
    )

    return LaunchDescription([
        Node(
            package="rosbridge_server",
            executable="rosbridge_websocket",
            name="rosbridge_websocket",
            output="screen",
            parameters=[{
                "port": 9090
            }]
        ),

        ExecuteProcess(
    cmd=[
        "python3",
        "-m",
        "http.server",
        "8080",
        "--bind",
        "0.0.0.0",
        "--directory",
        web_dir
    ],
    output="screen"
)
    ])