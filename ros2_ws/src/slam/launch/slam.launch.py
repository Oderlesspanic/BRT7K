from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")

    default_params_file = os.path.join(
        get_package_share_directory("slam"),
        "config",
        "slam_toolbox.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false"
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=default_params_file
        ),

        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[
                slam_params_file,
                {"use_sim_time": use_sim_time}
            ]
        )
    ])