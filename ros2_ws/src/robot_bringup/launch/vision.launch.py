from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    camera_dir = get_package_share_directory("camera")
    corner_manager_dir = get_package_share_directory("corner_manager")
    object_manager_dir = get_package_share_directory("object_manager")
    object_task_executor_dir = get_package_share_directory("object_task_executor")
    room_vision_dir = get_package_share_directory("room_vision")
    object_localizer_dir = get_package_share_directory("object_localizer")
    edge_color_dir = get_package_share_directory("edge_color")

    camera_launch = os.path.join(camera_dir, "launch", "camera.launch.py")
    corner_manager_launch = os.path.join(corner_manager_dir, "launch", "corner_manager.launch.py")
    object_manager_launch = os.path.join(object_manager_dir, "launch", "object_manager.launch.py")
    object_task_executor_launch = os.path.join(object_task_executor_dir, "launch", "object_task_executor.launch.py")
    room_vision_launch = os.path.join(room_vision_dir, "launch", "room_vision.launch.py")
    detector_launch = os.path.join(room_vision_dir, "launch", "detector.launch.py")
    object_localizer_launch = os.path.join(object_localizer_dir, "launch", "object_localizer.launch.py")
    edge_color_launch = os.path.join(edge_color_dir, "launch", "edge_color.launch.py")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(corner_manager_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(object_manager_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(object_task_executor_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(room_vision_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(detector_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(object_localizer_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(edge_color_launch)
        ),
    ])
