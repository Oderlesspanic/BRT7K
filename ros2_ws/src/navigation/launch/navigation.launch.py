import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    params_file = os.path.join(
        get_package_share_directory("navigation"),
        "config",
        "nav2_params.yaml"
    )

    map_file = LaunchConfiguration("map")
    default_map_dir = os.environ.get(
        "BRT7K_MAP_DIR",
        os.path.join(os.path.expanduser("~"), "BRT7K", "ros2_ws", "maps", "map"),
    )
    default_map_file = os.environ.get(
        "BRT7K_MAP_FILE",
        os.path.join(default_map_dir, "arena_map.yaml"),
    )

    declare_map = DeclareLaunchArgument(
        "map",
        default_value=default_map_file,
        description="Full path to map yaml file"
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_file,
            "use_sim_time": "false",
            "params_file": params_file,
            "autostart": "true"
        }.items()
    )

    return LaunchDescription([
        declare_map,
        nav2_launch
    ])
