import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    lifecycle_nodes = [
        "map_server",
        "amcl",
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "collision_monitor",
    ]

    default_map_dir = os.environ.get(
        "BRT7K_MAP_DIR",
        os.path.join(os.path.expanduser("~"), "BRT7K", "ros2_ws", "maps", "map"),
    )
    default_map_file = os.environ.get(
        "BRT7K_MAP_FILE",
        os.path.join(default_map_dir, "arena_map.yaml"),
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            get_package_share_directory("navigation"),
            "config",
            "nav2_params.yaml",
        ),
        description="Full path to the Nav2 parameter file",
    )

    declare_map = DeclareLaunchArgument(
        "map",
        default_value=default_map_file,
        description="Full path to map yaml file",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    return LaunchDescription([
        declare_params_file,
        declare_map,
        declare_use_sim_time,
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                params_file,
                {
                    "yaml_filename": map_file,
                    "use_sim_time": use_sim_time,
                },
            ],
            remappings=remappings,
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            remappings=remappings,
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            remappings=remappings,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            remappings=remappings,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            remappings=remappings,
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            remappings=remappings,
        ),
        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            remappings=remappings,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "autostart": True,
                    "bond_timeout": 30.0,
                    "node_names": lifecycle_nodes,
                }
            ],
        ),
    ])
