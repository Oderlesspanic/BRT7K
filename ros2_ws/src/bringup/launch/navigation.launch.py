from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    nav2_params = LaunchConfiguration("nav2_params")
    amcl_params = LaunchConfiguration("amcl_params")

    bringup_dir = get_package_share_directory("bringup")
    robot_description_dir = get_package_share_directory("robot_description")
    odometry_dir = get_package_share_directory("odometry")
    sllidar_dir = get_package_share_directory("sllidar_ros2")

    default_map = "/ros2_ws/maps/meine_karte.yaml"
    default_nav2_params = os.path.join(bringup_dir, "config", "nav2_params.yaml")
    default_amcl_params = os.path.join(bringup_dir, "config", "amcl.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("nav2_params", default_value=default_nav2_params),
        DeclareLaunchArgument("amcl_params", default_value=default_amcl_params),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_description_dir, "launch", "rsp.launch.py")
            ),
            launch_arguments={"use_sim_time": use_sim_time}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sllidar_dir, "launch", "sllidar_launch.py")
            )
        ),

        Node(
            package="odometry",
            executable="wheel_odometry_node",
            name="wheel_odometry_node",
            output="screen"
        ),

        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                nav2_params,
                {"yaml_filename": map_file}
            ]
        ),

        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[amcl_params]
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "autostart": True,
                "node_names": ["map_server", "amcl"]
            }]
        ),

        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[nav2_params]
        ),

        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_params]
        ),

        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[nav2_params]
        ),

        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[nav2_params]
        ),

        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[nav2_params]
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "autostart": True,
                "node_names": [
                    "controller_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                    "waypoint_follower"
                ]
            }]
        ),
        
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen"
        )
    ])