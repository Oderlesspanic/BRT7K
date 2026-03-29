from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_width = LaunchConfiguration("wheel_width")
    wheel_y = LaunchConfiguration("wheel_y")
    wheel_separation = LaunchConfiguration("wheel_separation")


    robot_description_pkg = get_package_share_directory("robot_description")
    imu_pkg = get_package_share_directory("imu")
    lidar_pkg = get_package_share_directory("sllidar_ros2")


    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_description_pkg, "launch", "rsp.launch.py")
        ),
        launch_arguments={
            "wheel_radius": wheel_radius,
            "wheel_width": wheel_width,
            "wheel_y": wheel_y
        }.items()
    )

    # IMU Launch
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imu_pkg, "launch", "imu.launch.py")
        )
    )

    # LiDAR Launch
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_pkg, "launch", "sllidar_c1_launch.py")
        )
    )

    # Wheel Odometry Node
    wheel_odometry_node = Node(
        package="odometry",
        executable="wheel_odometry_node",
        name="wheel_odometry_node",
        output="screen",
        parameters=[{
            "left_wheel_joint": "left_wheel_link_joint",
            "right_wheel_joint": "right_wheel_link_joint",
            "joint_state_topic": "/joint_states",
            "odom_topic": "/wheel/odometry",
            "odom_frame": "odom",
            "base_frame": "base_link",
            "wheel_radius": wheel_radius,
            "wheel_separation": wheel_separation,
            "publish_tf": True
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "wheel_radius",
            default_value="0.0745"
        ),
        DeclareLaunchArgument(
            "wheel_width",
            default_value="0.02"
        ),
        DeclareLaunchArgument(
            "wheel_y",
            default_value="0.11"
        ),
        DeclareLaunchArgument(
            "wheel_separation",
            default_value="0.22"
        ),

        robot_launch,
        imu_launch,
        lidar_launch,
        wheel_odometry_node,
    ])