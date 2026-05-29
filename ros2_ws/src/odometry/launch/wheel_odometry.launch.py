from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")

    return LaunchDescription([
        DeclareLaunchArgument("wheel_radius", default_value="0.0525"),
        DeclareLaunchArgument("wheel_separation", default_value="0.30"),

        Node(
            package="odometry",
            executable="wheel_odometry_node",
            name="wheel_odometry_node",
            parameters=[{
                "wheel_radius": wheel_radius,
                "wheel_separation": wheel_separation,
                "left_wheel_joint": "left_wheel_joint",
                "right_wheel_joint": "right_wheel_joint",
                "odom_frame": "odom",
                "base_frame": "base_link",
                "publish_tf": False
            }]
        )
    ])
