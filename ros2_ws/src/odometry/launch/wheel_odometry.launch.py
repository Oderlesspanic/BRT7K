from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")

    robot_description = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([
                FindPackageShare("robot_description"),
                "urdf",
                "robot.urdf.xacro"
            ]),
            " wheel_radius:=",
            wheel_radius,
            " wheel_separation:=",
            wheel_separation
        ]),
        value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument("wheel_radius", default_value="0.0525"),
        DeclareLaunchArgument("wheel_separation", default_value="0.30"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}]
        ),

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
                "publish_tf": True
            }]
        )
    ])