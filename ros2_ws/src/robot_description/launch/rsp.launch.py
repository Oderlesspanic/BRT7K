from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_width = LaunchConfiguration("wheel_width")
    wheel_y = LaunchConfiguration("wheel_y")

    robot_description_content = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([
                FindPackageShare("robot_description"),
                "urdf",
                "robot.urdf.xacro"
            ]),
            " wheel_radius:=", wheel_radius,
            " wheel_width:=", wheel_width,
            " wheel_y:=", wheel_y
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_content
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument("wheel_radius", default_value="0.03660"),
        DeclareLaunchArgument("wheel_width", default_value="0.0200"),
        DeclareLaunchArgument("wheel_y", default_value="0.1610"),
        robot_state_publisher_node
    ])
