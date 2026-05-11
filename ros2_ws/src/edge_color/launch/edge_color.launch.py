from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('edge_color'),
        'config',
        'edge_color.yaml'
    )

    color_edge_detection_node = Node(
        package='edge_color',
        executable='color_edge_detection_node',
        name='color_edge_detection_node',
        output='screen',
        parameters=[config]
    )

    color_wall_projector_node = Node(
        package='edge_color',
        executable='color_wall_projector_node',
        name='color_wall_projector_node',
        output='screen',
        parameters=[config]
    )

    color_corner_validator_node = Node(
        package='edge_color',
        executable='color_corner_validator_node',
        name='color_corner_validator_node',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([
        color_edge_detection_node,
        color_wall_projector_node,
        color_corner_validator_node
    ])