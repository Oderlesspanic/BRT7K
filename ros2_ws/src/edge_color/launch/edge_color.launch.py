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

    color_edge_mapper_node = Node(
        package='edge_color',
        executable='color_edge_mapper_node',
        name='color_edge_mapper_node',
        output='screen',
        parameters=[config]
    )

    semantic_color_edge_map_manager_node = Node(
        package='edge_color',
        executable='semantic_color_edge_map_manager_node',
        name='semantic_color_edge_map_manager_node',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([
        color_edge_detection_node,
        color_edge_mapper_node,
        semantic_color_edge_map_manager_node
    ])