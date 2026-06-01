from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    mapping_config = os.path.join(
        get_package_share_directory('mapping'),
        'config',
        'slam_toolbox.yaml'
    )

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[mapping_config]
        ),
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
                    output='screen'
                )
            ]
        ),
        TimerAction(
            period=4.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
                    output='screen'
                )
            ]
        )
    ])
