from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory('ekf'),
        'config',
        'ekf.yaml'
    )

    imu_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        ekf_node
    ])