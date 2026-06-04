from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('imu'),
        'config',
        'mpu6050.yaml'
    )

    imu_node = Node(
        package='imu',
        executable='imu_node',
        name='mpu6050_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        imu_node
    ])
