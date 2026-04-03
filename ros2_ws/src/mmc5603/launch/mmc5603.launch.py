from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='mmc5603',
            executable='mmc5603_node',
            name='mmc5603_node',
            output='screen',
            parameters=[{
                "i2c_bus": "/dev/i2c-1",
                "device_address": 48,
                "frame_id": "mag_link",
                "publish_rate": 50.0
            }]
        )
    ])