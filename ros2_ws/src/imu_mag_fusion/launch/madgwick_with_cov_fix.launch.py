from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('imu_mag_fusion')

    madgwick_config = os.path.join(pkg_share, 'config', 'madgwick.yaml')
    cov_fix_config = os.path.join(pkg_share, 'config', 'imu_covariance_fixer.yaml')

    return LaunchDescription([
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[madgwick_config],
            remappings=[
                ('/imu/data_raw', '/imu/data_raw'),
                ('/imu/mag', '/mag/data_raw'),
                ('/imu/data', '/imu/data'),
            ]
        ),
        Node(
            package='imu_mag_fusion',
            executable='imu_covariance_fixer',
            name='imu_covariance_fixer',
            output='screen',
            parameters=[cov_fix_config]
        )
    ])