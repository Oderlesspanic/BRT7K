from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("camera"),
        "config",
        "camera.yaml"
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name="LD_LIBRARY_PATH",
            value=[
                "/usr/local/lib/aarch64-linux-gnu:",
                EnvironmentVariable("LD_LIBRARY_PATH", default_value="")
            ]
        ),
        SetEnvironmentVariable(
            name="LIBCAMERA_IPA_MODULE_PATH",
            value="/usr/local/lib/aarch64-linux-gnu/libcamera/ipa"
        ),
        SetEnvironmentVariable(
            name="LIBCAMERA_DATA_DIR",
            value="/usr/local/share/libcamera"
        ),
        Node(
            package='camera',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[config_file]
        )
    ])
