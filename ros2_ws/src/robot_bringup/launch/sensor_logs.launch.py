from datetime import datetime
import os
import shlex

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo


def topic_logger(topic, msg_type, output_file):
    command = (
        f"ros2 topic echo --csv {shlex.quote(topic)} {shlex.quote(msg_type)} "
        f"> {shlex.quote(output_file)}"
    )
    return ExecuteProcess(
        cmd=["bash", "-lc", command],
        output="screen",
    )


def generate_launch_description():
    session = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = os.path.join("/tmp/brt7k-sensor-logs", session)
    os.makedirs(log_dir, exist_ok=True)

    return LaunchDescription([
        LogInfo(msg=f"Sensor-Logfiles: {log_dir}"),
        topic_logger(
            "/imu/data_raw",
            "sensor_msgs/msg/Imu",
            os.path.join(log_dir, "imu_raw.csv"),
        ),
        topic_logger(
            "/odom",
            "nav_msgs/msg/Odometry",
            os.path.join(log_dir, "ekf_odom.csv"),
        ),
    ])
