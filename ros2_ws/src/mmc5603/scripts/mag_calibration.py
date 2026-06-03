#!/usr/bin/env python3
"""
Magnetometer Hard-Iron Kalibrierung.

Dreht den Roboter 1.5 Runden, sammelt Magnetometerdaten und berechnet
daraus die Hard-Iron-Offsets (Ellipsenmittelpunkt). Die Werte werden
direkt in mmc5603.yaml geschrieben.

Starten:
  ros2 run mmc5603 mag_calibration

Optional: eigenen Config-Pfad angeben:
  ros2 run mmc5603 mag_calibration --ros-args -p config_path:=/pfad/zu/mmc5603.yaml
"""

import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import MagneticField

ANGULAR_SPEED_RAD_S = 0.3
ROTATIONS = 1.5
ROTATION_DURATION_S = (2.0 * math.pi * ROTATIONS) / ANGULAR_SPEED_RAD_S

# Muss mit mmc5603_convert.cpp übereinstimmen
RAW_ZERO = 524288.0
RAW_TO_UTESLA = 3000.0 / RAW_ZERO


class MagCalibrationNode(Node):

    def __init__(self) -> None:
        super().__init__('mag_calibration')

        self.declare_parameter(
            'config_path',
            os.path.join(
                get_package_share_directory('mmc5603'),
                'config',
                'mmc5603.yaml',
            ),
        )

        self._config_path: str = self.get_parameter('config_path').get_parameter_value().string_value

        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._mag_sub = self.create_subscription(
            MagneticField, '/mag/data_raw', self._mag_callback, 20
        )

        self._samples_x: list[float] = []
        self._samples_y: list[float] = []
        self._collecting = True
        self._start_time = self.get_clock().now()

        self._timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            f'Starte Kalibrierung: {ROTATIONS} Umdrehungen bei '
            f'{ANGULAR_SPEED_RAD_S} rad/s ({ROTATION_DURATION_S:.1f} s)'
        )

    def _mag_callback(self, msg: MagneticField) -> None:
        if self._collecting:
            self._samples_x.append(msg.magnetic_field.x)
            self._samples_y.append(msg.magnetic_field.y)

    def _control_loop(self) -> None:
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9

        if elapsed < ROTATION_DURATION_S:
            twist = Twist()
            twist.angular.z = ANGULAR_SPEED_RAD_S
            self._cmd_vel_pub.publish(twist)
            remaining = ROTATION_DURATION_S - elapsed
            if int(elapsed) % 5 == 0 and elapsed > 0:
                self.get_logger().info(
                    f'Sammle Daten ... {remaining:.0f} s verbleibend '
                    f'({len(self._samples_x)} Samples)',
                    throttle_duration_sec=5.0,
                )
        else:
            self._collecting = False
            self._cmd_vel_pub.publish(Twist())
            self._timer.cancel()
            self._compute_and_save()
            rclpy.shutdown()

    def _compute_and_save(self) -> None:
        n = len(self._samples_x)
        self.get_logger().info(f'Rotation abgeschlossen. {n} Samples gesammelt.')

        if n < 100:
            self.get_logger().error(
                f'Zu wenige Samples ({n}). Prüfe ob /mag/data_raw publiziert wird.'
            )
            return

        # Hard-Iron-Offset = Mittelpunkt der Daten-Ellipse
        center_x_t = (max(self._samples_x) + min(self._samples_x)) / 2.0
        center_y_t = (max(self._samples_y) + min(self._samples_y)) / 2.0

        spread_x_ut = (max(self._samples_x) - min(self._samples_x)) * 1e6
        spread_y_ut = (max(self._samples_y) - min(self._samples_y)) * 1e6

        if spread_x_ut < 5.0 or spread_y_ut < 5.0:
            self.get_logger().warn(
                f'Geringe Spreizung: X={spread_x_ut:.1f} µT, Y={spread_y_ut:.1f} µT. '
                'Hat sich der Roboter genug gedreht?'
            )

        # Umrechnung Tesla → Raw-ADC-Einheiten (für mmc5603_convert.cpp)
        offset_x_raw = center_x_t / (RAW_TO_UTESLA * 1e-6)
        offset_y_raw = center_y_t / (RAW_TO_UTESLA * 1e-6)

        self.get_logger().info(
            f'Ellipse X: [{min(self._samples_x)*1e6:.1f}, {max(self._samples_x)*1e6:.1f}] µT  '
            f'→ Mitte: {center_x_t*1e6:.2f} µT  →  offset_x: {offset_x_raw:.1f}'
        )
        self.get_logger().info(
            f'Ellipse Y: [{min(self._samples_y)*1e6:.1f}, {max(self._samples_y)*1e6:.1f}] µT  '
            f'→ Mitte: {center_y_t*1e6:.2f} µT  →  offset_y: {offset_y_raw:.1f}'
        )

        if not os.path.isfile(self._config_path):
            self.get_logger().error(f'Config-Datei nicht gefunden: {self._config_path}')
            return

        with open(self._config_path, 'r') as f:
            config = yaml.safe_load(f)

        params = config['mmc5603_node']['ros__parameters']
        params['offset_x'] = round(offset_x_raw, 1)
        params['offset_y'] = round(offset_y_raw, 1)

        with open(self._config_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False, allow_unicode=True, sort_keys=False)

        self.get_logger().info(f'Kalibrierung gespeichert: {self._config_path}')
        self.get_logger().info('Starte den mmc5603_node neu damit die Werte aktiv werden.')


def main() -> None:
    rclpy.init()
    node = MagCalibrationNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()