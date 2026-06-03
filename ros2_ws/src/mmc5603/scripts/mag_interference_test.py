#!/usr/bin/env python3
"""
Magnetometer Interferenz-Diagnose für MMC5603 (ROS2)

Workflow:
  1. Skript starten: python3 mag_interference_test.py
  2. Roboter von Störquellen fernhalten → Enter drücken für Baseline
  3. Roboter in die Nähe der Störquelle bringen → Enter drücken für Messung
  4. Ergebnis wird angezeigt

Benötigt: ros2 topic und sensor_msgs
Verwendung: ros2 run mmc5603 mag_interference_test
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField

import math
import sys
import threading
import time


class MagInterferenceTest(Node):

    def __init__(self):
        super().__init__('mag_interference_test')

        self._lock = threading.Lock()
        self._samples: list[tuple[float, float, float]] = []
        self._collecting = False

        self.sub = self.create_subscription(
            MagneticField,
            'mag/data_raw',
            self._mag_callback,
            10
        )

        self.get_logger().info("Magnetometer Interferenz-Test gestartet.")
        self.get_logger().info("Topic: mag/data_raw")

    def _mag_callback(self, msg: MagneticField):
        if not self._collecting:
            return
        x = msg.magnetic_field.x
        y = msg.magnetic_field.y
        z = msg.magnetic_field.z
        with self._lock:
            self._samples.append((x, y, z))

    def collect(self, label: str, duration_s: float = 5.0) -> dict:
        self.get_logger().info(
            f"[{label}] Sammle {duration_s:.0f}s Daten... Roboter jetzt positionieren!")

        with self._lock:
            self._samples.clear()
        self._collecting = True
        time.sleep(duration_s)
        self._collecting = False

        with self._lock:
            data = list(self._samples)

        if len(data) < 5:
            self.get_logger().error(
                f"[{label}] Zu wenige Samples ({len(data)}). "
                "Läuft der mmc5603_node? (ros2 topic echo /mag/data_raw)")
            return {}

        xs = [d[0] for d in data]
        ys = [d[1] for d in data]
        zs = [d[2] for d in data]
        norms = [math.sqrt(x**2 + y**2 + z**2) for x, y, z in data]

        def stats(vals):
            mean = sum(vals) / len(vals)
            std = math.sqrt(sum((v - mean) ** 2 for v in vals) / len(vals))
            return mean, std, min(vals), max(vals)

        mx, sx, minx, maxx = stats(xs)
        my, sy, miny, maxy = stats(ys)
        mz, sz, minz, maxz = stats(zs)
        mn, sn, minn, maxn = stats(norms)

        result = {
            'label': label,
            'n': len(data),
            'mean_x': mx, 'std_x': sx,
            'mean_y': my, 'std_y': sy,
            'mean_z': mz, 'std_z': sz,
            'mean_norm': mn, 'std_norm': sn,
            'min_norm': minn, 'max_norm': maxn,
        }

        T_to_uT = 1e6
        self.get_logger().info(
            f"\n{'='*55}\n"
            f"  {label} ({len(data)} Samples)\n"
            f"{'='*55}\n"
            f"  X:    {mx*T_to_uT:+8.3f} ±{sx*T_to_uT:.3f} µT\n"
            f"  Y:    {my*T_to_uT:+8.3f} ±{sy*T_to_uT:.3f} µT\n"
            f"  Z:    {mz*T_to_uT:+8.3f} ±{sz*T_to_uT:.3f} µT\n"
            f"  |B|:  {mn*T_to_uT:8.3f} ±{sn*T_to_uT:.3f} µT  "
            f"[{minn*T_to_uT:.3f} .. {maxn*T_to_uT:.3f}]\n"
            f"{'='*55}"
        )
        return result

    @staticmethod
    def compare(baseline: dict, test: dict):
        if not baseline or not test:
            return

        T_to_uT = 1e6
        dx = (test['mean_x'] - baseline['mean_x']) * T_to_uT
        dy = (test['mean_y'] - baseline['mean_y']) * T_to_uT
        dz = (test['mean_z'] - baseline['mean_z']) * T_to_uT
        dn = (test['mean_norm'] - baseline['mean_norm']) * T_to_uT

        noise_base = baseline['std_norm'] * T_to_uT
        noise_test = test['std_norm'] * T_to_uT

        print(f"\n{'='*55}")
        print(f"  VERGLEICH: {test['label']} vs. {baseline['label']}")
        print(f"{'='*55}")
        print(f"  ΔX:   {dx:+.3f} µT")
        print(f"  ΔY:   {dy:+.3f} µT")
        print(f"  ΔZ:   {dz:+.3f} µT")
        print(f"  Δ|B|: {dn:+.3f} µT")
        print(f"  Rauschen Baseline: {noise_base:.4f} µT  |  Test: {noise_test:.4f} µT")

        total_shift = math.sqrt(dx**2 + dy**2 + dz**2)
        print(f"\n  Gesamtversatz: {total_shift:.3f} µT", end="  →  ")

        if total_shift < 1.0:
            print("keine nennenswerte Interferenz")
        elif total_shift < 5.0:
            print("GERINGE Interferenz")
        elif total_shift < 20.0:
            print("MITTLERE Interferenz – Navigation prüfen!")
        else:
            print("STARKE Interferenz – Magnetometer unbrauchbar in dieser Umgebung!")

        print(f"{'='*55}\n")


def wait_for_enter(prompt: str):
    input(prompt)


def main():
    rclpy.init()
    node = MagInterferenceTest()

    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    try:
        wait_for_enter(
            "\n[1/3] Roboter WEIT WEG von Steckdosen/Geräten stellen. "
            "Dann ENTER drücken für Baseline-Messung...\n"
        )
        baseline = node.collect("Baseline (keine Störquelle)", duration_s=5.0)

        wait_for_enter(
            "\n[2/3] Roboter in die NÄHE der Störquelle bringen. "
            "Dann ENTER für Testmessung...\n"
        )
        test = node.collect("Test (Störquelle in der Nähe)", duration_s=5.0)

        MagInterferenceTest.compare(baseline, test)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
