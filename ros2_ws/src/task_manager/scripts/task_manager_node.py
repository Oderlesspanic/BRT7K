#!/usr/bin/env python3

"""Ein einfacher Task Manager Node, der mehrere ROS 2 Launch Dateien verwalten kann.
Er ermöglicht das Starten, Stoppen und Neustarten von vordefinierten Launch Dateien
über ROS 2 Services oder durch das Senden von Befehlen an ein Topic.

Die zu verwaltenden Launch Dateien werden über den Parameter 'launch_targets' definiert,
der eine Liste von Strings im Format 'name:package:launch_file[:arg:=value ...]' erwartet.
Beispiel:
    launch_targets:
        - description:robot_bringup:description.launch.py
        - web:robot_bringup:web.launch.py
        - hardware:robot_bringup:hardware.launch.py
        - vision:robot_bringup:vision.launch.py
        - odometry:robot_bringup:odometry.launch.py
        - mapping:robot_bringup:mapping.launch.py
        - navigation:navigation:navigation.launch.py
        - map_saver:mapping:map_saver.launch.py

Der Node bietet die folgenden ROS 2 Services:
- ~/status (Trigger): Gibt den aktuellen Status aller verwalteten Launch Dateien zurück.
- ~/start_all (Trigger): Startet alle verwalteten Launch Dateien.
- ~/stop_all (Trigger): Stoppt alle verwalteten Launch Dateien.
- ~/restart_all (Trigger): Startet alle verwalteten Launch Dateien neu.
- ~/save_map (Trigger): Speichert die aktuelle SLAM-Karte ohne Mapping zu beenden.
- ~/finish_mapping (Trigger): Speichert die Karte, beendet Mapping und startet Navigation.
- ~/stop_motion (Trigger): Stoppt aktive Navigationsziele und setzt cmd_vel auf 0.
- ~/start_<target> (Trigger): Startet die angegebene Launch Datei.
- ~/stop_<target> (Trigger): Stoppt die angegebene Launch Datei.
- ~/restart_<target> (Trigger): Startet die angegebene Launch Datei neu.

Der Node veroeffentlicht ausserdem regelmaessig den Status aller verwalteten Launch Dateien auf dem Topic '~/status_text' (String).
Beispielbefehle über das Topic '~/command':
- "status": Gibt den aktuellen Status aller Launch Dateien zurück.
- "start <target>": Startet die angegebene Launch Datei.
- "stop <target>": Stoppt die angegebene Launch Datei.
- "restart <target>": Startet die angegebene Launch Datei neu.
- "start all": Startet alle Launch Dateien.
- "stop all": Stoppt alle Launch Dateien.
- "restart all": Startet alle Launch Dateien neu.

Beispielaufrufe der Services:

ros2 service call /task_manager/start_description std_srvs/srv/Trigger {}
ros2 service call /task_manager/start_web std_srvs/srv/Trigger {}
ros2 service call /task_manager/start_hardware std_srvs/srv/Trigger {}
ros2 service call /task_manager/start_vision std_srvs/srv/Trigger {}
ros2 service call /task_manager/start_odometry std_srvs/srv/Trigger {}
ros2 service call /task_manager/start_mapping std_srvs/srv/Trigger {}
ros2 service call /task_manager/start_navigation std_srvs/srv/Trigger {}
ros2 service call /task_manager/start_map_saver std_srvs/srv/Trigger {}

ros2 service call /task_manager/stop_description std_srvs/srv/Trigger {}
ros2 service call /task_manager/stop_web std_srvs/srv/Trigger {}
ros2 service call /task_manager/stop_hardware std_srvs/srv/Trigger {}
ros2 service call /task_manager/stop_vision std_srvs/srv/Trigger {}
ros2 service call /task_manager/stop_odometry std_srvs/srv/Trigger {}
ros2 service call /task_manager/stop_mapping std_srvs/srv/Trigger {}
ros2 service call /task_manager/stop_navigation std_srvs/srv/Trigger {}
ros2 service call /task_manager/stop_map_saver std_srvs/srv/Trigger {}

ros2 service call /task_manager/status std_srvs/srv/Trigger {}
ros2 service call /task_manager/start_all std_srvs/srv/Trigger {}
ros2 service call /task_manager/stop_all std_srvs/srv/Trigger {}
ros2 service call /task_manager/restart_all std_srvs/srv/Trigger {}



"""


import os
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from action_msgs.srv import CancelGoal
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


@dataclass(frozen=True)
class LaunchTarget:
    name: str
    package: str
    launch_file: str
    arguments: Tuple[str, ...] = ()


class ManagedLaunch:
    def __init__(self, target: LaunchTarget) -> None:
        self.target = target
        self.process: Optional[subprocess.Popen] = None
        self.log_path: Optional[Path] = None

    def is_running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def returncode(self) -> Optional[int]:
        if self.process is None:
            return None
        return self.process.poll()


class TaskManagerNode(Node):
    START_ALL_DENYLIST = {
        "vision",
        "camera",
        "corner_manager",
        "object_manager",
        "object_task_executor",
        "room_vision",
        "object_localizer",
        "edge_color",
        "navigation",
        "navigation_debug",
        "navigation_slam",
        "frontier_explorer",
        "map_saver",
    }
    LOW_PRIORITY_TARGETS = {
        "vision",
        "camera",
        "corner_manager",
        "object_manager",
        "object_task_executor",
        "room_vision",
        "object_localizer",
        "edge_color",
    }
    NAVIGATION_SLAM_LIFECYCLE_NODES = (
        "/controller_server",
        "/planner_server",
        "/smoother_server",
        "/collision_monitor",
        "/behavior_server",
        "/bt_navigator",
        "/waypoint_follower",
    )

    def __init__(self) -> None:
        super().__init__("task_manager")

        self._targets = self._load_targets()
        self._managed: Dict[str, ManagedLaunch] = {
            target.name: ManagedLaunch(target) for target in self._targets
        }
        self._start_all_target_names = self._load_start_all_targets()
        self._autostart_enabled = self._load_autostart_enabled()
        self._autostart_delay_sec = self._load_autostart_delay()
        self._imu_startup_timeout_sec = self._load_imu_startup_timeout()
        self._imu_ready_topic = self._load_imu_ready_topic()
        self._autostart_done = False
        self._autostart_timer = None
        self._log_dir = self._load_log_dir()
        self._log_dir.mkdir(parents=True, exist_ok=True)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self._navigate_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
        )
        self._map_received_event = threading.Event()
        self._imu_ready_event = threading.Event()
        self._frontier_ready = False
        self._mapping_complete = False
        self._map_save_lock = threading.Lock()

        self._status_pub = self.create_publisher(String, "~/status_text", 10)
        self._frontier_ready_pub = self.create_publisher(String, "~/frontier_ready", 10)
        self._mapping_complete_pub = self.create_publisher(String, "~/mapping_complete", 10)
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            10,
        )
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._cmd_vel_nav_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self._command_sub = self.create_subscription(
            String,
            "~/command",
            self._handle_command,
            10,
        )
        self._map_sub = self.create_subscription(
            OccupancyGrid,
            "/map",
            self._handle_map,
            10,
        )
        self._imu_ready_sub = self.create_subscription(
            Imu,
            self._imu_ready_topic,
            self._handle_imu_ready,
            10,
        )

        self.create_service(Trigger, "~/status", self._status_service)
        self.create_service(Trigger, "~/start_all", self._start_all_service)
        self.create_service(Trigger, "~/stop_all", self._stop_all_service)
        self.create_service(Trigger, "~/restart_all", self._restart_all_service)
        self.create_service(Trigger, "~/save_map", self._save_map_service)
        self.create_service(Trigger, "~/finish_mapping", self._finish_mapping_service)
        self.create_service(Trigger, "~/stop_motion", self._stop_motion_service)

        for name in self._managed:
            self.create_service(
                Trigger,
                f"~/start_{name}",
                self._make_target_service(name, "start"),
            )
            self.create_service(
                Trigger,
                f"~/stop_{name}",
                self._make_target_service(name, "stop"),
            )
            self.create_service(
                Trigger,
                f"~/restart_{name}",
                self._make_target_service(name, "restart"),
            )

        self.create_timer(2.0, self._publish_status)
        if self._autostart_enabled:
            self._autostart_timer = self.create_timer(
                self._autostart_delay_sec,
                self._run_autostart_once,
            )

        target_names = ", ".join(self._managed.keys())
        self.get_logger().info(
            f"Task Manager gestartet. Targets: {target_names}. Logs: {self._log_dir}"
        )

    def _load_targets(self) -> List[LaunchTarget]:
        default_specs = [
            "description:robot_bringup:description.launch.py",
            "web:robot_bringup:web.launch.py",
            "hardware:robot_bringup:hardware.launch.py",
            "vision:robot_bringup:vision.launch.py",
            "odometry:robot_bringup:odometry.launch.py",
            "mapping:robot_bringup:mapping.launch.py",
            "slam:mapping:mapping.launch.py",
            "frontier_explorer:frontier_explorer:frontier_explorer.launch.py",
            "navigation:navigation:navigation.launch.py",
            "navigation_slam:navigation:navigation_slam.launch.py",
            "map_saver:mapping:map_saver.launch.py",
        ]

        self.declare_parameter("launch_targets", default_specs)
        specs = self.get_parameter("launch_targets").value

        targets: List[LaunchTarget] = []
        for spec in specs:
            parsed = self._parse_target_spec(str(spec))
            if parsed is None:
                self.get_logger().warn(
                    "Ignoriere ungueltiges launch_targets-Element: "
                    f"'{spec}'. Format: name:package:launch_file[:arg:=value ...]"
                )
                continue
            targets.append(parsed)

        return targets

    def _load_start_all_targets(self) -> List[str]:
        default_targets = [
            "web",
            "mapping",
        ]
        self.declare_parameter("start_all_targets", default_targets)
        configured_targets = [
            str(name) for name in self.get_parameter("start_all_targets").value
        ]
        start_targets = []
        for name in configured_targets:
            if name not in self._managed:
                continue

            if name in self.START_ALL_DENYLIST:
                self.get_logger().warn(
                    f"Ignoriere '{name}' in start_all_targets; "
                    "dieses Target wird nicht mit start_all gestartet"
                )
                continue

            start_targets.append(name)

        return start_targets

    def _load_autostart_enabled(self) -> bool:
        self.declare_parameter("autostart", False)
        return bool(self.get_parameter("autostart").value)

    def _load_autostart_delay(self) -> float:
        self.declare_parameter("autostart_delay_sec", 3.0)
        return max(0.1, float(self.get_parameter("autostart_delay_sec").value))

    def _load_imu_startup_timeout(self) -> float:
        self.declare_parameter("imu_startup_timeout_sec", 20.0)
        return max(0.0, float(self.get_parameter("imu_startup_timeout_sec").value))

    def _load_imu_ready_topic(self) -> str:
        self.declare_parameter("imu_ready_topic", "/imu/data_raw")
        return str(self.get_parameter("imu_ready_topic").value)

    def _load_log_dir(self) -> Path:
        default_log_dir = os.environ.get("BRT7K_TASK_LOG_DIR", "/tmp/brt7k-task-manager")
        self.declare_parameter("log_dir", default_log_dir)
        return Path(str(self.get_parameter("log_dir").value))

    def _parse_target_spec(self, spec: str) -> Optional[LaunchTarget]:
        parts = spec.split(":", maxsplit=3)
        if len(parts) < 3:
            return None

        name = parts[0].strip()
        package = parts[1].strip()
        launch_file = parts[2].strip()
        if not name or not package or not launch_file:
            return None

        arguments: Tuple[str, ...] = ()
        if len(parts) == 4 and parts[3].strip():
            arguments = tuple(parts[3].split())

        return LaunchTarget(name, package, launch_file, arguments)

    def _make_target_service(self, target_name: str, action: str):
        def callback(request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
            del request
            success, message = self._execute_action(action, target_name)
            response.success = success
            response.message = message
            return response

        return callback

    def _status_service(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        response.success = True
        response.message = self._status_text()
        return response

    def _start_all_service(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        results = self._start_all_sequence()
        response.success = True
        response.message = "\n".join(results)
        return response

    def _run_autostart_once(self) -> None:
        if self._autostart_done:
            return

        self._autostart_done = True
        if self._autostart_timer is not None:
            self._autostart_timer.cancel()

        threading.Thread(
            target=self._autostart_sequence,
            daemon=True,
        ).start()

    def _autostart_sequence(self) -> None:
        self.get_logger().info(
            "Autostart startet Targets: "
            + ", ".join(self._start_all_target_names)
        )
        for target_name in self._start_all_target_names:
            success, message = self._start_target_with_imu_barrier(target_name)
            if success:
                self.get_logger().info(message)
            else:
                self.get_logger().error(message)

    def _start_all_sequence(self) -> List[str]:
        return [
            self._start_target_with_imu_barrier(name)[1]
            for name in self._start_all_target_names
        ]

    def _start_target_with_imu_barrier(self, target_name: str) -> Tuple[bool, str]:
        if target_name in {"imu", "odometry"}:
            self._imu_ready_event.clear()

        success, message = self._execute_action("start", target_name)
        if target_name not in {"imu", "odometry"} or not success or self._imu_startup_timeout_sec <= 0.0:
            return success, message

        block_message = (
            f"Warte auf erste IMU-Nachricht auf {self._imu_ready_topic}; "
            f"Timeout {self._imu_startup_timeout_sec:.1f}s"
        )
        self.get_logger().info(block_message)
        if self._imu_ready_event.wait(timeout=self._imu_startup_timeout_sec):
            ready_message = "IMU publiziert; weitere Starts werden freigegeben"
            self.get_logger().info(ready_message)
            return success, f"{message}\n{block_message}\n{ready_message}"

        timeout_message = (
            "Timeout beim Warten auf IMU-Publish; weitere Starts werden trotzdem freigegeben"
        )
        self.get_logger().warn(timeout_message)
        return success, f"{message}\n{block_message}\n{timeout_message}"

    def _stop_all_service(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        results = [self._stop_target(name)[1] for name in reversed(list(self._managed.keys()))]
        response.success = True
        response.message = "\n".join(results)
        return response

    def _restart_all_service(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        stop_results = [self._stop_target(name)[1] for name in reversed(list(self._managed.keys()))]
        start_results = self._start_all_sequence()
        response.success = True
        response.message = "\n".join(stop_results + start_results)
        return response

    def _finish_mapping_service(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        threading.Thread(target=self._finish_mapping_sequence, daemon=True).start()
        response.success = True
        response.message = "Mapping-Abschlusssequenz gestartet"
        return response

    def _save_map_service(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        if self._map_save_lock.locked():
            response.success = False
            response.message = "Map-Save laeuft bereits"
            return response

        threading.Thread(target=self._save_map_sequence, daemon=True).start()
        response.success = True
        response.message = "Map-Save gestartet"
        return response

    def _stop_motion_service(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        threading.Thread(target=self._stop_motion_sequence, daemon=True).start()
        response.success = True
        response.message = "Stop-Motion gestartet"
        return response

    def _stop_motion_sequence(self) -> None:
        if "frontier_explorer" in self._managed:
            self._stop_target("frontier_explorer")

        cancel_success, cancel_message = self._cancel_navigate_to_pose_goals()
        if cancel_success:
            self.get_logger().info(cancel_message)
        else:
            self.get_logger().warn(cancel_message)

        self._publish_zero_velocity_burst()

    def _cancel_navigate_to_pose_goals(self) -> Tuple[bool, str]:
        client = self.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")
        if not client.wait_for_service(timeout_sec=2.0):
            return True, "Kein aktiver /navigate_to_pose Cancel-Service gefunden"

        request = CancelGoal.Request()
        future = client.call_async(request)
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)

        if not future.done():
            return False, "Timeout beim Cancel von /navigate_to_pose Goals"

        response = future.result()
        if response is None:
            return False, "Keine Antwort beim Cancel von /navigate_to_pose Goals"

        return True, f"/navigate_to_pose Cancel-Code: {response.return_code}"

    def _publish_zero_velocity_burst(self) -> None:
        twist = Twist()
        for _ in range(10):
            self._cmd_vel_pub.publish(twist)
            self._cmd_vel_nav_pub.publish(twist)
            time.sleep(0.1)

    def _finish_mapping_sequence(self) -> None:
        self.get_logger().info("Mapping abgeschlossen: speichere Map")

        save_success, save_message = self._run_map_saver_once(timeout_sec=60.0)
        if save_success:
            self.get_logger().info(save_message)
            self._set_mapping_complete(True)
        else:
            self.get_logger().error(save_message)
            return

        if "navigation" not in self._managed:
            self.get_logger().error("Target 'navigation' ist nicht im Taskmanager konfiguriert")
            return

        initial_pose_tf = self._lookup_current_robot_pose_in_map()

        self.get_logger().info("Stoppe Mapping/SLAM/Frontier/Nav2-SLAM")
        if "frontier_explorer" in self._managed:
            self._stop_target("frontier_explorer")
        if "navigation_slam" in self._managed:
            self._stop_target("navigation_slam")
        self._stop_target("mapping")

        self.get_logger().info("Starte Navigation mit gespeicherter Map")
        nav_success, nav_message = self._start_target("navigation")
        if nav_success:
            self.get_logger().info(nav_message)
            if initial_pose_tf is not None:
                self._publish_initial_pose_sequence(initial_pose_tf)
            else:
                self.get_logger().warn(
                    "Keine map->base_link Pose vor SLAM-Stop gefunden; "
                    "AMCL Initialpose muss manuell gesetzt werden"
                )
        else:
            self.get_logger().error(nav_message)

    def _save_map_sequence(self) -> None:
        success, message = self._run_map_saver_once(timeout_sec=60.0)
        if success:
            self.get_logger().info(message)
            self._set_mapping_complete(True)
        else:
            self.get_logger().error(message)

    def _run_map_saver_once(self, timeout_sec: float) -> Tuple[bool, str]:
        if not self._map_save_lock.acquire(blocking=False):
            return False, "Map-Save laeuft bereits"

        try:
            return self._run_map_saver_once_locked(timeout_sec=timeout_sec)
        finally:
            self._map_save_lock.release()

    def _run_map_saver_once_locked(self, timeout_sec: float) -> Tuple[bool, str]:
        if "map_saver" not in self._managed:
            return False, "Target 'map_saver' ist nicht im Taskmanager konfiguriert"

        if "mapping" not in self._managed:
            return False, "Target 'mapping' ist nicht im Taskmanager konfiguriert"

        self._prepare_stable_map_save()
        paused_measurements = self._set_slam_measurements_paused(True)

        try:
            success, message = self._start_target("map_saver")
            if not success:
                return False, message

            self.get_logger().info(message)

            map_saver = self._managed["map_saver"]
            deadline = time.monotonic() + timeout_sec
            while map_saver.is_running() and time.monotonic() < deadline:
                time.sleep(0.5)

            if map_saver.is_running():
                return False, "Map saver Timeout; Mapping bleibt aktiv"

            return_code = map_saver.returncode()
            if return_code not in (0, None):
                return (
                    False,
                    f"Map saver fehlgeschlagen mit Returncode {return_code}; Mapping bleibt aktiv",
                )

            self.get_logger().info("Map gespeichert; stoppe Map Saver")
            self._stop_target("map_saver")
            return True, "Map gespeichert"
        finally:
            if paused_measurements:
                self._set_slam_measurements_paused(False)

    def _prepare_stable_map_save(self) -> None:
        if self._is_target_running("frontier_explorer"):
            success, message = self._stop_target("frontier_explorer")
            if success:
                self.get_logger().info("Frontier vor Map-Save gestoppt: " + message)
            else:
                self.get_logger().warn("Frontier konnte vor Map-Save nicht gestoppt werden: " + message)

        self._cancel_navigate_to_pose_goals()
        self._publish_zero_velocity_burst()
        time.sleep(2.0)

    def _set_slam_measurements_paused(self, paused: bool) -> bool:
        command = [
            "ros2",
            "service",
            "call",
            "/slam_toolbox/pause_new_measurements",
            "slam_toolbox/srv/Pause",
            "{pause: " + ("true" if paused else "false") + "}",
        ]

        try:
            result = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=5.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.get_logger().warn(
                f"slam_toolbox pause_new_measurements nicht verfuegbar: {exc}"
            )
            return False

        if result.returncode != 0:
            self.get_logger().warn(
                "slam_toolbox pause_new_measurements fehlgeschlagen: "
                + (result.stdout or "").strip()
            )
            return False

        state = "pausiert" if paused else "fortgesetzt"
        self.get_logger().info(f"slam_toolbox Messungen {state}")
        return True

    def _handle_command(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if not command:
            return

        parts = command.split()
        if parts[0] == "status":
            self.get_logger().info(self._status_text())
            return

        if len(parts) != 2 or parts[0] not in {"start", "stop", "restart"}:
            self.get_logger().warn(
                "Ungueltiger Befehl. Nutze: start <target>, stop <target>, "
                "restart <target>, start all, stop all, restart all oder status"
            )
            return

        success, message = self._execute_action(parts[0], parts[1])
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().warn(message)

    def _execute_action(self, action: str, target_name: str) -> Tuple[bool, str]:
        if target_name == "all":
            if action == "start":
                messages = [
                    self._execute_action("start", name)[1]
                    for name in self._start_all_target_names
                ]
                return True, "\n".join(messages)
            if action == "stop":
                messages = [
                    self._stop_target(name)[1] for name in reversed(list(self._managed.keys()))
                ]
                return True, "\n".join(messages)
            messages = [
                self._stop_target(name)[1] for name in reversed(list(self._managed.keys()))
            ]
            messages.extend(
                self._execute_action("start", name)[1]
                for name in self._start_all_target_names
            )
            return True, "\n".join(messages)

        if target_name not in self._managed:
            known = ", ".join(self._managed.keys())
            return False, f"Unbekanntes Target '{target_name}'. Bekannt: {known}"

        if action == "start":
            if target_name == "mapping":
                return self._start_mapping_with_prerequisites()
            if target_name == "slam":
                return self._start_slam_target()
            if target_name == "navigation_slam":
                return self._start_navigation_slam_target()
            if target_name == "frontier_explorer":
                return self._start_frontier_explorer_target()
            return self._start_target(target_name)
        if action == "stop":
            return self._stop_target(target_name)
        return self._restart_target(target_name)

    def _start_target(self, target_name: str) -> Tuple[bool, str]:
        managed = self._managed[target_name]
        if managed.is_running():
            return True, f"{target_name} laeuft bereits"

        if managed.process is not None:
            return_code = managed.returncode()
            if return_code not in (None, 0):
                self.get_logger().warn(
                    f"{target_name} war beendet mit Returncode {return_code}; starte neu"
                )

        command = [
            "ros2",
            "launch",
            managed.target.package,
            managed.target.launch_file,
            *managed.target.arguments,
        ]
        if target_name in self.LOW_PRIORITY_TARGETS:
            command = ["nice", "-n", "10", *command]

        log_path = self._log_dir / f"{target_name}.log"
        managed.log_path = log_path

        try:
            log_file = log_path.open("ab")
            log_file.write(f"\n\n===== start {time.strftime('%Y-%m-%d %H:%M:%S')} =====\n".encode())
            log_file.write(("command: " + " ".join(command) + "\n").encode())
            log_file.flush()
            managed.process = subprocess.Popen(
                command,
                start_new_session=True,
                stdout=log_file,
                stderr=subprocess.STDOUT,
            )
            log_file.close()
        except OSError as exc:
            managed.process = None
            return False, f"{target_name} konnte nicht gestartet werden: {exc}"

        time.sleep(0.5)
        return_code = managed.returncode()
        if return_code not in (None, 0):
            managed.process = None
            return (
                False,
                f"{target_name} ist direkt beendet (Returncode {return_code}). "
                f"Log: {log_path}",
            )

        return True, f"{target_name} gestartet: {' '.join(command)}\nLog: {log_path}"

    def _stop_target(self, target_name: str) -> Tuple[bool, str]:
        if target_name in {"mapping", "slam", "navigation_slam", "frontier_explorer"}:
            self._set_frontier_ready(False)

        managed = self._managed[target_name]
        process = managed.process

        if process is None:
            return True, f"{target_name} ist nicht gestartet"

        if process.poll() is not None:
            return_code = process.returncode
            managed.process = None
            return True, f"{target_name} ist bereits beendet (Returncode {return_code})"

        pid = process.pid
        sigint_timeout = 25.0 if target_name == "navigation_slam" else 8.0
        sigterm_timeout = 10.0 if target_name == "navigation_slam" else 4.0
        try:
            os.killpg(pid, signal.SIGINT)
            process.wait(timeout=sigint_timeout)
        except subprocess.TimeoutExpired:
            self.get_logger().warn(
                f"{target_name} reagiert nicht auf SIGINT; sende SIGTERM"
            )
            try:
                os.killpg(pid, signal.SIGTERM)
                process.wait(timeout=sigterm_timeout)
            except subprocess.TimeoutExpired:
                self.get_logger().error(
                    f"{target_name} reagiert nicht auf SIGTERM; sende SIGKILL"
                )
                os.killpg(pid, signal.SIGKILL)
                process.wait(timeout=2.0)
        except ProcessLookupError:
            pass
        finally:
            managed.process = None

        return True, f"{target_name} beendet"

    def _is_target_running(self, target_name: str) -> bool:
        managed = self._managed.get(target_name)
        return managed is not None and managed.is_running()

    def _restart_target(self, target_name: str) -> Tuple[bool, str]:
        stop_success, stop_message = self._stop_target(target_name)
        if not stop_success:
            return False, stop_message

        if target_name == "slam":
            start_success, start_message = self._start_slam_target()
        elif target_name == "navigation_slam":
            start_success, start_message = self._start_navigation_slam_target()
        elif target_name == "frontier_explorer":
            start_success, start_message = self._start_frontier_explorer_target()
        else:
            start_success, start_message = self._start_target(target_name)
        return start_success, f"{stop_message}\n{start_message}"

    def _start_slam_target(self) -> Tuple[bool, str]:
        messages = []
        success, message = self._start_target("slam")
        messages.append(message)
        if not success:
            return False, "\n".join(messages)

        threading.Thread(
            target=self._activate_slam_sequence,
            daemon=True,
        ).start()
        messages.append(
            "slam_toolbox Aktivierung laeuft im Hintergrund; "
            "Status mit 'ros2 lifecycle get /slam_toolbox' pruefen"
        )

        return True, "\n".join(messages)

    def _start_navigation_slam_target(self) -> Tuple[bool, str]:
        messages = []
        success, message = self._start_target("navigation_slam")
        messages.append(message)
        if not success:
            return False, "\n".join(messages)

        threading.Thread(
            target=self._ensure_navigation_slam_ready_sequence,
            daemon=True,
        ).start()
        messages.append(
            "navigation_slam Ready-Check laeuft im Hintergrund; "
            "Status ueber /navigate_to_pose oder Lifecycle-Nodes pruefen"
        )

        return True, "\n".join(messages)

    def _start_frontier_explorer_target(self) -> Tuple[bool, str]:
        managed = self._managed["frontier_explorer"]
        if managed.is_running():
            return True, "frontier_explorer laeuft bereits"

        threading.Thread(
            target=self._wait_for_nav_and_start_frontier,
            daemon=True,
        ).start()
        return (
            True,
            "frontier_explorer wartet im Hintergrund auf /navigate_to_pose"
        )

    def _wait_for_nav_and_start_frontier(self) -> None:
        success, message = self._wait_for_navigation_slam_ready(
            timeout_sec=180.0,
        )
        if not success:
            self.get_logger().warn(message)
            return

        self.get_logger().info(message)
        success, message = self._start_target("frontier_explorer")
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().warn(message)

    def _activate_slam_sequence(self) -> None:
        success, message = self._activate_lifecycle_node(
            "/slam_toolbox",
            timeout_sec=180.0,
        )
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)

    def _ensure_navigation_slam_ready_sequence(self) -> None:
        success, message = self._wait_for_navigation_slam_active(timeout_sec=45.0)
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().warn(message)
            self.get_logger().warn(
                "Versuche navigation_slam Lifecycle-Nodes manuell zu aktivieren"
            )
            for node_name in self.NAVIGATION_SLAM_LIFECYCLE_NODES:
                success, message = self._activate_lifecycle_node(
                    node_name,
                    timeout_sec=90.0,
                )
                if success:
                    self.get_logger().info(message)
                    continue

                self.get_logger().error(message)
                self.get_logger().error(
                    f"navigation_slam Aktivierung abgebrochen bei {node_name}"
                )
                return

        success, message = self._wait_for_action_server(
            "/navigate_to_pose",
            timeout_sec=90.0,
        )
        if success:
            self.get_logger().info(message)
            self._set_frontier_ready(True)
        else:
            self.get_logger().warn(message)

    def _start_mapping_with_prerequisites(self) -> Tuple[bool, str]:
        messages = []
        self._map_received_event.clear()
        self._set_frontier_ready(False)
        self._set_mapping_complete(False)

        for prerequisite in ("description", "odometry"):
            if prerequisite not in self._managed:
                continue
            success, message = self._start_target_with_imu_barrier(prerequisite)
            messages.append(message)
            if not success:
                self.get_logger().error(message)

        success, message = self._start_target("mapping")
        messages.append(message)
        if not success:
            return False, "\n".join(messages)

        threading.Thread(
            target=self._mapping_start_sequence,
            daemon=True,
        ).start()
        messages.append(
            "Startfolge laeuft im Hintergrund: slam_toolbox aktivieren, "
            "auf /map warten, navigation_slam starten, Frontier-Ready setzen"
        )

        return True, "\n".join(messages)

    def _mapping_start_sequence(self) -> None:
        success, message = self._activate_lifecycle_node(
            "/slam_toolbox",
            timeout_sec=180.0,
        )
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)
            return

        success, message = self._wait_for_map(timeout_sec=180.0)
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)
            return

        success, message = self._wait_for_tf(
            "map",
            "base_link",
            timeout_sec=120.0,
        )
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)
            return

        if "navigation_slam" not in self._managed:
            self.get_logger().error(
                "Target 'navigation_slam' ist nicht im Taskmanager konfiguriert"
            )
            return

        success, message = self._start_navigation_slam_target()
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)
            return

        success, message = self._wait_for_frontier_ready(timeout_sec=180.0)
        if success:
            self.get_logger().info(message)
            self._set_frontier_ready(True)
        else:
            self.get_logger().error(message)

    def _activate_lifecycle_node(
        self,
        node_name: str,
        timeout_sec: float,
    ) -> Tuple[bool, str]:
        deadline = time.monotonic() + timeout_sec
        last_state: Optional[str] = None

        while time.monotonic() < deadline:
            state = self._get_lifecycle_state(node_name)
            if state is None:
                time.sleep(1.0)
                continue

            if state != last_state:
                self.get_logger().info(f"{node_name} Lifecycle-State: {state}")
                last_state = state

            if state == "active":
                return True, f"{node_name} ist active"

            if state == "unconfigured":
                success, message = self._set_lifecycle_transition(node_name, "configure")
                if not success:
                    return False, message
                self.get_logger().info(message)
                time.sleep(2.0)
                continue

            if state == "inactive":
                success, message = self._set_lifecycle_transition(node_name, "activate")
                if not success:
                    return False, message
                self.get_logger().info(message)
                time.sleep(2.0)
                continue

            time.sleep(1.0)

        return False, f"Timeout beim Aktivieren von {node_name}; letzter State: {last_state}"

    def _wait_for_navigation_slam_active(self, timeout_sec: float) -> Tuple[bool, str]:
        deadline = time.monotonic() + timeout_sec
        last_states: Dict[str, Optional[str]] = {}

        while time.monotonic() < deadline:
            states = {
                node_name: self._get_lifecycle_state(node_name)
                for node_name in self.NAVIGATION_SLAM_LIFECYCLE_NODES
            }
            if all(state == "active" for state in states.values()):
                return True, "Alle navigation_slam Lifecycle-Nodes sind active"

            changed_states = {
                node_name: state
                for node_name, state in states.items()
                if last_states.get(node_name) != state
            }
            if changed_states:
                state_text = ", ".join(
                    f"{node_name}={state or 'missing'}"
                    for node_name, state in states.items()
                )
                self.get_logger().info(f"navigation_slam Lifecycle-States: {state_text}")
                last_states = states

            time.sleep(1.0)

        state_text = ", ".join(
            f"{node_name}={state or 'missing'}"
            for node_name, state in last_states.items()
        )
        return False, f"Timeout beim Warten auf navigation_slam active: {state_text}"

    def _wait_for_lifecycle_state(
        self,
        node_name: str,
        desired_states: set[str],
        timeout_sec: float,
    ) -> Tuple[bool, str]:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            state = self._get_lifecycle_state(node_name)
            if state in desired_states:
                desired = ", ".join(sorted(desired_states))
                return True, f"{node_name} Lifecycle-State ist {state} (erwartet: {desired})"
            time.sleep(1.0)

        desired = ", ".join(sorted(desired_states))
        return False, f"Timeout beim Warten auf {node_name} Lifecycle-State: {desired}"

    def _get_lifecycle_state(self, node_name: str) -> Optional[str]:
        client = self.create_client(GetState, f"{node_name}/get_state")
        if not client.wait_for_service(timeout_sec=1.0):
            return None

        future = client.call_async(GetState.Request())
        deadline = time.monotonic() + 20.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)

        if not future.done():
            return None

        response = future.result()
        if response is None:
            return None

        return str(response.current_state.label or "").lower()

    def _set_lifecycle_transition(
        self,
        node_name: str,
        transition: str,
    ) -> Tuple[bool, str]:
        transition_ids = {
            "configure": Transition.TRANSITION_CONFIGURE,
            "activate": Transition.TRANSITION_ACTIVATE,
        }
        transition_id = transition_ids.get(transition)
        if transition_id is None:
            return False, f"{node_name} lifecycle {transition}: unbekannte Transition"

        client = self.create_client(ChangeState, f"{node_name}/change_state")
        if not client.wait_for_service(timeout_sec=5.0):
            return False, f"{node_name} lifecycle {transition}: Service nicht erreichbar"

        request = ChangeState.Request()
        request.transition.id = transition_id
        request.transition.label = transition
        future = client.call_async(request)
        deadline = time.monotonic() + 60.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)

        if not future.done():
            return False, f"{node_name} lifecycle {transition}: Timeout"

        response = future.result()
        if response is None:
            return False, f"{node_name} lifecycle {transition}: keine Antwort"

        if not response.success:
            return False, f"{node_name} lifecycle {transition}: abgelehnt"

        return True, f"{node_name} lifecycle {transition}: successful"

    def _wait_for_tf(
        self,
        target_frame: str,
        source_frame: str,
        timeout_sec: float,
    ) -> Tuple[bool, str]:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._tf_buffer.can_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
            ):
                return True, f"TF {target_frame}->{source_frame} ist verfuegbar"

            time.sleep(0.25)

        return (
            False,
            f"Timeout beim Warten auf TF {target_frame}->{source_frame}; "
            "navigation_slam/frontier_explorer werden nicht gestartet",
        )

    def _lookup_current_robot_pose_in_map(self) -> Optional[TransformStamped]:
        try:
            return self._tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                timeout=Duration(seconds=2.0),
            )
        except Exception as exc:  # noqa: BLE001 - tf2 exception types vary by distro.
            self.get_logger().warn(f"map->base_link Lookup fehlgeschlagen: {exc}")
            return None

    def _publish_initial_pose_sequence(self, transform: TransformStamped) -> None:
        def publish_initial_pose() -> None:
            success, message = self._wait_for_lifecycle_state(
                "/amcl",
                {"active"},
                timeout_sec=60.0,
            )
            if success:
                self.get_logger().info(message)
            else:
                self.get_logger().warn(message)

            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.pose.position.x = transform.transform.translation.x
            pose_msg.pose.pose.position.y = transform.transform.translation.y
            pose_msg.pose.pose.position.z = 0.0
            pose_msg.pose.pose.orientation = transform.transform.rotation
            pose_msg.pose.covariance[0] = 0.05
            pose_msg.pose.covariance[7] = 0.05
            pose_msg.pose.covariance[35] = 0.10

            for _ in range(5):
                pose_msg.header.stamp = rclpy.time.Time().to_msg()
                self._initial_pose_pub.publish(pose_msg)
                time.sleep(0.5)

            self.get_logger().info(
                "AMCL Initialpose publiziert: "
                f"x={pose_msg.pose.pose.position.x:.3f}, "
                f"y={pose_msg.pose.pose.position.y:.3f}"
            )

        threading.Thread(target=publish_initial_pose, daemon=True).start()

    def _wait_for_action_server(
        self,
        action_name: str,
        timeout_sec: float,
    ) -> Tuple[bool, str]:
        if action_name == "/navigate_to_pose":
            if self._navigate_to_pose_client.wait_for_server(timeout_sec=timeout_sec):
                return True, f"Action Server {action_name} ist verfuegbar"

            success, message = self._wait_for_action_server_via_cli(
                action_name,
                timeout_sec=10.0,
            )
            if success:
                return True, message

            return (
                False,
                f"Timeout beim Warten auf Action Server {action_name}; "
                "frontier_explorer wird nicht gestartet",
            )

        return self._wait_for_action_server_via_cli(action_name, timeout_sec)

    def _wait_for_action_server_via_cli(
        self,
        action_name: str,
        timeout_sec: float,
    ) -> Tuple[bool, str]:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            result = subprocess.run(
                ["ros2", "action", "info", action_name],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=5.0,
                check=False,
            )
            output = result.stdout or ""
            if "Action servers: 1" in output:
                return True, f"Action Server {action_name} ist verfuegbar"

            time.sleep(1.0)

        return (
            False,
            f"Timeout beim Warten auf Action Server {action_name}; "
            "frontier_explorer wird nicht gestartet",
        )

    def _wait_for_map(self, timeout_sec: float) -> Tuple[bool, str]:
        if self._map_received_event.wait(timeout=timeout_sec):
            return True, "/map wurde publiziert"

        return False, "Timeout beim Warten auf /map; navigation_slam wird nicht gestartet"

    def _wait_for_frontier_ready(self, timeout_sec: float) -> Tuple[bool, str]:
        success, message = self._wait_for_navigation_slam_ready(timeout_sec=timeout_sec)
        if not success:
            return False, message

        return True, "Frontier Explorer ist startbereit"

    def _wait_for_navigation_slam_ready(self, timeout_sec: float) -> Tuple[bool, str]:
        deadline = time.monotonic() + timeout_sec

        success, message = self._wait_for_navigation_slam_active(
            timeout_sec=min(120.0, max(1.0, deadline - time.monotonic())),
        )
        if not success:
            return False, message

        remaining = max(1.0, deadline - time.monotonic())
        success, message = self._wait_for_action_server(
            "/navigate_to_pose",
            timeout_sec=remaining,
        )
        if not success:
            return False, message

        return True, "navigation_slam ist active und /navigate_to_pose ist verfuegbar"

    def _handle_map(self, msg: OccupancyGrid) -> None:
        del msg
        self._map_received_event.set()

    def _handle_imu_ready(self, msg: Imu) -> None:
        del msg
        self._imu_ready_event.set()

    def _set_frontier_ready(self, ready: bool) -> None:
        if self._frontier_ready == ready:
            return

        self._frontier_ready = ready
        self._publish_frontier_ready()
        self._publish_mapping_complete()

    def _publish_status(self) -> None:
        msg = String()
        msg.data = self._status_text()
        self._status_pub.publish(msg)
        self._publish_frontier_ready()

    def _publish_frontier_ready(self) -> None:
        msg = String()
        msg.data = "ready" if self._frontier_ready else "not_ready"
        self._frontier_ready_pub.publish(msg)

    def _set_mapping_complete(self, complete: bool) -> None:
        if self._mapping_complete == complete:
            return

        self._mapping_complete = complete
        self._publish_mapping_complete()

    def _publish_mapping_complete(self) -> None:
        msg = String()
        msg.data = "complete" if self._mapping_complete else "not_complete"
        self._mapping_complete_pub.publish(msg)

    def _status_text(self) -> str:
        lines = []
        for name, managed in self._managed.items():
            if managed.is_running():
                log_suffix = f" log={managed.log_path}" if managed.log_path else ""
                lines.append(f"{name}: running pid={managed.process.pid}{log_suffix}")
                continue

            return_code = managed.returncode()
            if return_code is None:
                lines.append(f"{name}: stopped")
            else:
                log_suffix = f" log={managed.log_path}" if managed.log_path else ""
                lines.append(f"{name}: exited returncode={return_code}{log_suffix}")
        return "\n".join(lines)

    def stop_all(self) -> None:
        for name in reversed(list(self._managed.keys())):
            self._stop_target(name)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
