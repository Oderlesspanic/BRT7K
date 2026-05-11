"""
ROS 2 Node: room_vision_target_selector

Berechnet fuer ALLE Detection2DArray-Detektionen die Bild-Geometrie
(dx, dy, direction, distance) und publiziert sie als Multi-Line-String.
Zusaetzlich wird ein annotiertes Bild mit Fadenkreuz und Linien zu allen
erkannten Objekten erzeugt.

Verantwortung: Geometrie-Berechnung + Visualisierung (KEINE Steuerung).

Topics:
    Subscribe:
        /room_vision/detections    (vision_msgs/Detection2DArray)
        /camera/camera_info        (sensor_msgs/CameraInfo)  - optional
        /camera/image_raw          (sensor_msgs/Image)        - fuer Visualisierung
    Publish:
        /room_vision/target            (std_msgs/String)
                                       - eine Zeile pro erkanntem Objekt
        /room_vision/target_annotated  (sensor_msgs/Image)

Format der String-Nachricht (eine Zeile pro Objekt):
    class=<name>;confidence=<0..1>;dx=<int_px>;dy=<int_px>;
    direction=<left|right|center>;distance_cm=<float>
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from interfaces.msg import CameraDetection, CameraDetectionArray


@dataclass
class TargetInfo:
    """Aufbereitete Geometrie-Info fuer ein erkanntes Objekt."""
    cls: str
    conf: float
    dx: int
    dy: int
    direction: str
    distance_cm: float
    obj_x: int        # Pixel-Mittelpunkt
    obj_y: int
    box_x1: int       # Bounding-Box-Ecken
    box_y1: int
    box_x2: int
    box_y2: int


class TargetSelectorNode(Node):
    def __init__(self) -> None:
        super().__init__("room_vision_target_selector")

        self.declare_parameter("detections_topic", "/room_vision/detections")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("target_topic", "/room_vision/target")
        self.declare_parameter("target_image_topic", "/room_vision/target_annotated")
        self.declare_parameter("filter_classes", "")
        self.declare_parameter("deadzone_px", 40)
        self.declare_parameter("distance_k", 8000.0)
        self.declare_parameter("publish_no_target", False)
        self.declare_parameter("publish_annotated_image", True)

        self.detections_topic: str = self.get_parameter("detections_topic").value
        self.camera_info_topic: str = self.get_parameter("camera_info_topic").value
        self.image_topic: str = self.get_parameter("image_topic").value
        self.target_topic: str = self.get_parameter("target_topic").value
        self.target_image_topic: str = self.get_parameter("target_image_topic").value
        filter_str: str = self.get_parameter("filter_classes").value or ""
        self.filter_classes: List[str] = [c.strip() for c in filter_str.split(",") if c.strip()]
        self.deadzone_px: int = int(self.get_parameter("deadzone_px").value)
        self.distance_k: float = float(self.get_parameter("distance_k").value)
        self.publish_no_target: bool = bool(self.get_parameter("publish_no_target").value)
        self.publish_annotated_image: bool = bool(self.get_parameter("publish_annotated_image").value)

        # State
        self.image_width: Optional[int] = None
        self.image_height: Optional[int] = None
        self._size_source_logged = False
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_frame_header = None
        self.bridge = CvBridge()

        # Subscriptions
        self.sub_det = self.create_subscription(
            CameraDetectionArray, self.detections_topic, self._on_detections, 10
        )
        self.sub_info = self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_camera_info, 10
        )
        self.sub_image = self.create_subscription(
            Image, self.image_topic, self._on_image, 10
        )

        # Publishers
        self.pub_target = self.create_publisher(String, self.target_topic, 10)
        self.pub_target_image = self.create_publisher(
            Image, self.target_image_topic, 10
        )

        self.get_logger().info(
            "Target Selector gestartet\n"
            f"  Subscribe: {self.detections_topic}\n"
            f"             {self.camera_info_topic} (optional)\n"
            f"             {self.image_topic}\n"
            f"  Publish:   {self.target_topic} (alle Objekte, eine Zeile pro Objekt)\n"
            f"             {self.target_image_topic} (annotiert)\n"
            f"  Filter:    {self.filter_classes if self.filter_classes else 'alle Klassen'}\n"
            f"  Deadzone:  {self.deadzone_px} px\n"
            f"  K:         {self.distance_k}"
        )

    # ---------------------------------------------------------------------
    def _set_size(self, width: int, height: int, source: str) -> None:
        if width <= 0 or height <= 0:
            return
        first_time = self.image_width is None
        self.image_width = int(width)
        self.image_height = int(height)
        if first_time or not self._size_source_logged:
            self.get_logger().info(
                f"Bildmasse gesetzt: {self.image_width}x{self.image_height} (Quelle: {source})"
            )
            self._size_source_logged = True

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._set_size(msg.width, msg.height, "camera_info")

    def _on_image(self, msg: Image) -> None:
        if self.image_width is None:
            self._set_size(msg.width, msg.height, "image_raw")
        if self.publish_annotated_image:
            try:
                self._latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                self._latest_frame_header = msg.header
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"cv_bridge konnte Bild nicht konvertieren: {exc}")

    # ---------------------------------------------------------------------
    def _direction_from_dx(self, dx: float) -> str:
        if dx < -self.deadzone_px:
            return "left"
        if dx > self.deadzone_px:
            return "right"
        return "center"

    def _build_target_info(self, det: CameraDetection) -> Optional[TargetInfo]:
        cls = det.class_name

        if self.filter_classes and cls not in self.filter_classes:
            return None

        conf = float(det.confidence)

        cx = self.image_width / 2.0
        cy = self.image_height / 2.0

        ox = det.bbox.center.position.x
        oy = det.bbox.center.position.y
        bw = max(det.bbox.size_x, 1.0)
        bh = max(det.bbox.size_y, 1.0)

        dx = int(round(ox - cx))
        dy = int(round(oy - cy))

        distance_cm = self.distance_k / bh
        direction = self._direction_from_dx(dx)

        x1 = int(round(ox - bw / 2.0))
        y1 = int(round(oy - bh / 2.0))
        x2 = int(round(ox + bw / 2.0))
        y2 = int(round(oy + bh / 2.0))

        return TargetInfo(
            cls=cls,
            conf=conf,
            dx=dx,
            dy=dy,
            direction=direction,
            distance_cm=distance_cm,
            obj_x=int(round(ox)),
            obj_y=int(round(oy)),
            box_x1=x1,
            box_y1=y1,
            box_x2=x2,
            box_y2=y2,
        )
    # ---------------------------------------------------------------------
    def _draw_crosshair(self, img: np.ndarray, cx: int, cy: int) -> None:
        cv2.circle(img, (cx, cy), 6, (255, 255, 255), -1)
        cv2.circle(img, (cx, cy), 7, (0, 0, 0), 1)
        cv2.line(img, (cx - 25, cy), (cx + 25, cy), (255, 255, 255), 2)
        cv2.line(img, (cx, cy - 25), (cx, cy + 25), (255, 255, 255), 2)

    def _draw_target(
        self, img: np.ndarray, cx_img: int, cy_img: int, t: TargetInfo
    ) -> None:
        """Zeichnet einen einzelnen Target: Box, Linie, Marker, Label."""
        color = (0, 255, 255)  # gelb

        # Box
        cv2.rectangle(img, (t.box_x1, t.box_y1), (t.box_x2, t.box_y2), color, 2)
        # Linie zur Bildmitte
        cv2.line(img, (cx_img, cy_img), (t.obj_x, t.obj_y), color, 2)
        # Marker
        cv2.circle(img, (t.obj_x, t.obj_y), 5, color, -1)
        cv2.circle(img, (t.obj_x, t.obj_y), 6, (0, 0, 0), 1)

        # Label
        label_lines = [
            f"{t.cls} ({t.conf:.2f})",
            f"dx={t.dx} dy={t.dy} dist={t.distance_cm:.0f}cm",
        ]
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.5
        thickness = 1
        line_height = 17

        # Hintergrund fuer Text
        max_tw = max(
            cv2.getTextSize(line, font, scale, thickness)[0][0] for line in label_lines
        )
        text_x = max(t.box_x1, 5)
        text_y_top = max(t.box_y1 - line_height * len(label_lines) - 4, 0)
        cv2.rectangle(
            img,
            (text_x - 3, text_y_top - 3),
            (text_x + max_tw + 5, text_y_top + line_height * len(label_lines) + 2),
            (0, 0, 0), thickness=-1,
        )
        for i, line in enumerate(label_lines):
            y = text_y_top + (i + 1) * line_height - 4
            cv2.putText(img, line, (text_x, y), font, scale,
                        color, thickness, cv2.LINE_AA)

    def _draw_no_target(self, img: np.ndarray) -> None:
        cv2.putText(img, "keine Ziele", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

    # ---------------------------------------------------------------------
    def _format_target_string(self, t: TargetInfo) -> str:
        return (
            f"class={t.cls};"
            f"confidence={t.conf:.2f};"
            f"dx={t.dx};"
            f"dy={t.dy};"
            f"direction={t.direction};"
            f"distance_cm={t.distance_cm:.1f}"
        )

    # ---------------------------------------------------------------------
    def _on_detections(self, msg: CameraDetectionArray) -> None:
        if self.image_width is None or self.image_height is None:
            return

        # Geometrie fuer alle Detections berechnen
        targets: List[TargetInfo] = []
        for det in msg.detections:
            ti = self._build_target_info(det)
            if ti is not None:
                targets.append(ti)

        # String publishen (multi-line, eine Zeile pro Objekt)
        if targets:
            lines = [self._format_target_string(t) for t in targets]
            self.pub_target.publish(String(data="\n".join(lines)))
        else:
            if self.publish_no_target:
                self.pub_target.publish(String(data="no_target"))

        # Annotiertes Bild
        if not self.publish_annotated_image or self._latest_frame is None:
            return

        annotated = self._latest_frame.copy()
        h, w = annotated.shape[:2]
        cx_img, cy_img = w // 2, h // 2

        self._draw_crosshair(annotated, cx_img, cy_img)

        if targets:
            for t in targets:
                self._draw_target(annotated, cx_img, cy_img, t)
        else:
            self._draw_no_target(annotated)

        try:
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            out_msg.header = self._latest_frame_header
            self.pub_target_image.publish(out_msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"Konnte annotiertes Bild nicht publishen: {exc}")


def main(args: list | None = None) -> None:
    rclpy.init(args=args)
    node = TargetSelectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
