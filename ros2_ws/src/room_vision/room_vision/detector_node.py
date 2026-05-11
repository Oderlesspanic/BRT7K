"""
ROS2-Node: Kamerabild -> YOLOv8 (ONNX) -> annotiertes Bild + Detections.

Topics:
    Subscribe:
        /camera/image_raw                (sensor_msgs/Image)
    Publish:
        /room_vision/image_annotated     (sensor_msgs/Image)
        /room_vision/detections          (interfaces/CameraDetectionArray)

Parameter (per ros2 param oder Launch-File ueberschreibbar):
    model_path          : absoluter Pfad zu best.onnx
                          Default: <room_vision>/models/best.onnx (ueber ament_index)
    input_topic         : /camera/image_raw
    image_output_topic  : /room_vision/image_annotated
    detections_topic    : /room_vision/detections
    input_size          : 640
    confidence_threshold: 0.25
    nms_threshold       : 0.45
    class_names         : ["wurfel", "ball", "mate", "fhgr_logo"]

Ausfuehren:
    ros2 run room_vision detector

Abhaengigkeiten (package.xml / rosdep):
    rclpy, sensor_msgs, vision_msgs, std_msgs, geometry_msgs, cv_bridge,
    python3-opencv, python3-numpy
"""

from __future__ import annotations

import os
from typing import List, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D
from interfaces.msg import CameraDetection, CameraDetectionArray

try:
    # optional — nur zum Auffinden des Default-Modellpfads im installierten Paket
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover
    get_package_share_directory = None  # type: ignore[assignment]


DEFAULT_CLASS_NAMES = ["wurfel", "ball", "mate", "fhgr_logo"]


class YoloDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("room_vision_detector")

        # ----- Parameter deklarieren -------------------------------------------------
        default_model_path = self._resolve_default_model_path()
        self.declare_parameter("model_path", default_model_path)
        self.declare_parameter("input_topic", "/camera/image_raw")
        self.declare_parameter("image_output_topic", "/room_vision/image_annotated")
        self.declare_parameter("detections_topic", "/room_vision/detections")
        self.declare_parameter("input_size", 640)
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("nms_threshold", 0.45)
        self.declare_parameter("class_names", DEFAULT_CLASS_NAMES)

        self.model_path: str = self.get_parameter("model_path").value
        self.input_size: int = int(self.get_parameter("input_size").value)
        self.conf_thres: float = float(self.get_parameter("confidence_threshold").value)
        self.nms_thres: float = float(self.get_parameter("nms_threshold").value)
        self.class_names: List[str] = list(self.get_parameter("class_names").value)

        input_topic: str = self.get_parameter("input_topic").value
        image_out_topic: str = self.get_parameter("image_output_topic").value
        det_out_topic: str = self.get_parameter("detections_topic").value

        # ----- Modell laden ----------------------------------------------------------
        if not os.path.isfile(self.model_path):
            raise FileNotFoundError(
                f"ONNX-Modell nicht gefunden: {self.model_path}\n"
                f"Tipp: Parameter 'model_path' setzen oder best.onnx nach "
                f"<room_vision>/models/best.onnx legen."
            )
        self.get_logger().info(f"Lade ONNX-Modell: {self.model_path}")
        self.net = cv2.dnn.readNetFromONNX(self.model_path)

        # Optional: CUDA aktivieren, wenn OpenCV mit CUDA kompiliert ist.
        # self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        # self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

        # Farben pro Klasse — deterministisch, damit Boxen konsistent aussehen.
        rng = np.random.default_rng(42)
        self.class_colors = (rng.integers(0, 255, size=(max(len(self.class_names), 1), 3))
                             .astype(int).tolist())

        # ----- ROS-Interfaces --------------------------------------------------------
        self.bridge = CvBridge()
        self.sub_image = self.create_subscription(
            Image, input_topic, self._on_image, 10
        )
        self.pub_image = self.create_publisher(Image, image_out_topic, 10)
        self.pub_detections = self.create_publisher(CameraDetectionArray, det_out_topic, 10)

        self.get_logger().info(
            f"Subscribed:  {input_topic}\n"
            f"Publishing:  {image_out_topic} (annotated)\n"
            f"             {det_out_topic} (Detection2DArray)\n"
            f"Klassen:     {self.class_names}"
        )

    # -------------------------------------------------------------------------------
    # Hilfen
    # -------------------------------------------------------------------------------
    @staticmethod
    def _resolve_default_model_path() -> str:
        """Default ist <install>/share/room_vision/models/best.onnx — falls installiert."""
        if get_package_share_directory is not None:
            try:
                share = get_package_share_directory("room_vision")
                return os.path.join(share, "models", "best.onnx")
            except Exception:
                pass
        # Fallback: relativer Pfad, falls direkt aus Source gestartet wird.
        return os.path.join(os.path.dirname(__file__), "..", "models", "best.onnx")

    def _letterbox(self, img: np.ndarray) -> Tuple[np.ndarray, float, int, int]:
        """
        YOLOv8-typische Vorverarbeitung: aspect-ratio erhalten, mit 114 padden.
        Liefert (padded_img, scale, pad_x, pad_y) — brauchen wir, um Boxen
        spaeter auf das Originalbild zurueckzurechnen.
        """
        h, w = img.shape[:2]
        s = self.input_size / max(h, w)
        new_w, new_h = int(round(w * s)), int(round(h * s))
        resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        pad_x = (self.input_size - new_w) // 2
        pad_y = (self.input_size - new_h) // 2
        padded = np.full((self.input_size, self.input_size, 3), 114, dtype=np.uint8)
        padded[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized
        return padded, s, pad_x, pad_y

    def _postprocess(
        self,
        raw_out: np.ndarray,
        scale: float,
        pad_x: int,
        pad_y: int,
        orig_shape: Tuple[int, int],
    ) -> List[Tuple[int, int, int, int, float, int]]:
        """
        YOLOv8-Output: shape (1, 4 + num_classes, N) mit N = 8400 Anchors.
        Ergebnis: Liste von (x1, y1, x2, y2, score, class_id) in Original-Bildkoords.
        """
        pred = raw_out[0]                       # (4 + C, N)
        pred = pred.transpose(1, 0)             # (N, 4 + C)

        boxes_xywh = pred[:, :4]                # center x, center y, w, h (im 640er Frame)
        class_scores = pred[:, 4:]              # (N, C)

        class_ids = np.argmax(class_scores, axis=1)
        scores = class_scores[np.arange(class_scores.shape[0]), class_ids]

        keep = scores >= self.conf_thres
        boxes_xywh = boxes_xywh[keep]
        scores = scores[keep]
        class_ids = class_ids[keep]

        if boxes_xywh.shape[0] == 0:
            return []

        # xywh (center) -> xyxy im 640er Raster
        cx, cy, w, h = boxes_xywh.T
        x1 = cx - w / 2.0
        y1 = cy - h / 2.0
        x2 = cx + w / 2.0
        y2 = cy + h / 2.0

        # Letterbox rueckrechnen -> Originalbild
        x1 = (x1 - pad_x) / scale
        y1 = (y1 - pad_y) / scale
        x2 = (x2 - pad_x) / scale
        y2 = (y2 - pad_y) / scale

        H, W = orig_shape
        x1 = np.clip(x1, 0, W - 1)
        y1 = np.clip(y1, 0, H - 1)
        x2 = np.clip(x2, 0, W - 1)
        y2 = np.clip(y2, 0, H - 1)

        # NMS pro Klasse via cv2.dnn.NMSBoxes (erwartet xywh im Zielraster)
        boxes_for_nms = np.stack([x1, y1, x2 - x1, y2 - y1], axis=1).tolist()
        scores_list = scores.tolist()

        idx = cv2.dnn.NMSBoxes(
            boxes_for_nms, scores_list, self.conf_thres, self.nms_thres
        )
        if len(idx) == 0:
            return []
        idx = np.array(idx).flatten()

        results: List[Tuple[int, int, int, int, float, int]] = []
        for i in idx:
            results.append((
                int(round(x1[i])), int(round(y1[i])),
                int(round(x2[i])), int(round(y2[i])),
                float(scores[i]),
                int(class_ids[i]),
            ))
        return results

    def _class_name(self, cls_id: int) -> str:
        if 0 <= cls_id < len(self.class_names):
            return self.class_names[cls_id]
        return f"class_{cls_id}"

    # -------------------------------------------------------------------------------
    # Hauptcallback
    # -------------------------------------------------------------------------------
    def _on_image(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001 — cv_bridge wirft diverse Subklassen
            self.get_logger().warning(f"cv_bridge konnte Bild nicht konvertieren: {exc}")
            return

        padded, scale, pad_x, pad_y = self._letterbox(frame)
        blob = cv2.dnn.blobFromImage(
            padded, 1.0 / 255.0, (self.input_size, self.input_size),
            swapRB=True, crop=False,
        )
        self.net.setInput(blob)
        raw_out = self.net.forward()

        detections = self._postprocess(
            raw_out, scale, pad_x, pad_y, orig_shape=frame.shape[:2]
        )

        # --- CameraDetectionArray zusammenbauen + Boxen zeichnen ----------------
        det_array = CameraDetectionArray()
        det_array.header = msg.header  # gleicher frame_id/stamp wie die Kamera

        annotated = frame.copy()
        for (x1, y1, x2, y2, score, cls_id) in detections:
            name = self._class_name(cls_id)
            color = self.class_colors[cls_id % len(self.class_colors)]
            color_bgr = (int(color[0]), int(color[1]), int(color[2]))

            cv2.rectangle(annotated, (x1, y1), (x2, y2), color_bgr, 2)
            label = f"{name} {score:.2f}"
            (tw, th), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
            )
            cv2.rectangle(
                annotated,
                (x1, max(0, y1 - th - baseline - 3)),
                (x1 + tw + 2, y1),
                color_bgr, thickness=-1,
            )
            cv2.putText(
                annotated, label, (x1 + 1, max(th, y1 - 3)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA,
            )

            det = CameraDetection()
            det.header = msg.header
            bbox = BoundingBox2D()
            bbox.center.position.x = float((x1 + x2) / 2.0)
            bbox.center.position.y = float((y1 + y2) / 2.0)
            bbox.center.theta = 0.0
            bbox.size_x = float(x2 - x1)
            bbox.size_y = float(y2 - y1)
            det.bbox = bbox
            det.class_name = name
            det.confidence = float(score)
            det_array.detections.append(det)

        self.pub_detections.publish(det_array)

        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out_msg.header = msg.header
        self.pub_image.publish(out_msg)


def main(args: list | None = None) -> None:
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
