import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from ultralytics import YOLO
import cv2


class YoloNode(Node):
    def __init__(self):
        super().__init__("yolo_node")

        self.bridge = CvBridge()
        self.model = YOLO("/home/ros/yolo/best.pt")  # dein eigenes Modell

        self.deadzone_px = 40
        self.max_turn_speed = 0.25
        self.min_turn_speed = 0.05

        # Distanz-Kalibrierung: muss später angepasst werden
        # distance_cm = distance_k / box_height_px
        self.distance_k = 8000.0

        self.image_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10
        )

        self.image_pub = self.create_publisher(
            Image,
            "/yolo/image_annotated",
            10
        )

        self.target_pub = self.create_publisher(
            String,
            "/yolo/target",
            10
        )

        self.get_logger().info("YOLO Node gestartet")

    def calculate_turn_speed(self, dx, image_center_x):
        if abs(dx) < self.deadzone_px:
            return 0.0

        normalized_error = dx / image_center_x
        turn_speed = normalized_error * self.max_turn_speed

        # Begrenzen
        turn_speed = max(min(turn_speed, self.max_turn_speed), -self.max_turn_speed)

        # minimale Drehgeschwindigkeit, damit Motor überhaupt reagiert
        if abs(turn_speed) < self.min_turn_speed:
            turn_speed = self.min_turn_speed if turn_speed > 0 else -self.min_turn_speed

        return turn_speed

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self.model(frame, verbose=False)

        annotated = results[0].plot()

        h, w = annotated.shape[:2]
        image_center_x = w // 2
        image_center_y = h // 2

        # Bildmitte einzeichnen
        cv2.circle(annotated, (image_center_x, image_center_y), 6, (255, 255, 255), -1)
        cv2.line(annotated, (image_center_x - 20, image_center_y),
                 (image_center_x + 20, image_center_y), (255, 255, 255), 2)
        cv2.line(annotated, (image_center_x, image_center_y - 20),
                 (image_center_x, image_center_y + 20), (255, 255, 255), 2)

        best_target = None

        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            class_name = self.model.names[cls_id]

            obj_center_x = (x1 + x2) // 2
            obj_center_y = (y1 + y2) // 2

            dx = obj_center_x - image_center_x
            dy = obj_center_y - image_center_y

            box_height = max(y2 - y1, 1)
            distance_cm = self.distance_k / box_height

            turn_speed = self.calculate_turn_speed(dx, image_center_x)

            if turn_speed > 0:
                direction = "right"
            elif turn_speed < 0:
                direction = "left"
            else:
                direction = "center"

            # Nimm aktuell das Objekt mit höchster Confidence als Ziel
            if best_target is None or conf > best_target["conf"]:
                best_target = {
                    "class": class_name,
                    "conf": conf,
                    "dx": dx,
                    "dy": dy,
                    "direction": direction,
                    "turn_speed": turn_speed,
                    "distance_cm": distance_cm,
                    "center": (obj_center_x, obj_center_y),
                    "box": (x1, y1, x2, y2),
                }

            # Objektzentrum und Linie einzeichnen
            cv2.circle(annotated, (obj_center_x, obj_center_y), 5, (0, 255, 255), -1)
            cv2.line(annotated, (image_center_x, image_center_y),
                     (obj_center_x, obj_center_y), (0, 255, 255), 2)

            text = f"{class_name} dx={dx}px turn={turn_speed:.2f} dist={distance_cm:.0f}cm"
            cv2.putText(
                annotated,
                text,
                (x1, max(y1 - 10, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                2
            )

        if best_target is not None:
            target_text = (
                f"class={best_target['class']};"
                f"confidence={best_target['conf']:.2f};"
                f"dx={best_target['dx']};"
                f"dy={best_target['dy']};"
                f"direction={best_target['direction']};"
                f"turn_speed={best_target['turn_speed']:.3f};"
                f"distance_cm={best_target['distance_cm']:.1f}"
            )
            self.target_pub.publish(String(data=target_text))
            self.get_logger().info(target_text)

        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out_msg.header = msg.header
        self.image_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
