"""
Objekterkennung Demo — Mac Kamera
Portierung der room_vision ROS-Node (YOLOv8 ONNX)

Voraussetzungen:
    pip install opencv-python numpy

Modell:
    ros2_ws/src/room_vision/models/best.onnx
    (wird automatisch gefunden wenn das Script aus dem BRT7K-Ordner gestartet wird)

Starten:
    python demo/objekterkennung.py
    python demo/objekterkennung.py --model /pfad/zu/best.onnx
"""

import argparse
import os
import time

import cv2
import numpy as np


# ── Parameter (entsprechen detector.yaml) ───────────────────────────────────
CLASS_NAMES          = ["Würfel", "Ball", "Mate", "FHGR Logo"]
INPUT_SIZE           = 640
CONFIDENCE_THRESHOLD = 0.25
NMS_THRESHOLD        = 0.45

DEFAULT_MODEL = os.path.join(
    os.path.dirname(__file__), "..", "ros2_ws", "src", "room_vision", "models", "best.onnx"
)

WINDOW = "Objekterkennung"


def letterbox(img: np.ndarray, size: int = 640):
    """Aspect-ratio erhalten, mit 114 auffüllen — exakt wie im ROS-Node."""
    h, w = img.shape[:2]
    s = size / max(h, w)
    new_w, new_h = int(round(w * s)), int(round(h * s))
    resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    pad_x = (size - new_w) // 2
    pad_y = (size - new_h) // 2
    padded = np.full((size, size, 3), 114, dtype=np.uint8)
    padded[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized
    return padded, s, pad_x, pad_y


def postprocess(raw_out, scale, pad_x, pad_y, orig_shape, conf_thres, nms_thres):
    """YOLOv8 Output (1, 4+C, 8400) -> Liste von (x1,y1,x2,y2, score, class_id)."""
    pred = raw_out[0].transpose(1, 0)       # (8400, 4+C)

    boxes_xywh  = pred[:, :4]
    class_scores = pred[:, 4:]

    class_ids = np.argmax(class_scores, axis=1)
    scores    = class_scores[np.arange(len(class_ids)), class_ids]

    keep = scores >= conf_thres
    if not keep.any():
        return []

    boxes_xywh = boxes_xywh[keep]
    scores     = scores[keep]
    class_ids  = class_ids[keep]

    cx, cy, w, h = boxes_xywh.T
    x1 = (cx - w / 2 - pad_x) / scale
    y1 = (cy - h / 2 - pad_y) / scale
    x2 = (cx + w / 2 - pad_x) / scale
    y2 = (cy + h / 2 - pad_y) / scale

    H, W = orig_shape
    x1 = np.clip(x1, 0, W - 1)
    y1 = np.clip(y1, 0, H - 1)
    x2 = np.clip(x2, 0, W - 1)
    y2 = np.clip(y2, 0, H - 1)

    boxes_nms = np.stack([x1, y1, x2 - x1, y2 - y1], axis=1).tolist()
    idx = cv2.dnn.NMSBoxes(boxes_nms, scores.tolist(), conf_thres, nms_thres)
    if len(idx) == 0:
        return []

    idx = np.array(idx).flatten()
    return [(int(round(x1[i])), int(round(y1[i])),
             int(round(x2[i])), int(round(y2[i])),
             float(scores[i]), int(class_ids[i])) for i in idx]


def draw_detections(frame: np.ndarray, detections: list,
                    class_names: list, colors: list, fps: float) -> np.ndarray:
    out = frame.copy()
    h, w = out.shape[:2]
    cx_img, cy_img = w // 2, h // 2

    for (x1, y1, x2, y2, score, cls_id) in detections:
        name  = class_names[cls_id] if cls_id < len(class_names) else f"class_{cls_id}"
        color = tuple(int(c) for c in colors[cls_id % len(colors)])
        obj_cx, obj_cy = (x1 + x2) // 2, (y1 + y2) // 2

        # Linie von Bildmitte zum Objekt
        cv2.line(out, (cx_img, cy_img), (obj_cx, obj_cy), (0, 255, 255), 1)

        # Bounding Box
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)

        # Label mit Hintergrund
        label = f"{name}  {score:.0%}"
        (tw, th), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        ly = max(y1 - 4, th + 4)
        cv2.rectangle(out, (x1, ly - th - 3), (x1 + tw + 6, ly + baseline), color, -1)
        cv2.putText(out, label, (x1 + 3, ly),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

    # Fadenkreuz Bildmitte
    cv2.circle(out, (cx_img, cy_img), 6, (255, 255, 255), -1)
    cv2.circle(out, (cx_img, cy_img), 7, (0, 0, 0), 1)
    cv2.line(out, (cx_img - 20, cy_img), (cx_img + 20, cy_img), (255, 255, 255), 2)
    cv2.line(out, (cx_img, cy_img - 20), (cx_img, cy_img + 20), (255, 255, 255), 2)

    # Info-Overlay
    info = f"FPS: {fps:.1f}   Objekte: {len(detections)}   Conf >= {CONFIDENCE_THRESHOLD:.0%}"
    cv2.rectangle(out, (0, 0), (len(info) * 8 + 10, 22), (0, 0, 0), -1)
    cv2.putText(out, info, (5, 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

    return out


def main():
    parser = argparse.ArgumentParser(description="Objekterkennung Demo")
    parser.add_argument("--model", default=DEFAULT_MODEL,
                        help="Pfad zu best.onnx")
    parser.add_argument("--conf",  type=float, default=CONFIDENCE_THRESHOLD)
    parser.add_argument("--nms",   type=float, default=NMS_THRESHOLD)
    args = parser.parse_args()

    model_path = os.path.abspath(args.model)
    if not os.path.isfile(model_path):
        print(f"Modell nicht gefunden: {model_path}")
        print("Tipp: --model /pfad/zu/best.onnx")
        return

    print(f"Lade Modell: {model_path}")
    net = cv2.dnn.readNetFromONNX(model_path)
    print("Modell geladen.")

    rng    = np.random.default_rng(42)
    colors = rng.integers(50, 255, size=(max(len(CLASS_NAMES), 1), 3)).tolist()

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Kamera konnte nicht geöffnet werden.")
        return

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)

    print("Objekterkennung gestartet — [Q] oder [ESC] zum Beenden")

    fps     = 0.0
    t_prev  = time.time()
    frame_n = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        padded, scale, pad_x, pad_y = letterbox(frame, INPUT_SIZE)
        blob = cv2.dnn.blobFromImage(
            padded, 1.0 / 255.0, (INPUT_SIZE, INPUT_SIZE), swapRB=True, crop=False
        )
        net.setInput(blob)
        raw_out = net.forward()

        detections = postprocess(
            raw_out, scale, pad_x, pad_y,
            frame.shape[:2], args.conf, args.nms
        )

        frame_n += 1
        if frame_n % 10 == 0:
            fps    = 10.0 / (time.time() - t_prev)
            t_prev = time.time()

        annotated = draw_detections(frame, detections, CLASS_NAMES, colors, fps)
        cv2.imshow(WINDOW, annotated)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
