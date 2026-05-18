"""
Farberkennung Demo — Mac Kamera
Exakte Portierung der edge_color ROS-Node Logik (color_patch_detector.cpp)

Voraussetzungen:
    pip install opencv-python numpy

Starten:
    python demo/farberkennung.py
"""

import cv2
import numpy as np


# ── Parameter (entsprechen edge_color.yaml) ─────────────────────────────────
MIN_CONTOUR_AREA = 800
SAT_MIN          = 80
VAL_MIN          = 60

WINDOW_MAIN = "Farberkennung"
WINDOW_MASK = "HSV-Maske"


def detect_color_patches(bgr: np.ndarray, sat_min: int, val_min: int, min_area: float):
    """
    Exakt die Logik aus color_patch_detector.cpp:
    BGR -> HSV -> inRange -> Morph(E,D,D,E) -> Konturen -> RotatedRect
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, (0, sat_min, val_min), (180, 255, 255))

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.erode(mask, kernel)
    mask = cv2.dilate(mask, kernel)
    mask = cv2.dilate(mask, kernel)
    mask = cv2.erode(mask, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    patches = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue

        rect      = cv2.minAreaRect(c)
        box       = cv2.boxPoints(rect).astype(int)
        center    = (int(rect[0][0]), int(rect[0][1]))

        # HSV-Statistik des Patches
        patch_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        cv2.drawContours(patch_mask, [c], 0, 255, -1)
        h_vals = hsv[:, :, 0][patch_mask == 255].astype(float)
        s_vals = hsv[:, :, 1][patch_mask == 255].astype(float)
        v_vals = hsv[:, :, 2][patch_mask == 255].astype(float)

        stats = {
            "mean_h": float(np.mean(h_vals)),
            "mean_s": float(np.mean(s_vals)),
            "mean_v": float(np.mean(v_vals)),
            "area":   area,
        }
        patches.append({"box": box, "center": center, "stats": stats})

    return patches, mask


def draw_patches(frame: np.ndarray, patches: list) -> np.ndarray:
    out = frame.copy()

    for p in patches:
        box    = p["box"]
        center = p["center"]
        stats  = p["stats"]

        # Grüner Rahmen + gelbe Ecken
        cv2.polylines(out, [box], True, (0, 255, 0), 2)
        for pt in box:
            cv2.circle(out, tuple(pt), 5, (0, 255, 255), -1)

        # Magenta Mittelpunkt
        cv2.circle(out, center, 7, (255, 0, 255), -1)

        # Label mit HSV + Fläche
        label = (f"H={stats['mean_h']:.0f}  "
                 f"S={stats['mean_s']:.0f}  "
                 f"V={stats['mean_v']:.0f}  "
                 f"A={stats['area']:.0f}px")

        lx, ly = box[1][0], box[1][1] - 8
        ly = max(ly, 14)

        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(out, (lx, ly - th - 3), (lx + tw + 4, ly + 3), (0, 0, 0), -1)
        cv2.putText(out, label, (lx + 2, ly),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    # Info-Overlay oben links
    info = f"Patches: {len(patches)}   SAT>={SAT_MIN}  VAL>={VAL_MIN}  AREA>={MIN_CONTOUR_AREA}"
    cv2.rectangle(out, (0, 0), (len(info) * 8 + 10, 22), (0, 0, 0), -1)
    cv2.putText(out, info, (5, 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

    return out


def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Kamera konnte nicht geöffnet werden.")
        return

    cv2.namedWindow(WINDOW_MAIN, cv2.WINDOW_NORMAL)
    cv2.namedWindow(WINDOW_MASK, cv2.WINDOW_NORMAL)

    # Trackbars zum Live-Anpassen der Schwellenwerte
    cv2.createTrackbar("SAT min",  WINDOW_MAIN, SAT_MIN,          255, lambda x: None)
    cv2.createTrackbar("VAL min",  WINDOW_MAIN, VAL_MIN,          255, lambda x: None)
    cv2.createTrackbar("Area min", WINDOW_MAIN, MIN_CONTOUR_AREA, 5000, lambda x: None)

    print("Farberkennung gestartet — [Q] oder [ESC] zum Beenden")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        sat_min  = cv2.getTrackbarPos("SAT min",  WINDOW_MAIN)
        val_min  = cv2.getTrackbarPos("VAL min",  WINDOW_MAIN)
        min_area = max(1, cv2.getTrackbarPos("Area min", WINDOW_MAIN))

        patches, mask = detect_color_patches(frame, sat_min, val_min, min_area)

        annotated   = draw_patches(frame, patches)
        mask_color  = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        cv2.imshow(WINDOW_MAIN, annotated)
        cv2.imshow(WINDOW_MASK, mask_color)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
