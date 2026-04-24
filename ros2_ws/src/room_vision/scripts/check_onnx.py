"""
Sanity-Check für das exportierte YOLOv8-ONNX-Modell.

Prüft, ob cv2.dnn das Modell laden kann und ob die Output-Shape
zu einem YOLOv8-Detector mit 4 Klassen passt.

Ausführung:
    python check_onnx.py /pfad/zu/best.onnx
"""

import sys
import numpy as np
import cv2


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: python check_onnx.py /pfad/zu/best.onnx")
        sys.exit(1)

    onnx_path = sys.argv[1]

    print(f"Lade Modell: {onnx_path}")
    net = cv2.dnn.readNetFromONNX(onnx_path)

    dummy = np.zeros((640, 640, 3), dtype=np.uint8)
    blob = cv2.dnn.blobFromImage(dummy, 1 / 255.0, (640, 640), swapRB=True)

    net.setInput(blob)
    out = net.forward()

    print(f"Output-Shape: {out.shape}")

    # YOLOv8 Detect-Head-Output:
    #   shape = (1, 4 + num_classes, num_anchors)
    # Für 4 Klassen erwarten wir also shape = (1, 8, 8400)
    if out.ndim == 3 and out.shape[1] == 4 + 4:
        num_classes = out.shape[1] - 4
        print(f"OK — sieht aus wie YOLOv8 mit {num_classes} Klassen.")
        print(f"Anchors: {out.shape[2]}")
    else:
        print("WARNUNG — Output-Shape passt nicht zum erwarteten Format.")
        print("  Erwartet: (1, 8, N) für 4 Klassen.")
        print(f"  Bekommen: {out.shape}")


if __name__ == "__main__":
    main()
