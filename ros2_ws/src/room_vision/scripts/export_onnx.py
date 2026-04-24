"""
Exportiert das trainierte YOLOv8-Modell (train3) nach ONNX.

Voraussetzung:
    pip install ultralytics onnx onnxsim

Ausführung (auf dem Rechner, auf dem die KI trainiert wurde):
    python export_onnx.py /pfad/zum/BildverarbeitungsKI

Erzeugt:
    <BildverarbeitungsKI>/runs/detect/train3/weights/best.onnx

Danach auf dem ROS2-Rechner:
    cp best.onnx ~/ros2_ws/src/room_vision/models/best.onnx
"""

import sys
from pathlib import Path

try:
    from ultralytics import YOLO
except ImportError:
    print("FEHLER: 'ultralytics' ist nicht installiert.")
    print("Installiere mit:  pip install ultralytics onnx onnxsim")
    sys.exit(1)


def main() -> None:
    if len(sys.argv) >= 2:
        root = Path(sys.argv[1]).expanduser().resolve()
    else:
        # Fallback: Skript liegt im BildverarbeitungsKI-Ordner
        root = Path(__file__).resolve().parent

    pt_path = root / "runs" / "detect" / "train3" / "weights" / "best.pt"

    if not pt_path.exists():
        print(f"FEHLER: Kein best.pt gefunden unter: {pt_path}")
        print("Gib den Pfad zum BildverarbeitungsKI-Ordner als Argument an:")
        print("    python export_onnx.py /pfad/zum/BildverarbeitungsKI")
        sys.exit(1)

    print(f"Lade Modell: {pt_path}")
    model = YOLO(str(pt_path))

    # Export-Parameter so gewählt, dass das ONNX zu detector_node.py passt:
    #   imgsz=640       -> Node baut Blob mit (640, 640)
    #   opset=12        -> sicher kompatibel zu cv2.dnn.readNetFromONNX
    #   simplify=True   -> kleineres, schnelleres Modell
    #   dynamic=False   -> feste Input-Shape, cv2.dnn mag feste Shapes lieber
    onnx_path = model.export(
        format="onnx",
        imgsz=640,
        opset=12,
        simplify=True,
        dynamic=False,
    )

    print("\nFertig!")
    print(f"ONNX liegt unter: {onnx_path}")
    print("\nNächste Schritte:")
    print(f"  1. Datei auf den ROS2-Rechner übertragen")
    print(f"  2. cp best.onnx ~/ros2_ws/src/room_vision/models/best.onnx")
    print(f"  3. cd ~/ros2_ws && colcon build --packages-select room_vision")
    print(f"  4. source install/setup.bash")
    print(f"  5. ros2 run room_vision detector")


if __name__ == "__main__":
    main()
