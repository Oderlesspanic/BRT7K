# Trainierte KI ins ROS2-Paket einbinden

Dein `room_vision` ROS2-Paket ist jetzt auf **4 Klassen** konfiguriert,
passend zu deinem trainierten YOLOv8-Modell (`train3`):

| Index | Klasse       |
|-------|--------------|
| 0     | wurfel       |
| 1     | ball         |
| 2     | mate         |
| 3     | fhgr_logo    |

## Schritt 1 — ONNX-Export (auf dem Trainings-Rechner)

Ultralytics wird nur für den Export gebraucht:

```bash
pip install ultralytics onnx onnxsim
python export_onnx.py /pfad/zum/BildverarbeitungsKI
```

Das Skript erzeugt `runs/detect/train3/weights/best.onnx`.

## Schritt 2 — ONNX ins ROS2-Paket kopieren

```bash
cp /pfad/zu/best.onnx ~/ros2_ws/src/room_vision/models/best.onnx
```

## Schritt 3 — Sanity-Check

```bash
python check_onnx.py ~/ros2_ws/src/room_vision/models/best.onnx
```

Erwartete Ausgabe:
```
Output-Shape: (1, 8, 8400)
OK — sieht aus wie YOLOv8 mit 4 Klassen.
```

Wenn die Shape etwas wie `(1, 7, …)` oder `(1, 84, …)` ist, wurde das
Modell mit einer anderen Klassenzahl trainiert — dann ein neues
`best.pt` wählen oder neu trainieren.

## Schritt 4 — `detector_node.py` ins Paket kopieren

Die neue `detector_node.py` aus diesem Ordner ersetzt den bisherigen
Test-Node. Sie abonniert die Kamera, schickt das Bild durchs ONNX-Netz
und gibt die Ergebnisse als ROS2-Topics raus:

```bash
cp detector_node.py ~/ros2_ws/src/room_vision/room_vision/detector_node.py
```

In `package.xml` müssen folgende Abhängigkeiten stehen (falls noch nicht):

```xml
<depend>rclpy</depend>
<depend>sensor_msgs</depend>
<depend>vision_msgs</depend>
<depend>cv_bridge</depend>
<exec_depend>python3-opencv</exec_depend>
<exec_depend>python3-numpy</exec_depend>
```

In `setup.py` der Entry-Point (falls noch nicht vorhanden):

```python
entry_points={
    "console_scripts": [
        "detector = room_vision.detector_node:main",
    ],
},
```

## Schritt 5 — ROS2 bauen und starten

```bash
cd ~/ros2_ws
colcon build --packages-select room_vision
source install/setup.bash
ros2 run room_vision detector
```

## Topics

| Richtung   | Topic                          | Typ                           |
|------------|--------------------------------|-------------------------------|
| Subscribe  | `/camera/image_raw`            | `sensor_msgs/Image`           |
| Publish    | `/room_vision/image_annotated` | `sensor_msgs/Image`           |
| Publish    | `/room_vision/detections`      | `vision_msgs/Detection2DArray`|

Anschauen / debuggen:

```bash
# Boxen live sehen
ros2 run rqt_image_view rqt_image_view /room_vision/image_annotated

# Detections als Text
ros2 topic echo /room_vision/detections
```

## Parameter (optional überschreibbar)

```bash
ros2 run room_vision detector --ros-args \
    -p input_topic:=/camera/color/image_raw \
    -p confidence_threshold:=0.3 \
    -p nms_threshold:=0.45
```

Weitere Parameter: `model_path`, `image_output_topic`, `detections_topic`,
`input_size`, `class_names`.

## Hinweise

- **Pre-Processing**: Die Node macht Letterboxing (aspect-ratio erhalten,
  mit 114 padden) — das matcht, womit Ultralytics trainiert. Plain Resize
  würde die Accuracy leicht verschlechtern.
- **`real_sizes_m`** in `config/objects.yaml` sind Platzhalter (0.057 m,
  0.19 m, 0.168 m). Für reine 2D-Detections (wie jetzt) sind sie egal;
  sobald du 3D-Posen willst (z. B. via RealSense-Depth), kommen sie zum
  Einsatz. `fhgr_logo` bleibt `null` — ein 2D-Marker hat keine Tiefe.
- **Custom-Interface**: Falls du doch `room_interfaces/ObjectDetectionArray`
  statt `vision_msgs/Detection2DArray` brauchst, sag Bescheid — ich passe
  die Publisher-Zeile an, sobald ich die .msg-Definition sehe.
