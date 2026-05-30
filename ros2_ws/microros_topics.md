# BRT7K micro-ROS Topic / Message Übersicht

Die ESP32-Boards kommunizieren über micro-ROS mit dem ROS 2-Agent auf dem
Haupt-Rechner (Raspberry Pi / PC). Die Verbindung läuft jeweils über
**UART0 (USB-Serial)** mit dem `micro_ros_agent`.

```bash
micro_ros_agent serial --dev /dev/ttyUSBx --baud 115200
```

Die Raspberry-Pi-Serial-Zuordnung liest beim Start die Boot-Banner der Boards:

| Board | Banner | Symlink |
|-------|--------|---------|
| Drive / PSU | `BRT7K_ROLE=drive` | `/dev/esp_drive` |
| Gripper | `BRT7K_ROLE=gripper` | `/dev/esp_gripper` |

---

## 1. Drive / PSU PCB (`esp32_drive`)

**Framework:** Arduino Core + micro_ros_arduino
**Source:** `firmware/esp_wroom_32/BRT7K_ESP32_PSU/src/main.cpp`
**Sensoren:** INA226 Batterie-Monitor, SHARP Analog-TOF
**Aktoren:** DFRobot AGV-Motoren, 2x NeoPixel-Ring (32 LEDs je)

### Subscriptions (ROS 2 → ESP32)

| Topic | Message-Typ | Beschreibung |
|-------|-------------|--------------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Fahrbefehl für Differentialantrieb |
| `/gripper/left_ring_color` | `std_msgs/msg/Int32` | Farbe Ring 1 (links) als `0x00RRGGBB` |
| `/gripper/right_ring_color` | `std_msgs/msg/Int32` | Farbe Ring 2 (rechts) als `0x00RRGGBB` |

#### `/cmd_vel` (`geometry_msgs/msg/Twist`)

| Feld | Typ | Einheit | Bedeutung |
|------|-----|---------|-----------|
| `linear.x` | `float64` | m/s | Vorwärts-/Rückwärtsgeschwindigkeit |
| `angular.z` | `float64` | rad/s | Drehgeschwindigkeit |

Die Firmware berechnet daraus die linke und rechte Rad-Drehzahl:

```text
rpm_left  = (linear.x - angular.z * WHEEL_BASE_M / 2) * rpm_factor
rpm_right = (linear.x + angular.z * WHEEL_BASE_M / 2) * rpm_factor
```

Der rechte Motor wird in der Firmware invertiert, weil er mechanisch gespiegelt
eingebaut ist.

```bash
# Langsam vorwärts fahren
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"

# Auf der Stelle drehen
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

#### Farbkodierung (packed RGB)

```text
Bit 23..16 -> R (0-255)
Bit 15..8  -> G (0-255)
Bit  7..0  -> B (0-255)
```

```bash
# Ring 1 auf Rot setzen (R=255, G=0, B=0 -> 0xFF0000 = 16711680)
ros2 topic pub --once /gripper/left_ring_color \
  std_msgs/msg/Int32 "{data: 16711680}"

# Ring 2 auf Grün setzen (0x00FF00 = 65280)
ros2 topic pub --once /gripper/right_ring_color \
  std_msgs/msg/Int32 "{data: 65280}"

# Alles aus
ros2 topic pub --once /gripper/left_ring_color  std_msgs/msg/Int32 "{data: 0}"
ros2 topic pub --once /gripper/right_ring_color std_msgs/msg/Int32 "{data: 0}"
```

### Publications (ESP32 → ROS 2)

| Topic | Message-Typ | Rate | Beschreibung |
|-------|-------------|------|--------------|
| `/esp32_drive/heartbeat` | `std_msgs/msg/Empty` | 1 Hz | Lebenszeichen für `hardware_supervisor` |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | 1 Hz | Spannung, Strom und berechneter Akkustand |
| `/tof_distance_cm` | `std_msgs/msg/Float32` | 10 Hz | SHARP-TOF-Abstand in cm, `-1.0` bei out of range |

#### `sensor_msgs/msg/BatteryState` belegte Felder

| Feld | Typ | Einheit | Wert |
|------|-----|---------|------|
| `voltage` | `float32` | V | Busspannung (INA226, 1.25 mV/LSB) |
| `current` | `float32` | A | Strom (positiv = Entladung) |
| `charge` | `float32` | Ah | Restkapazität aus Integration |
| `capacity` | `float32` | Ah | `BATTERY_CAPACITY_AH` |
| `design_capacity` | `float32` | Ah | `BATTERY_CAPACITY_AH` |
| `percentage` | `float32` | 0-1 | Restanteil aus verbrauchter Energie |
| `power_supply_status` | `uint8` | - | `UNKNOWN` (0) |
| `power_supply_health` | `uint8` | - | `UNKNOWN` (0) |
| `power_supply_technology` | `uint8` | - | `UNKNOWN` (0) |
| `present` | `bool` | - | `true` |

Die Firmware integriert Strom und Leistung im Loop:

```text
consumed_wh += voltage * current * dt_h
consumed_ah += current * dt_h
```

Die Kapazitätswerte sind aktuell auf `36 Wh` und `2 Ah` gesetzt und müssen zur
tatsächlich verwendeten Batterie passen.

```bash
# Spannung und Strom live anzeigen
ros2 topic echo /battery_state

# Heartbeat prüfen
ros2 topic hz /esp32_drive/heartbeat

# TOF-Wert anzeigen
ros2 topic echo /tof_distance_cm
```

---

## 2. Gripper PCB (`esp32_gripper`)

**Framework:** Arduino Core + micro_ros_arduino
**Source:** `firmware/esp_wroom_32/BRT7K_ESP32_GRIPPER/src/main.cpp`

### Subscriptions (ROS 2 → ESP32)

| Topic | Message-Typ | Beschreibung |
|-------|-------------|--------------|
| `/esp32_gripper/command` | `std_msgs/msg/Float32` | Ziel-Spaltbreite zwischen den Greifbacken in Meter |

#### `std_msgs/msg/Float32` Feld

| Feld | Typ | Einheit | Bedeutung |
|------|-----|---------|-----------|
| `data` | `float32` | m | Ziel-Spaltbreite zwischen den Greifbacken |

```bash
# Beispiel: Greifer schließen (gap=0.01 m)
ros2 topic pub --once /esp32_gripper/command \
  std_msgs/msg/Float32 \
  "{data: 0.01}"

# Beispiel: Greifer öffnen (gap=0.08 m)
ros2 topic pub --once /esp32_gripper/command \
  std_msgs/msg/Float32 \
  "{data: 0.08}"
```

### Publications (ESP32 → ROS 2)

| Topic | Message-Typ | Rate | Beschreibung |
|-------|-------------|------|--------------|
| `/esp32_gripper/is_closed` | `std_msgs/msg/Bool` | on change | `true` = Objekt gegriffen und Plattform oben |
| `/esp32_gripper/heartbeat` | `std_msgs/msg/Empty` | 2 Hz | Lebenszeichen für `hardware_supervisor` |
| `/esp32_gripper/lift_weight` | `std_msgs/msg/Int32` | 10 Hz | NAU7802-Rohwert Hubplattform, nach Firmware-Tara |
| `/esp32_gripper/left_weight` | `std_msgs/msg/Int32` | 10 Hz | NAU7802-Rohwert linker Greifarm, nach Firmware-Tara |
| `/esp32_gripper/right_weight` | `std_msgs/msg/Int32` | 10 Hz | NAU7802-Rohwert rechter Greifarm, nach Firmware-Tara |

#### Gripper-Zustandsmaschine (zur Information)

```text
HOMING_ARMS -> HOMING_LIFT -> READY
  ↓ (command)
LOWERING -> OPENING  (gap ≈ max -> öffnen)
         -> CLOSING  (gap < max -> schließen) -> LIFTING -> HOLDING
```

---

## Integrationshinweise

- Das Web-GUI liest `/battery_state` und sendet Fahrbefehle auf `/cmd_vel`.
  Diese Topics passen zur aktuellen Drive/PSU-Firmware.
- `hardware_supervisor` überwacht `/esp32_drive/heartbeat` und
  `/esp32_gripper/heartbeat`.
- Der aktuelle `object_task_executor` im ROS-Workspace nutzt
  `/gripper/command` und `/platform/command` als `std_msgs/msg/String`
  (`open`, `close`, `up`, `down`). Diese Schnittstelle passt nicht direkt zur
  Arduino-Gripper-Firmware und braucht entweder eine Bridge oder eine Anpassung
  im Executor/Firmware-Protokoll.

---

## Schnellübersicht aller ESP32-Topics

| Topic | Richtung | Typ | Board |
|-------|----------|-----|-------|
| `/cmd_vel` | ROS→ESP | `geometry_msgs/Twist` | Drive / PSU |
| `/gripper/left_ring_color` | ROS→ESP | `std_msgs/Int32` | Drive / PSU |
| `/gripper/right_ring_color` | ROS→ESP | `std_msgs/Int32` | Drive / PSU |
| `/esp32_drive/heartbeat` | ESP→ROS | `std_msgs/Empty` | Drive / PSU |
| `/battery_state` | ESP→ROS | `sensor_msgs/BatteryState` | Drive / PSU |
| `/tof_distance_cm` | ESP→ROS | `std_msgs/Float32` | Drive / PSU |
| `/esp32_gripper/command` | ROS→ESP | `std_msgs/Float32` | Gripper |
| `/esp32_gripper/is_closed` | ESP→ROS | `std_msgs/Bool` | Gripper |
| `/esp32_gripper/heartbeat` | ESP→ROS | `std_msgs/Empty` | Gripper |
| `/esp32_gripper/lift_weight` | ESP→ROS | `std_msgs/Int32` | Gripper |
| `/esp32_gripper/left_weight` | ESP→ROS | `std_msgs/Int32` | Gripper |
| `/esp32_gripper/right_weight` | ESP→ROS | `std_msgs/Int32` | Gripper |

---

## micro-ROS Agent starten

```bash
# Beide ESP32-Boards jeweils auf eigenem Port anschließen, dann:
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp_drive --baud 115200 &
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp_gripper --baud 115200 &

# Alternativ mit ttyUSB-Ports:
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baud 115200 &
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1 --baud 115200 &
```

> Hinweis: Der Arduino-Gripper-Node pingt den Agent regelmäßig und erstellt
> seine micro-ROS-Entities nach Agent-Ausfall neu. Der aktuelle Drive/PSU-Node
> hat keinen solchen Reconnect-Loop; bei Verbindungsverlust muss das Board neu
> gestartet werden.
