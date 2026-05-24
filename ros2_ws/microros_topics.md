# BRT7K — micro-ROS Topic / Message Übersicht

Alle drei ESP32-Boards kommunizieren über micro-ROS mit dem ROS 2-Agent
auf dem Haupt-Rechner (Raspberry Pi / PC). Die Verbindung läuft jeweils
über **UART0 (USB-Serial)** mit dem `micro_ros_agent`.

```
micro_ros_agent serial --dev /dev/ttyUSBx --baud 115200
```

---

## 1. Gripper PCB  (`esp32_gripper`)

**Framework:** ESP-IDF + micro-ROS Component  
**Source:** `firmware/esp_wroom_32/gripper/`

### Subscriptions (ROS 2 → ESP32)

| Topic | Message-Typ | Beschreibung |
|-------|-------------|--------------|
| `/esp32_gripper/command` | `control_msgs/msg/GripperCommand` | Greifbefehl: Spaltbreite + max. Kraft |

#### `control_msgs/msg/GripperCommand` Felder

| Feld | Typ | Einheit | Bedeutung |
|------|-----|---------|-----------|
| `position` | `float64` | m | Ziel-Spaltbreite zwischen den Greifbacken |
| `max_effort` | `float64` | N | Max. Greifkraft; `0.0` = nur positions-geregelt |

```bash
# Beispiel: Greifer schließen (gap=0.01 m, max 5 N)
ros2 topic pub --once /esp32_gripper/command \
  control_msgs/msg/GripperCommand \
  "{position: 0.01, max_effort: 5.0}"

# Beispiel: Greifer öffnen (gap=0.08 m)
ros2 topic pub --once /esp32_gripper/command \
  control_msgs/msg/GripperCommand \
  "{position: 0.08, max_effort: 0.0}"
```

### Publications (ESP32 → ROS 2)

| Topic | Message-Typ | Rate | Beschreibung |
|-------|-------------|------|--------------|
| `/esp32_gripper/is_closed` | `std_msgs/msg/Bool` | on change | `true` = Objekt gegriffen & Plattform oben |
| `/esp32_gripper/heartbeat` | `std_msgs/msg/Empty` | 2 Hz | Lebenszeichen des Nodes |

#### Gripper-Zustandsmaschine (zur Information)

```
HOMING_ARMS → HOMING_LIFT → READY
  ↓ (command)
LOWERING → OPENING  (gap ≈ max → öffnen)
         → CLOSING  (gap < max → schließen) → LIFTING → HOLDING
```

---

## 2. Energy Monitor PCB  (`esp32_monitor`)

**Framework:** ESP-IDF + micro-ROS Component  
**Source:** `firmware/esp_wroom_32/monitor/`  
**Sensor:** INA226 @ I2C 0x40 (Shunt 10 mΩ, Current-LSB 0,5 mA)

### Publications (ESP32 → ROS 2)

| Topic | Message-Typ | Rate | Beschreibung |
|-------|-------------|------|--------------|
| `/esp32_monitor/battery_state` | `sensor_msgs/msg/BatteryState` | 1 Hz | Spannung, Strom, Leistung der Batterie |
| `/esp32_monitor/heartbeat` | `std_msgs/msg/Empty` | 1 Hz | Lebenszeichen des Nodes |

#### `sensor_msgs/msg/BatteryState` — belegte Felder

| Feld | Typ | Einheit | Wert |
|------|-----|---------|------|
| `voltage` | `float32` | V | Busspannung (INA226, 1,25 mV/LSB) |
| `current` | `float32` | A | Strom (positiv = Entladung) |
| `charge` | `float32` | Ah | `NaN` (nicht gemessen) |
| `capacity` | `float32` | Ah | `NaN` |
| `design_capacity` | `float32` | Ah | `NaN` |
| `percentage` | `float32` | 0–1 | `NaN` |
| `power_supply_status` | `uint8` | — | `UNKNOWN` (0) |
| `power_supply_health` | `uint8` | — | `UNKNOWN` (0) |
| `power_supply_technology` | `uint8` | — | `UNKNOWN` (0) |
| `present` | `bool` | — | `true` |

> **Leistung** wird intern aus `INA226_REG_POWER` berechnet:
> `power_W = raw × 0,5 mA × 25 = raw × 12,5 mW/LSB`.
> Sie wird aktuell **nicht** in der BatteryState-Message übertragen,
> da kein passendes Feld existiert. Falls gewünscht, kann ein zweiter
> Publisher mit `std_msgs/msg/Float32` auf `/esp32_monitor/power_w`
> hinzugefügt werden.

```bash
# Spannung & Strom live anzeigen
ros2 topic echo /esp32_monitor/battery_state
```

---

## 3. PSU PCB  (`gripper_node`)

**Framework:** Arduino Core + micro_ros_arduino  
**Source:** `firmware/esp_wroom_32/ExampleCodeFlo/BRT7K_ESP32_PSU/src/main.cpp`  
**Sensoren:** 3× NAU7802 (via TCA9548A I2C-Mux @ 0x70)  
**Aktoren:** 2× NeoPixel-Ring (32 LEDs je)

### Publications (ESP32 → ROS 2)

| Topic | Message-Typ | Rate | Beschreibung |
|-------|-------------|------|--------------|
| `gripper/lift_weight` | `std_msgs/msg/Int32` | 10 Hz | Rohwert NAU7802 Hubplattform (Mux-Kanal 7) |
| `gripper/left_weight` | `std_msgs/msg/Int32` | 10 Hz | Rohwert NAU7802 linker Greifarm (Mux-Kanal 6) |
| `gripper/right_weight` | `std_msgs/msg/Int32` | 10 Hz | Rohwert NAU7802 rechter Greifarm (Mux-Kanal 5) |

> Die NAU7802-Rohwerte sind 24-bit-ADC-Counts. Umrechnung in Gramm/Newton
> erfordert eine Kalibrierung (Tara + Skalierungsfaktor).

### Subscriptions (ROS 2 → ESP32)

| Topic | Message-Typ | Beschreibung |
|-------|-------------|--------------|
| `gripper/left_ring_color` | `std_msgs/msg/Int32` | Farbe Ring 1 (links) als 0x00RRGGBB |
| `gripper/right_ring_color` | `std_msgs/msg/Int32` | Farbe Ring 2 (rechts) als 0x00RRGGBB |

#### Farbkodierung (packed RGB)

```
Bit 23..16 → R (0–255)
Bit 15..8  → G (0–255)
Bit  7..0  → B (0–255)
```

```bash
# Ring 1 auf Rot setzen (R=255, G=0, B=0 → 0xFF0000 = 16711680)
ros2 topic pub --once gripper/left_ring_color \
  std_msgs/msg/Int32 "{data: 16711680}"

# Ring 2 auf Grün setzen (0x00FF00 = 65280)
ros2 topic pub --once gripper/right_ring_color \
  std_msgs/msg/Int32 "{data: 65280}"

# Alles aus (0x000000 = 0)
ros2 topic pub --once gripper/left_ring_color  std_msgs/msg/Int32 "{data: 0}"
ros2 topic pub --once gripper/right_ring_color std_msgs/msg/Int32 "{data: 0}"
```

---

## Schnellübersicht aller Topics

| Topic | Richtung | Typ | PCB |
|-------|----------|-----|-----|
| `/esp32_gripper/command` | ROS→ESP | `control_msgs/GripperCommand` | Gripper |
| `/esp32_gripper/is_closed` | ESP→ROS | `std_msgs/Bool` | Gripper |
| `/esp32_gripper/heartbeat` | ESP→ROS | `std_msgs/Empty` | Gripper |
| `/esp32_monitor/battery_state` | ESP→ROS | `sensor_msgs/BatteryState` | Monitor |
| `/esp32_monitor/heartbeat` | ESP→ROS | `std_msgs/Empty` | Monitor |
| `gripper/lift_weight` | ESP→ROS | `std_msgs/Int32` | PSU |
| `gripper/left_weight` | ESP→ROS | `std_msgs/Int32` | PSU |
| `gripper/right_weight` | ESP→ROS | `std_msgs/Int32` | PSU |
| `gripper/left_ring_color` | ROS→ESP | `std_msgs/Int32` | PSU |
| `gripper/right_ring_color` | ROS→ESP | `std_msgs/Int32` | PSU |

---

## micro-ROS Agent starten

```bash
# Alle drei Boards jeweils auf eigenem Port anschließen, dann:
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baud 115200 &
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1 --baud 115200 &
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB2 --baud 115200 &

# Oder über Docker (micro_ros_agent Image):
docker run -it --rm --net=host \
  microros/micro-ros-agent:humble \
  serial --dev /dev/ttyUSB0 -b 115200
```

> **Hinweis:** Der Gripper-Node (ESP-IDF) und der Monitor-Node (ESP-IDF) verbinden
> sich automatisch neu, wenn der Agent neu gestartet wird (`rcl_ret_t`-Reconnect-Loop
> noch nicht implementiert — bei Verbindungsverlust ESP32 neu starten).
