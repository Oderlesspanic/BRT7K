#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"
/*
ardware:

Servo L: GPIO 4 (war GRIP_L_ENCA), Servo R: GPIO 13 (war GRIP_R_ENCA)
LEDC Kanal 0+1 @ 50 Hz, 16-Bit — kein externer Library-Overhead
Lift Motor + Encoder unverändert (Kanal 2 @ 20 kHz)
ROS-Steuerung:

Topic	Typ	Funktion
/esp32_gripper/command	Float32	Direkt-Winkel 0–180°
/esp32_gripper/manual	Int32	0 stop, 1 open, 2 close, 3 lift↑, 4 lift↓
/esp32_gripper/grip_force	Int32	ADC-Schwelle → kraft-geregelt schließen
/esp32_gripper/set_pos	Int32	Lift-Position (3×100000+target+50000)
Kraft-Regelung:


# Schließt Grad für Grad bis Wägezelle um 50000 ADC-Einheiten ansteigt
ros2 topic pub --once /esp32_gripper/grip_force std_msgs/msg/Int32 "{data: 50000}"
Anpassen: SERVO_MAX_DEG = 90 und FORCE_STEP_MS = 50 (ms/Grad) je nach Mechanik. Auch hier: wenn main.cpp aktiv ist, muss eine der Dateien per build_src_filter deaktiviert werden.
*/
/*
 * BRT7K — Gripper Controller SERVO Variant (micro-ROS)
 * Platform : ESP32 WROOM-32 (Arduino Core 2.x)
 *
 * Greifer-Motoren ERSETZT durch Servos:
 *   Servo L  Signal → GPIO  4  (war GRIP_L_ENCA)
 *   Servo R  Signal → GPIO 13  (war GRIP_R_ENCA)
 *   5 V / GND vom Nachbarboard
 *
 * Lift-Motor + Encoder unverändert:
 *   PWM=21  IN1=19  IN2=18  ENC-A=2  ENC-B=16
 *
 * I2C  SDA=22  SCL=23   TCA9548A-Mux @ 0x70
 *   ch7 → LeftGripperScale   ch6 → LiftingScale   ch5 → RightGripperScale
 *
 * Endstop (Lichtschranke)  GPIO17  RISING
 *
 * ROS Topics:
 *   SUB  /esp32_gripper/command     Float32  Servo-Winkel direkt  0-180 °
 *   SUB  /esp32_gripper/manual      Int32    0=stop 1=open 2=close 3=lift↑ 4=lift↓
 *   SUB  /esp32_gripper/grip_force  Int32    ADC-Anstieg als Kraftschwelle → kraft-geregeltes Schließen
 *   SUB  /esp32_gripper/set_pos     Int32    Lift-Pos  (3*100000 + ziel + 50000)
 *   PUB  /esp32_gripper/heartbeat   Empty    2 Hz
 *   PUB  /esp32_gripper/is_closed   Bool     Kraftschwelle erreicht (on change)
 *   PUB  /esp32_gripper/lift_weight Int32    10 Hz
 *   PUB  /esp32_gripper/left_weight Int32    10 Hz
 *   PUB  /esp32_gripper/right_weight Int32   10 Hz
 *   PUB  /esp32_gripper/diagnostics String   1 Hz
 */

#define BRT7K_ROLE "BRT7K_ROLE=gripper"

// ═══════════════════════ Servo Pins ════════════════════════════

#define SERVO_L_PIN   4    // GPIO4  — Encoder-Pin wiederverwendet
#define SERVO_R_PIN   13   // GPIO13

// Servo LEDC — 50 Hz, 16 Bit
// Pulse: 500 µs (0°) … 2500 µs (180°) bei 20 ms Periode
const uint8_t  SERVO_CH_L   = 0;
const uint8_t  SERVO_CH_R   = 1;
const uint32_t SERVO_FREQ   = 50;
const uint8_t  SERVO_RES    = 16;
const uint32_t SERVO_TICK_0   = 1638;  // 500 µs
const uint32_t SERVO_TICK_180 = 8192;  // 2500 µs

#define SERVO_OPEN_DEG  0
#define SERVO_MAX_DEG   90   // mechanisch maximal schließbar — anpassen!

// ═══════════════════════ Lift Motor Pins ═══════════════════════

#define LIFT_PWM    21
#define LIFT_IN1    19
#define LIFT_IN2    18
#define LIFT_ENCA    2
#define LIFT_ENCB   16
#define ENDSTOP_PIN 17

#define IIC_MUX_SDA 22
#define IIC_MUX_SCL 23

// ═══════════════════════ Lift Limits / Speed ═══════════════════

#define LIFT_MIN     0
#define LIFT_MAX 20000

const uint32_t LIFT_PWM_FREQ = 20000;
const uint8_t  LIFT_PWM_RES  = 8;
const uint8_t  LIFT_PWM_CH   = 2;
const int      LIFT_SPEED    = 150;
const int32_t  POS_TOLERANCE = 100;
const int32_t  DECEL_ZONE    = 800;
const int      POS_MAX_SPD   = 150;
const int      POS_MIN_SPD   = 50;

// ═══════════════════════ Scale / I2C ═══════════════════════════

#define TCAADDR 0x70

NAU7802 LiftingScale, LeftGripperScale, RightGripperScale;
const uint8_t CH_GRIP_L = 7, CH_LIFT = 6, CH_GRIP_R = 5;

int32_t scaleReadings[3] = {0, 0, 0};  // [LIFT, LEFT, RIGHT]
bool tcaOk = false, liftScaleOk = false, leftScaleOk = false, rightScaleOk = false;

void tcaSelect(uint8_t ch) {
  if (ch > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

// ═══════════════════════ Lift Encoder ══════════════════════════

struct Encoder { uint8_t pinA, pinB; volatile int32_t count; volatile uint8_t lastState; };
Encoder liftEnc = { LIFT_ENCA, LIFT_ENCB, 0, 0 };

const int8_t QDEC[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

// ═══════════════════════ micro-ROS Handles ══════════════════════

rcl_allocator_t  allocator;
rclc_support_t   support;
rcl_node_t       node;
rclc_executor_t  executor;

rcl_publisher_t    heartbeat_pub;
rcl_publisher_t    is_closed_pub;
rcl_publisher_t    lift_weight_pub;
rcl_publisher_t    left_weight_pub;
rcl_publisher_t    right_weight_pub;
rcl_publisher_t    diagnostics_pub;
rcl_subscription_t cmd_sub;
rcl_subscription_t manual_sub;
rcl_subscription_t grip_force_sub;
rcl_subscription_t set_pos_sub;

std_msgs__msg__Empty  heartbeat_msg;
std_msgs__msg__Bool   is_closed_msg;
std_msgs__msg__Int32  lift_weight_msg;
std_msgs__msg__Int32  left_weight_msg;
std_msgs__msg__Int32  right_weight_msg;
std_msgs__msg__String diagnostics_msg;
std_msgs__msg__Float32 cmd_msg;
std_msgs__msg__Int32  manual_msg;
std_msgs__msg__Int32  grip_force_msg;
std_msgs__msg__Int32  set_pos_msg;

enum class AgentState : uint8_t { WAITING, CONNECTED };
AgentState urosState = AgentState::WAITING;
char diagnosticsText[180] = "INFO boot";
bool serialDiagnosticsEnabled = true;

void setDiagnostics(const char* level, const char* text) {
  snprintf(diagnosticsText, sizeof(diagnosticsText), "%s %s", level, text);
  if (serialDiagnosticsEnabled) Serial.println(diagnosticsText);
}

void publishDiagnostics() {
  rosidl_runtime_c__String__assign(&diagnostics_msg.data, diagnosticsText);
  rcl_publish(&diagnostics_pub, &diagnostics_msg, NULL);
}

// ═══════════════════════ ISRs ══════════════════════════════════

volatile bool endstopTriggered = false;
void IRAM_ATTR endstopISR() { endstopTriggered = true; }

void IRAM_ATTR liftEncoderISR(void* arg) {
  Encoder* e = (Encoder*)arg;
  uint8_t s = (digitalRead(e->pinA) << 1) | digitalRead(e->pinB);
  e->count += QDEC[(e->lastState << 2) | s];
  e->lastState = s;
}

// ═══════════════════════ Servo Control ═════════════════════════

int servoAngle = SERVO_OPEN_DEG;

void setServos(int deg) {
  deg = constrain(deg, 0, SERVO_MAX_DEG);
  servoAngle = deg;
  uint32_t ticks = map(deg, 0, 180, SERVO_TICK_0, SERVO_TICK_180);
  ledcWrite(SERVO_CH_L, ticks);
  ledcWrite(SERVO_CH_R, ticks);
}

// ═══════════════════════ Lift Motor Control ════════════════════

int liftSpeed = 0;

void setLift(int spd) {
  spd = constrain(spd, -255, 255);
  liftSpeed = spd;
  if (spd == 0) {
    digitalWrite(LIFT_IN1, LOW);
    digitalWrite(LIFT_IN2, LOW);
    ledcWrite(LIFT_PWM_CH, 0);
  } else {
    bool fwd = spd > 0;
    digitalWrite(LIFT_IN1, fwd ? HIGH : LOW);
    digitalWrite(LIFT_IN2, fwd ? LOW  : HIGH);
    ledcWrite(LIFT_PWM_CH, abs(spd));
  }
}

void brakeLift() {
  digitalWrite(LIFT_IN1, HIGH);
  digitalWrite(LIFT_IN2, HIGH);
  ledcWrite(LIFT_PWM_CH, 0);
  liftSpeed = 0;
}

// ═══════════════════════ Lift Position Control ═════════════════

struct PosCtrl { bool active; bool arrived; int32_t target; };
PosCtrl liftCtrl = {false, false, 0};

void goToLift(int32_t target) {
  target = constrain(target, LIFT_MIN, LIFT_MAX);
  liftCtrl = {true, false, target};
}

void updateLiftCtrl() {
  if (!liftCtrl.active) return;
  int32_t err = liftCtrl.target - liftEnc.count;
  if (abs(err) <= POS_TOLERANCE) {
    setLift(0);
    if (!liftCtrl.arrived) {
      liftCtrl.arrived = true;
      setDiagnostics("INFO", "Lift Position erreicht");
    }
    return;
  }
  liftCtrl.arrived = false;
  int spd = (abs(err) >= DECEL_ZONE)
    ? POS_MAX_SPD
    : (int)map(abs(err), POS_TOLERANCE, DECEL_ZONE, POS_MIN_SPD, POS_MAX_SPD);
  if (err < 0) spd = -spd;
  setLift(spd);
}

void checkLiftLimits() {
  if (liftEnc.count > LIFT_MAX && liftSpeed > 0) {
    brakeLift(); liftCtrl.active = false;
    setDiagnostics("WARN", "Lift: max limit");
  }
  if (liftEnc.count <= LIFT_MIN && liftSpeed < 0) {
    brakeLift(); liftCtrl.active = false;
    setDiagnostics("WARN", "Lift: min limit");
  }
  if (endstopTriggered) {
    endstopTriggered = false;
    brakeLift(); liftCtrl.active = false;
    setDiagnostics("INFO", "Lift: Endstop");
  }
}

// ═══════════════════════ Force-Grip ════════════════════════════

const int FORCE_STEP_MS = 50;  // ms pro Grad beim kraft-geregelten Schließen

struct ForceGrip {
  bool    active;
  bool    reached;
  int32_t threshold;   // ADC-Anstieg der als Kraft gilt
  int32_t baselineL;   // left-scale beim Start
  int32_t baselineR;   // right-scale beim Start
  uint32_t lastStep;
};
ForceGrip fg = {false, false, 50000, 0, 0, 0};

void startForceGrip(int32_t threshold) {
  fg.threshold = threshold;
  fg.baselineL = scaleReadings[1];
  fg.baselineR = scaleReadings[2];
  fg.lastStep  = millis();
  fg.active    = true;
  fg.reached   = false;
  char msg[64];
  snprintf(msg, sizeof(msg), "ForceGrip start, Schwelle %ld", (long)threshold);
  setDiagnostics("INFO", msg);
}

void updateForceGrip() {
  if (!fg.active) return;

  int32_t dL = scaleReadings[1] - fg.baselineL;
  int32_t dR = scaleReadings[2] - fg.baselineR;

  if (dL >= fg.threshold || dR >= fg.threshold || servoAngle >= SERVO_MAX_DEG) {
    fg.active  = false;
    fg.reached = true;
    setDiagnostics("INFO", "ForceGrip: Kraft erreicht, halte Position");
    return;
  }

  if (millis() - fg.lastStep >= FORCE_STEP_MS) {
    fg.lastStep = millis();
    setServos(servoAngle + 1);
  }
}

// ═══════════════════════ Scale Polling ════════════════════════

void pollScales() {
  static uint32_t t = 0;
  if (millis() - t < 100) return;
  t = millis();

  tcaSelect(CH_LIFT);
  if (LiftingScale.available())     scaleReadings[0] = LiftingScale.getReading();
  tcaSelect(CH_GRIP_L);
  if (LeftGripperScale.available()) scaleReadings[1] = LeftGripperScale.getReading();
  tcaSelect(CH_GRIP_R);
  if (RightGripperScale.available()) scaleReadings[2] = RightGripperScale.getReading();
}

// ═══════════════════════ micro-ROS Callbacks ═══════════════════

void cmd_callback(const void* msgin) {
  // Direkter Servo-Winkel 0-180°
  float angle = ((const std_msgs__msg__Float32*)msgin)->data;
  fg.active = false;
  setServos((int)constrain(angle, 0.0f, 180.0f));
}

void manual_callback(const void* msgin) {
  int32_t cmd = ((const std_msgs__msg__Int32*)msgin)->data;
  switch (cmd) {
    case 0:
      fg.active = false;
      liftCtrl.active = false;
      setServos(servoAngle);  // hält aktuelle Position
      setLift(0);
      setDiagnostics("INFO", "stop");
      break;
    case 1:
      fg.active = false;
      setServos(SERVO_OPEN_DEG);
      setDiagnostics("INFO", "open");
      break;
    case 2:
      fg.active = false;
      setServos(SERVO_MAX_DEG);
      setDiagnostics("INFO", "close");
      break;
    case 3:
      liftCtrl.active = false;
      setLift(LIFT_SPEED);
      setDiagnostics("INFO", "lift up");
      break;
    case 4:
      liftCtrl.active = false;
      setLift(-LIFT_SPEED);
      setDiagnostics("INFO", "lift down");
      break;
    default:
      setDiagnostics("WARN", "unbekannter manual command");
  }
}

void grip_force_callback(const void* msgin) {
  // Kraft-geregeltes Schließen: Servos starten bei 0° und fahren Grad für Grad zu
  int32_t threshold = ((const std_msgs__msg__Int32*)msgin)->data;
  if (threshold <= 0) {
    fg.active = false;
    setDiagnostics("INFO", "ForceGrip abgebrochen");
    return;
  }
  setServos(SERVO_OPEN_DEG);
  delay(200);  // kurz warten bis Servo offen ist
  startForceGrip(threshold);
}

void set_pos_callback(const void* msgin) {
  // Nur Lift wird per set_pos angesteuert: data = 3*100000 + target + 50000
  int32_t val       = ((const std_msgs__msg__Int32*)msgin)->data;
  int32_t motor_idx = val / 100000;
  int32_t target    = (val % 100000) - 50000;
  if (motor_idx != 3) {
    setDiagnostics("WARN", "set_pos: nur Motor 3 (Lift) gueltig");
    return;
  }
  goToLift(target);
  char info[56];
  snprintf(info, sizeof(info), "Lift → %ld", (long)target);
  setDiagnostics("INFO", info);
}

// ═══════════════════════ micro-ROS Lifecycle ═══════════════════

void announceBoardRole() {
  Serial.begin(115200);
  for (int i = 0; i < 20; i++) { Serial.println(BRT7K_ROLE); delay(250); }
}

void createEntities() {
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_gripper", "", &support);

  rclc_publisher_init_default(&heartbeat_pub,   &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),  "/esp32_gripper/heartbeat");
  rclc_publisher_init_default(&is_closed_pub,   &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),   "/esp32_gripper/is_closed");
  rclc_publisher_init_default(&diagnostics_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/esp32_gripper/diagnostics");
  rclc_publisher_init_default(&lift_weight_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),  "/esp32_gripper/lift_weight");
  rclc_publisher_init_default(&left_weight_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),  "/esp32_gripper/left_weight");
  rclc_publisher_init_default(&right_weight_pub,&node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),  "/esp32_gripper/right_weight");

  rclc_subscription_init_default(&cmd_sub,        &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/esp32_gripper/command");
  rclc_subscription_init_default(&manual_sub,     &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),   "/esp32_gripper/manual");
  rclc_subscription_init_default(&grip_force_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),   "/esp32_gripper/grip_force");
  rclc_subscription_init_default(&set_pos_sub,    &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),   "/esp32_gripper/set_pos");

  rclc_executor_init(&executor, &support.context, 4, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_sub,        &cmd_msg,        &cmd_callback,        ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &manual_sub,     &manual_msg,     &manual_callback,     ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &grip_force_sub, &grip_force_msg, &grip_force_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &set_pos_sub,    &set_pos_msg,    &set_pos_callback,    ON_NEW_DATA);

  is_closed_msg.data = false;
  std_msgs__msg__String__init(&diagnostics_msg);
  setDiagnostics("INFO", "micro-ROS verbunden");
  publishDiagnostics();
}

void destroyEntities() {
  rmw_context_t* rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  rcl_publisher_fini(&heartbeat_pub,    &node);
  rcl_publisher_fini(&is_closed_pub,    &node);
  rcl_publisher_fini(&diagnostics_pub,  &node);
  rcl_publisher_fini(&lift_weight_pub,  &node);
  rcl_publisher_fini(&left_weight_pub,  &node);
  rcl_publisher_fini(&right_weight_pub, &node);
  rcl_subscription_fini(&cmd_sub,        &node);
  rcl_subscription_fini(&manual_sub,     &node);
  rcl_subscription_fini(&grip_force_sub, &node);
  rcl_subscription_fini(&set_pos_sub,    &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  std_msgs__msg__String__fini(&diagnostics_msg);
}

// ═══════════════════════ Setup ═════════════════════════════════

void setup() {
  announceBoardRole();
  set_microros_transports();
  serialDiagnosticsEnabled = false;
  delay(2000);

  // Servos (LEDC 50 Hz / 16 Bit)
  ledcSetup(SERVO_CH_L, SERVO_FREQ, SERVO_RES);
  ledcSetup(SERVO_CH_R, SERVO_FREQ, SERVO_RES);
  ledcAttachPin(SERVO_L_PIN, SERVO_CH_L);
  ledcAttachPin(SERVO_R_PIN, SERVO_CH_R);
  setServos(SERVO_OPEN_DEG);

  // Lift Motor
  pinMode(LIFT_IN1, OUTPUT); pinMode(LIFT_IN2, OUTPUT);
  digitalWrite(LIFT_IN1, LOW); digitalWrite(LIFT_IN2, LOW);
  ledcSetup(LIFT_PWM_CH, LIFT_PWM_FREQ, LIFT_PWM_RES);
  ledcAttachPin(LIFT_PWM, LIFT_PWM_CH);
  ledcWrite(LIFT_PWM_CH, 0);

  // Lift Encoder
  pinMode(liftEnc.pinA, INPUT_PULLUP);
  pinMode(liftEnc.pinB, INPUT_PULLUP);
  liftEnc.lastState = (digitalRead(liftEnc.pinA) << 1) | digitalRead(liftEnc.pinB);
  attachInterruptArg(liftEnc.pinA, liftEncoderISR, &liftEnc, CHANGE);
  attachInterruptArg(liftEnc.pinB, liftEncoderISR, &liftEnc, CHANGE);

  // Endstop
  pinMode(ENDSTOP_PIN, INPUT_PULLUP);
  attachInterrupt(ENDSTOP_PIN, endstopISR, RISING);

  // I2C + Scales
  Wire.begin(IIC_MUX_SDA, IIC_MUX_SCL);

  for (int i = 0; i < 10; i++) {
    Wire.beginTransmission(TCAADDR);
    if (Wire.endTransmission() == 0) { tcaOk = true; break; }
    delay(300);
  }

  if (tcaOk) {
    tcaSelect(CH_LIFT);   for (int i=0;i<5;i++) { if (LiftingScale.begin())     { liftScaleOk  = true; break; } delay(300); }
    tcaSelect(CH_GRIP_L); for (int i=0;i<5;i++) { if (LeftGripperScale.begin())  { leftScaleOk  = true; break; } delay(300); }
    tcaSelect(CH_GRIP_R); for (int i=0;i<5;i++) { if (RightGripperScale.begin()) { rightScaleOk = true; break; } delay(300); }
  }
}

// ═══════════════════════ Loop ══════════════════════════════════

void loop() {
  switch (urosState) {

    case AgentState::WAITING: {
      static uint32_t tRetry = 0;
      if (millis() - tRetry >= 500) {
        tRetry = millis();
        if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) {
          createEntities();
          urosState = AgentState::CONNECTED;
        }
      }
      break;
    }

    case AgentState::CONNECTED: {
      static uint32_t tPing = 0;
      if (millis() - tPing >= 1000) {
        tPing = millis();
        if (RMW_RET_OK != rmw_uros_ping_agent(50, 1)) {
          destroyEntities();
          setLift(0);
          urosState = AgentState::WAITING;
          break;
        }
      }

      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

      pollScales();
      updateForceGrip();
      checkLiftLimits();
      updateLiftCtrl();

      // Heartbeat @ 2 Hz
      static uint32_t tHb = 0;
      if (millis() - tHb >= 500) {
        tHb = millis();
        rcl_publish(&heartbeat_pub, &heartbeat_msg, NULL);
      }

      // is_closed — publish on change (ForceGrip Schwelle erreicht)
      static bool prevClosed = false;
      if (fg.reached != prevClosed) {
        prevClosed         = fg.reached;
        is_closed_msg.data = fg.reached;
        rcl_publish(&is_closed_pub, &is_closed_msg, NULL);
      }

      // Weight @ 10 Hz
      static uint32_t tW = 0;
      if (millis() - tW >= 100) {
        tW = millis();
        lift_weight_msg.data  = scaleReadings[0];
        left_weight_msg.data  = scaleReadings[1];
        right_weight_msg.data = scaleReadings[2];
        rcl_publish(&lift_weight_pub,  &lift_weight_msg,  NULL);
        rcl_publish(&left_weight_pub,  &left_weight_msg,  NULL);
        rcl_publish(&right_weight_pub, &right_weight_msg, NULL);
      }

      // Diagnostics @ 1 Hz
      static uint32_t tDiag = 0;
      if (millis() - tDiag >= 1000) {
        tDiag = millis();
        char buf[80];
        snprintf(buf, sizeof(buf), "running servo=%d lift=%ld fg=%s",
          servoAngle, (long)liftEnc.count, fg.active ? "closing" : (fg.reached ? "holding" : "open"));
        setDiagnostics("INFO", buf);
        publishDiagnostics();
      }

      break;
    }
  }
}
