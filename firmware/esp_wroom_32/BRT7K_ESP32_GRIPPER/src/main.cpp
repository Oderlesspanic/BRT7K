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

// Send data: data = motor * 100000 + target + 50000
/*
>>> def calcData(motor,data):
...     return motor * 100000 + data + 50000
...     

*/
/*
 * BRT7K — Gripper Controller (micro-ROS)
 * Platform : ESP32 WROOM-32 (Arduino Core 2.x)
 * Transport: UART0 / USB  @  115200 baud
 *   micro_ros_agent serial --dev /dev/ttyUSBx --baud 115200
 *
 * ┌─────────────┬─────┬─────┬─────┬───────┬───────┐
 * │ Motor       │ PWM │ IN1 │ IN2 │ ENC-A │ ENC-B │
 * ├─────────────┼─────┼─────┼─────┼───────┼───────┤
 * │ Gripper L   │  25 │  26 │  27 │     4 │     5 │
 * │ Gripper R   │  14 │  33 │  32 │    13 │    15 │
 * │ Lifting     │  21 │  19 │  18 │     2 │    16 │
 * └─────────────┴─────┴─────┴─────┴───────┴───────┘
 *
 * I2C  SDA=22  SCL=23   TCA9548A-Mux @ 0x70
 *   ch7 → LeftGripperScale
 *   ch6 → LiftingScale
 *   ch5 → RightGripperScale
 *
 * Endstop (Lichtschranke, Lifting top)  GPIO17  RISING
 *
 * ROS Topics:
 *   SUB  /esp32_gripper/command      std_msgs/Float32  (target gap in m)
 *   SUB  /esp32_gripper/manual       std_msgs/Int32    (0 stop, 1 open, 2 close, 3 lift up, 4 lift down)
 *   SUB  /esp32_gripper/set_pos     std_msgs/Int32  (motor*100000 + target + 50000, motor: 1=GripL 2=GripR 3=Lift)
 *   PUB  /esp32_gripper/is_closed    std_msgs/Bool   (on change)
 *   PUB  /esp32_gripper/heartbeat    std_msgs/Empty  (2 Hz)
 *   PUB  /esp32_gripper/lift_weight  std_msgs/Int32  (10 Hz, tared)
 *   PUB  /esp32_gripper/left_weight  std_msgs/Int32  (10 Hz, tared)
 *   PUB  /esp32_gripper/right_weight std_msgs/Int32  (10 Hz, tared)
 *
 * State machine:
 *   READY → CLOSING → LIFTING → HOLDING
 *   HOLDING → LOWERING → OPENING → READY   (command.position ≈ max)
 *   HOLDING → LOWERING → CLOSING → ...     (command.position < max)
 */

// ─────────────────────────────────────────────
//  Board role announcement (for auto-detecting which board is which in multi-board setups)
// ─────────────────────────────────────────────
#define BRT7K_ROLE "gripper"

// ═══════════════════════ Pin Definitions ════════════════════════

#define GRIP_L_PWM   25
#define GRIP_L_IN1   26
#define GRIP_L_IN2   27
#define GRIP_L_ENCA   4
#define GRIP_L_ENCB   5

#define GRIP_R_PWM   14
#define GRIP_R_IN1   33
#define GRIP_R_IN2   32
#define GRIP_R_ENCA  13
#define GRIP_R_ENCB  15

#define LIFT_PWM     21
#define LIFT_IN1     19
#define LIFT_IN2     18
#define LIFT_ENCA     2
#define LIFT_ENCB    16

#define ENDSTOP_PIN  17
#define IIC_MUX_SDA   22
#define IIC_MUX_SCL   23

// ═══════════════════════ Motor Indices ══════════════════════════

#define MOTOR_GRIP_L  0
#define MOTOR_GRIP_R  1
#define MOTOR_LIFT    2

// ═══════════════════════ Encoder Limits ═════════════════════════

#define GRIP_L_MIN  -6500
#define GRIP_L_MAX      0
#define GRIP_R_MIN      0
#define GRIP_R_MAX   6500
#define LIFT_MIN        0
#define LIFT_MAX    20000

// ═══════════════════════ PWM Config ═════════════════════════════

const uint32_t PWM_FREQ = 20000;
const uint8_t  PWM_RES  = 8;
const int      PWM_MAX  = 255;

// ═══════════════════════ Gripper Constants ══════════════════════

// Physical calibration: max gap at GRIP_L_MAX (0 counts)
const float   GRIP_OPEN_M   = 0.08f;
const float   COUNTS_PER_M  = (float)(-GRIP_L_MIN) / GRIP_OPEN_M;
const int     GRIP_SPEED    = 150;
const int     LIFT_SPEED    = 150;
const int32_t POS_TOLERANCE = 100;

// ═══════════════════════ Position Control Config ═════════════════

const int32_t DECEL_ZONE    = 300;
const int     POS_MAX_SPD   = 100;   // Normalgeschwindigkeit
const int     POS_MIN_SPD   = 250;

// Kraft-Erkennung → Geschwindigkeits-Boost
// left_weight > +THRESHOLD oder right_weight < -THRESHOLD → Boost aktiv
const int32_t FORCE_THRESHOLD = 50000;
const int     FORCE_BOOST_SPD = 250;

// GRIP_L: pos. Speed → Encoder sinkt → Dir -1; GRIP_R und LIFT: Dir +1
const int8_t  posCtrlDir[3]    = { -1, 1, 1 };
const int32_t MOTOR_LIM_MIN[3] = { GRIP_L_MIN, GRIP_R_MIN, LIFT_MIN };
const int32_t MOTOR_LIM_MAX[3] = { GRIP_L_MAX, GRIP_R_MAX, LIFT_MAX };

struct PosCtrl { bool active; bool arrived; int32_t target; };
PosCtrl posCtrl[3] = { {false,false,0}, {false,false,0}, {false,false,0} };

// ═══════════════════════ Scale / I2C ════════════════════════════

#define TCAADDR 0x70

NAU7802 LiftingScale;
NAU7802 LeftGripperScale;
NAU7802 RightGripperScale;

const uint8_t CH_GRIP_L = 7;
const uint8_t CH_LIFT   = 6;
const uint8_t CH_GRIP_R = 5;

int32_t scaleReadings[3] = { 0, 0, 0 }; // [LIFT, GRIP_L, GRIP_R]
int32_t scaleOffsets[3]  = { 0, 0, 0 };
bool tcaOk = false;
bool liftScaleOk = false;
bool leftScaleOk = false;
bool rightScaleOk = false;

void tcaSelect(uint8_t ch) {
  if (ch > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

// ═══════════════════════ Motor / Encoder Structs ════════════════

struct Motor   { uint8_t pwm, in1, in2; };
struct Encoder { uint8_t pinA, pinB; volatile int32_t count; volatile uint8_t lastState; };

Motor motor[3] = {
  { GRIP_L_PWM, GRIP_L_IN1, GRIP_L_IN2 },
  { GRIP_R_PWM, GRIP_R_IN1, GRIP_R_IN2 },
  { LIFT_PWM,   LIFT_IN1,   LIFT_IN2   },
};

Encoder enc[3] = {
  { GRIP_L_ENCA, GRIP_L_ENCB, 0, 0 },
  { GRIP_R_ENCA, GRIP_R_ENCB, 0, 0 },
  { LIFT_ENCA,   LIFT_ENCB,   0, 0 },
};

int motorSpeed[3] = { 0, 0, 0 };

const int8_t QDEC[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

// ═══════════════════════ Gripper State Machine ══════════════════

enum class GripperState : uint8_t {
  READY,
  CLOSING,
  LIFTING,
  HOLDING,
  LOWERING,
  OPENING
};

GripperState gripperState   = GripperState::READY;
bool         newCommand     = false;
double       targetPositionM = GRIP_OPEN_M;
int32_t      cmdTargetCounts = 0;

// ═══════════════════════ micro-ROS Handles ══════════════════════

rcl_allocator_t  allocator;
rclc_support_t   support;
rcl_node_t       node;
rclc_executor_t  executor;

rcl_publisher_t    is_closed_pub;
rcl_publisher_t    heartbeat_pub;
rcl_publisher_t    lift_weight_pub;
rcl_publisher_t    left_weight_pub;
rcl_publisher_t    right_weight_pub;
rcl_publisher_t    diagnostics_pub;
rcl_subscription_t cmd_sub;
rcl_subscription_t manual_sub;
rcl_subscription_t set_pos_sub;

std_msgs__msg__Bool                  is_closed_msg;
std_msgs__msg__Empty                 heartbeat_msg;
std_msgs__msg__Int32                 lift_weight_msg;
std_msgs__msg__Int32                 left_weight_msg;
std_msgs__msg__Int32                 right_weight_msg;
std_msgs__msg__String                diagnostics_msg;
std_msgs__msg__Float32               cmd_msg;
std_msgs__msg__Int32                 manual_msg;
std_msgs__msg__Int32                 set_pos_msg;

enum class AgentState : uint8_t { WAITING, CONNECTED };
AgentState urosState = AgentState::WAITING;
char diagnosticsText[180] = "INFO boot";
bool serialDiagnosticsEnabled = true;
int manualLiftCommand = 0;

void setDiagnostics(const char* level, const char* text) {
  snprintf(diagnosticsText, sizeof(diagnosticsText), "%s %s", level, text);
  if (serialDiagnosticsEnabled) {
    Serial.println(diagnosticsText);
  }
}

void publishDiagnostics() {
  rosidl_runtime_c__String__assign(&diagnostics_msg.data, diagnosticsText);
  rcl_ret_t rc = rcl_publish(&diagnostics_pub, &diagnostics_msg, NULL);
  (void)rc;
}

// ═══════════════════════ ISRs ════════════════════════════════════

volatile bool endstopTriggered = false;
void IRAM_ATTR endstopISR() { endstopTriggered = true; }

void IRAM_ATTR encoderISR(void* arg) {
  Encoder* e = static_cast<Encoder*>(arg);
  uint8_t s = (digitalRead(e->pinA) << 1) | digitalRead(e->pinB);
  e->count += QDEC[(e->lastState << 2) | s];
  e->lastState = s;
}

// ═══════════════════════ Motor Control ══════════════════════════

void setMotor(int i, int speed) {
  if (i < 0 || i > 2) return;
  speed = constrain(speed, -PWM_MAX, PWM_MAX);
  motorSpeed[i] = speed;
  if (speed == 0) {
    digitalWrite(motor[i].in1, LOW);
    digitalWrite(motor[i].in2, LOW);
    ledcWrite(i, 0);
  } else {
    bool fwd = speed > 0;
    digitalWrite(motor[i].in1, fwd ? HIGH : LOW);
    digitalWrite(motor[i].in2, fwd ? LOW  : HIGH);
    ledcWrite(i, abs(speed));
  }
}

void stopAll()  { for (int i = 0; i < 3; i++) setMotor(i, 0); }

void brakeAll() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(motor[i].in1, HIGH);
    digitalWrite(motor[i].in2, HIGH);
    ledcWrite(i, 0);
    motorSpeed[i] = 0;
  }
}

// ═══════════════════════ Soft Limits + Endstop ══════════════════

void checkLimits() {
  // Gripper Left — coast at limits
  if (enc[MOTOR_GRIP_L].count < GRIP_L_MIN && motorSpeed[MOTOR_GRIP_L] > 0)
    setMotor(MOTOR_GRIP_L, 0);
  if (enc[MOTOR_GRIP_L].count >= GRIP_L_MAX && motorSpeed[MOTOR_GRIP_L] < 0)
    setMotor(MOTOR_GRIP_L, 0);

  // Gripper Right — brake at limits
  if (enc[MOTOR_GRIP_R].count < GRIP_R_MIN && motorSpeed[MOTOR_GRIP_R] < 0) {
    setMotor(MOTOR_GRIP_R, 0); brakeAll();
  }
  if (enc[MOTOR_GRIP_R].count >= GRIP_R_MAX && motorSpeed[MOTOR_GRIP_R] > 0) {
    setMotor(MOTOR_GRIP_R, 0); brakeAll();
  }

  // Lifting — brake at encoder limits
  if (enc[MOTOR_LIFT].count > LIFT_MAX && motorSpeed[MOTOR_LIFT] > 0) {
    setMotor(MOTOR_LIFT, 0);
    brakeAll();
    if (gripperState == GripperState::LIFTING)
      gripperState = GripperState::HOLDING;
  }
  if (enc[MOTOR_LIFT].count <= LIFT_MIN && motorSpeed[MOTOR_LIFT] < 0) {
    setMotor(MOTOR_LIFT, 0);
    brakeAll();
  }

  // Lichtschranke Endstop
  if (endstopTriggered) {
    endstopTriggered = false;
    setMotor(MOTOR_LIFT, 0);
    brakeAll();
    if (gripperState == GripperState::LIFTING)
      gripperState = GripperState::HOLDING;
  }
}

// ═══════════════════════ Scale Polling ══════════════════════════

void pollScales() {
  static uint32_t tLast = 0;
  if (millis() - tLast < 100) return;
  tLast = millis();

  if (!tcaOk) return;

  tcaSelect(CH_LIFT);
  if (liftScaleOk  && LiftingScale.available())      scaleReadings[0] = LiftingScale.getReading()      - scaleOffsets[0];

  tcaSelect(CH_GRIP_L);
  if (leftScaleOk  && LeftGripperScale.available())  scaleReadings[1] = LeftGripperScale.getReading()  - scaleOffsets[1];

  tcaSelect(CH_GRIP_R);
  if (rightScaleOk && RightGripperScale.available()) scaleReadings[2] = RightGripperScale.getReading() - scaleOffsets[2];
}

void tareScales() {
  if (!tcaOk) return;
  int64_t sum[3]  = {0, 0, 0};
  int     cnt[3]  = {0, 0, 0};
  uint32_t start  = millis();
  while (millis() - start < 2000) {
    tcaSelect(CH_LIFT);
    if (liftScaleOk  && LiftingScale.available())      { sum[0] += LiftingScale.getReading();      cnt[0]++; }
    tcaSelect(CH_GRIP_L);
    if (leftScaleOk  && LeftGripperScale.available())  { sum[1] += LeftGripperScale.getReading();  cnt[1]++; }
    tcaSelect(CH_GRIP_R);
    if (rightScaleOk && RightGripperScale.available()) { sum[2] += RightGripperScale.getReading(); cnt[2]++; }
    delay(100);
  }
  for (int i = 0; i < 3; i++)
    if (cnt[i] > 0) scaleOffsets[i] = (int32_t)(sum[i] / cnt[i]);
}

bool waitForTca() {
  for (int attempt = 0; attempt < 10; attempt++) {
    Wire.beginTransmission(TCAADDR);
    if (Wire.endTransmission() == 0) {
      setDiagnostics("INFO", "I2C mux TCA9548A gefunden");
      return true;
    }
    setDiagnostics("WARN", "warte auf I2C mux TCA9548A");
    delay(300);
  }

  setDiagnostics("ERROR", "I2C mux TCA9548A nicht gefunden, micro-ROS startet trotzdem");
  return false;
}

bool initScale(NAU7802& scale, uint8_t channel, const char* name) {
  if (!tcaOk) return false;

  tcaSelect(channel);
  for (int attempt = 0; attempt < 5; attempt++) {
    if (scale.begin()) {
      char message[96];
      snprintf(message, sizeof(message), "%s Waage gefunden, tariere", name);
      setDiagnostics("INFO", message);
      scale.calculateZeroOffset(32);
      return true;
    }

    char message[96];
    snprintf(message, sizeof(message), "warte auf %s Waage", name);
    setDiagnostics("WARN", message);
    delay(300);
  }

  char message[112];
  snprintf(message, sizeof(message), "%s Waage nicht gefunden, gripper startet ohne diese Waage", name);
  setDiagnostics("ERROR", message);
  return false;
}

// ═══════════════════════ Gripper Position Control ═══════════════

int32_t positionToCounts(double posM) {
  float p = constrain((float)posM, 0.0f, GRIP_OPEN_M);
  return (int32_t)(-COUNTS_PER_M * (GRIP_OPEN_M - p));
}

bool grippersAtTarget(int32_t target) {
  return abs(enc[MOTOR_GRIP_L].count - target) <= POS_TOLERANCE
      && abs(enc[MOTOR_GRIP_R].count - target) <= POS_TOLERANCE;
}

void driveGrippersTo(int32_t target) {
  // GRIP_L: positive speed = closing (count decreases toward GRIP_L_MIN)
  int32_t eL = enc[MOTOR_GRIP_L].count - target;
  setMotor(MOTOR_GRIP_L, abs(eL) > POS_TOLERANCE ? (eL > 0 ? GRIP_SPEED : -GRIP_SPEED) : 0);

  // GRIP_R: mirrored — negative speed = closing
  int32_t eR = enc[MOTOR_GRIP_R].count - target;
  setMotor(MOTOR_GRIP_R, abs(eR) > POS_TOLERANCE ? (eR > 0 ? -GRIP_SPEED : GRIP_SPEED) : 0);
}

// ═══════════════════════ Position Control ═══════════════════════

void goToPos(int i, int32_t target) {
  if (i < 0 || i > 2) return;
  target = constrain(target, MOTOR_LIM_MIN[i], MOTOR_LIM_MAX[i]);
  posCtrl[i] = { true, false, target };
}

// left_weight (scaleReadings[1]) steigt → >+THRESHOLD
// right_weight (scaleReadings[2]) fällt → <-THRESHOLD
bool forceDetected() {
  return scaleReadings[1] >  FORCE_THRESHOLD
      || scaleReadings[2] < -FORCE_THRESHOLD;
}

void updatePositionControl() {
  bool boost = forceDetected();
  for (int i = 0; i < 3; i++) {
    if (!posCtrl[i].active) continue;
    int32_t err = posCtrl[i].target - enc[i].count;
    if (abs(err) <= POS_TOLERANCE) {
      setMotor(i, 0);
      if (!posCtrl[i].arrived) {
        posCtrl[i].arrived = true;
        char msg[64];
        snprintf(msg, sizeof(msg), "Motor %d Position %ld", i + 1, (long)enc[i].count);
        setDiagnostics("INFO", msg);
      }
      continue;  // active bleibt true → hält Position
    }
    posCtrl[i].arrived = false;
    int spd = boost
      ? FORCE_BOOST_SPD
      : ((abs(err) >= DECEL_ZONE)
          ? POS_MAX_SPD
          : (int)map(abs(err), POS_TOLERANCE, DECEL_ZONE, POS_MIN_SPD, POS_MAX_SPD));
    if ((err * (int32_t)posCtrlDir[i]) < 0) spd = -spd;
    setMotor(i, spd);
  }
}

// ═══════════════════════ State Machine ══════════════════════════

void updateStateMachine() {
  switch (gripperState) {

    case GripperState::READY:
      if (newCommand) {
        newCommand       = false;
        cmdTargetCounts  = positionToCounts(targetPositionM);
        if (cmdTargetCounts < -POS_TOLERANCE)
          gripperState = GripperState::CLOSING;
      }
      break;

    case GripperState::CLOSING:
      driveGrippersTo(cmdTargetCounts);
      if (grippersAtTarget(cmdTargetCounts)) {
        setMotor(MOTOR_LIFT, LIFT_SPEED);
        gripperState = GripperState::LIFTING;
      }
      break;

    case GripperState::LIFTING:
      // motor already started on CLOSING→LIFTING transition
      // checkLimits() handles endstop / LIFT_MAX → transitions to HOLDING
      break;

    case GripperState::HOLDING:
      if (newCommand) {
        newCommand      = false;
        cmdTargetCounts = positionToCounts(targetPositionM);
        setMotor(MOTOR_LIFT, -LIFT_SPEED);
        gripperState = GripperState::LOWERING;
      }
      break;

    case GripperState::LOWERING:
      if (enc[MOTOR_LIFT].count > LIFT_MIN + POS_TOLERANCE) {
        setMotor(MOTOR_LIFT, -LIFT_SPEED);
      } else {
        brakeAll();
        gripperState = (cmdTargetCounts < -POS_TOLERANCE)
          ? GripperState::CLOSING
          : GripperState::OPENING;
      }
      break;

    case GripperState::OPENING:
      driveGrippersTo(GRIP_L_MAX);
      if (grippersAtTarget(GRIP_L_MAX))
        gripperState = GripperState::READY;
      break;
  }
}

// ═══════════════════════ micro-ROS Callback ═════════════════════

void gripper_cmd_callback(const void* msgin) {
  const std_msgs__msg__Float32* msg =
    (const std_msgs__msg__Float32*)msgin;
  for (int i = 0; i < 3; i++) posCtrl[i].active = false;
  manualLiftCommand = 0;
  targetPositionM = msg->data;
  newCommand      = true;
}

void manual_cmd_callback(const void* msgin) {
  const std_msgs__msg__Int32* msg = (const std_msgs__msg__Int32*)msgin;

  switch (msg->data) {
    case 0:
      for (int i = 0; i < 3; i++) posCtrl[i].active = false;
      manualLiftCommand = 0;
      newCommand = false;
      gripperState = GripperState::READY;
      stopAll();
      setDiagnostics("INFO", "manual stop");
      break;
    case 1:
      manualLiftCommand = 0;
      targetPositionM = GRIP_OPEN_M;
      newCommand = true;
      setDiagnostics("INFO", "manual gripper open");
      break;
    case 2:
      manualLiftCommand = 0;
      targetPositionM = 0.0;
      newCommand = true;
      setDiagnostics("INFO", "manual gripper close");
      break;
    case 3:
      manualLiftCommand = 1;
      setDiagnostics("INFO", "manual lift up");
      break;
    case 4:
      manualLiftCommand = -1;
      setDiagnostics("INFO", "manual lift down");
      break;
    default:
      setDiagnostics("WARN", "unbekannter manual command");
      break;
  }
}

// Encoding: motor * 100000 + target + 50000
//   z.B.  Motor 1, Pos -6500 → 143500
//         Motor 2, Pos  6500 → 256500
//         Motor 3, Pos 20000 → 370000
void set_pos_callback(const void* msgin) {
  const std_msgs__msg__Int32* msg = (const std_msgs__msg__Int32*)msgin;
  int32_t val       = msg->data;
  int32_t motor_idx = val / 100000;
  int32_t target    = (val % 100000) - 50000;
  if (motor_idx < 1 || motor_idx > 3) {
    setDiagnostics("WARN", "set_pos: Motor 1=GripL 2=GripR 3=Lift");
    return;
  }
  manualLiftCommand = 0;
  goToPos((int)(motor_idx - 1), target);
  char info[56];
  snprintf(info, sizeof(info), "posCtrl M%d→%ld", (int)motor_idx, (long)target);
  setDiagnostics("INFO", info);
}

// ═══════════════════════ micro-ROS Lifecycle ════════════════════

void announceBoardRole() {
  Serial.begin(115200);

  for (int i = 0; i < 20; i++) {
    Serial.print("BRT7K_ROLE=");
    Serial.println(BRT7K_ROLE);
    Serial.flush();
    delay(250);
  }
}

void createEntities() {
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_gripper", "", &support);

  rclc_publisher_init_default(
    &is_closed_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/esp32_gripper/is_closed");

  rclc_publisher_init_default(
    &heartbeat_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
    "/esp32_gripper/heartbeat");

  rclc_publisher_init_default(
    &diagnostics_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/esp32_gripper/diagnostics");

  rclc_publisher_init_default(
    &lift_weight_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/esp32_gripper/lift_weight");

  rclc_publisher_init_default(
    &left_weight_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/esp32_gripper/left_weight");

  rclc_publisher_init_default(
    &right_weight_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/esp32_gripper/right_weight");

  rclc_subscription_init_default(
    &cmd_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/esp32_gripper/command");

  rclc_subscription_init_default(
    &manual_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/esp32_gripper/manual");

  rclc_subscription_init_default(
    &set_pos_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/esp32_gripper/set_pos");

  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_subscription(
    &executor, &cmd_sub,    &cmd_msg,    &gripper_cmd_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(
    &executor, &manual_sub, &manual_msg, &manual_cmd_callback,  ON_NEW_DATA);
  rclc_executor_add_subscription(
    &executor, &set_pos_sub, &set_pos_msg, &set_pos_callback,   ON_NEW_DATA);

  is_closed_msg.data  = false;
  set_pos_msg.data    = 0;
  std_msgs__msg__String__init(&diagnostics_msg);
  setDiagnostics("INFO", "micro-ROS verbunden, gripper heartbeat aktiv");
  publishDiagnostics();
}

void destroyEntities() {
  rmw_context_t* rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  rcl_publisher_fini(&is_closed_pub,    &node);
  rcl_publisher_fini(&heartbeat_pub,    &node);
  rcl_publisher_fini(&diagnostics_pub,  &node);
  rcl_publisher_fini(&lift_weight_pub,  &node);
  rcl_publisher_fini(&left_weight_pub,  &node);
  rcl_publisher_fini(&right_weight_pub, &node);
  rcl_subscription_fini(&cmd_sub,     &node);
  rcl_subscription_fini(&manual_sub,  &node);
  rcl_subscription_fini(&set_pos_sub, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  std_msgs__msg__String__fini(&diagnostics_msg);
}

// ═══════════════════════ Setup ══════════════════════════════════

void setup() {
  announceBoardRole();

  // micro-ROS serial transport (UART0 / USB)
  set_microros_transports();
  serialDiagnosticsEnabled = false;
  delay(2000);

  // Motors
  for (int i = 0; i < 3; i++) {
    pinMode(motor[i].in1, OUTPUT);
    pinMode(motor[i].in2, OUTPUT);
    digitalWrite(motor[i].in1, LOW);
    digitalWrite(motor[i].in2, LOW);
    ledcSetup(i, PWM_FREQ, PWM_RES);
    ledcAttachPin(motor[i].pwm, i);
    ledcWrite(i, 0);
  }

  // Encoders (INPUT_PULLUP for open-collector outputs)
  for (int i = 0; i < 3; i++) {
    pinMode(enc[i].pinA, INPUT_PULLUP);
    pinMode(enc[i].pinB, INPUT_PULLUP);
    enc[i].lastState = (digitalRead(enc[i].pinA) << 1) | digitalRead(enc[i].pinB);
    attachInterruptArg(enc[i].pinA, encoderISR, &enc[i], CHANGE);
    attachInterruptArg(enc[i].pinB, encoderISR, &enc[i], CHANGE);
  }

  // Endstop Lichtschranke
  pinMode(ENDSTOP_PIN, INPUT_PULLUP);
  attachInterrupt(ENDSTOP_PIN, endstopISR, RISING);

  // I2C + Scales
  Wire.begin(IIC_MUX_SDA, IIC_MUX_SCL);

  tcaOk = waitForTca();
  liftScaleOk = initScale(LiftingScale, CH_LIFT, "Lift");
  leftScaleOk = initScale(LeftGripperScale, CH_GRIP_L, "Links");
  rightScaleOk = initScale(RightGripperScale, CH_GRIP_R, "Rechts");

  tareScales();

  if (!tcaOk || !liftScaleOk || !leftScaleOk || !rightScaleOk) {
    setDiagnostics("WARN", "Hardware unvollstaendig, micro-ROS startet fuer Diagnose und Heartbeat");
  } else {
    setDiagnostics("INFO", "Gripper Hardware initialisiert, Waagen genullt");
  }
}

// ═══════════════════════ Loop ═══════════════════════════════════

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
      // Periodic agent liveness check
      static uint32_t tPing = 0;
      if (millis() - tPing >= 1000) {
        tPing = millis();
        if (RMW_RET_OK != rmw_uros_ping_agent(50, 1)) {
          destroyEntities();
          stopAll();
          urosState = AgentState::WAITING;
          break;
        }
      }

      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

      checkLimits();
      pollScales();
      updatePositionControl();

      bool anyPosCtrl = posCtrl[0].active || posCtrl[1].active || posCtrl[2].active;
      if (!anyPosCtrl) {
        if (manualLiftCommand > 0) {
          setMotor(MOTOR_LIFT, LIFT_SPEED);
        } else if (manualLiftCommand < 0) {
          setMotor(MOTOR_LIFT, -LIFT_SPEED);
        } else {
          updateStateMachine();
        }
      }

      // Heartbeat @ 2 Hz
      static uint32_t tHb = 0;
      if (millis() - tHb >= 500) {
        tHb = millis();
        rcl_publish(&heartbeat_pub, &heartbeat_msg, NULL);
      }

      // Diagnostics @ 1 Hz
      static uint32_t tDiag = 0;
      if (millis() - tDiag >= 1000) {
        tDiag = millis();
        if (!tcaOk) {
          setDiagnostics("ERROR", "I2C mux TCA9548A fehlt");
        } else if (!liftScaleOk || !leftScaleOk || !rightScaleOk) {
          setDiagnostics("WARN", "eine oder mehrere Waagen fehlen");
        } else {
          setDiagnostics("INFO", "running");
        }
        publishDiagnostics();
      }

      // Weight data @ 10 Hz
      static uint32_t tWeight = 0;
      if (millis() - tWeight >= 100) {
        tWeight = millis();
        lift_weight_msg.data  = scaleReadings[0];
        left_weight_msg.data  = scaleReadings[1];
        right_weight_msg.data = scaleReadings[2];
        rcl_publish(&lift_weight_pub,  &lift_weight_msg,  NULL);
        rcl_publish(&left_weight_pub,  &left_weight_msg,  NULL);
        rcl_publish(&right_weight_pub, &right_weight_msg, NULL);
      }

      // is_closed — publish on change
      static bool prevClosed = false;
      bool isClosed = (gripperState == GripperState::HOLDING);
      if (isClosed != prevClosed) {
        prevClosed         = isClosed;
        is_closed_msg.data = isClosed;
        rcl_publish(&is_closed_pub, &is_closed_msg, NULL);
      }
      break;
    }
  }
}
