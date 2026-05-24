#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"

/*
 * BRT7K — Gripper Controller
 * Platform : ESP32 WROOM-32 (Arduino Core 2.x)
 * Serial   : 115200 baud
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
 * Endstop (Lichtschranke, Lifting)  GPIO17  RISING
 *
 * Commands:
 *   M <1-3> <-255..255>  Set motor speed (1=GripL 2=GripR 3=Lift)
 *   S / B                Stop (coast) / Brake all
 *   E / R                Show / Reset encoder counts
 *   C                    Stop all + reset encoders
 *   T                    Toggle telemetry
 *   H                    Help
 */

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

// ═══════════════════════ Motor Indices ══════════════════════════

#define MOTOR_GRIP_L  0
#define MOTOR_GRIP_R  1
#define MOTOR_LIFT    2

// ═══════════════════════ Encoder Limits ═════════════════════════

#define GRIP_L_MIN  -5000
#define GRIP_L_MAX      0
#define GRIP_R_MIN  -5000
#define GRIP_R_MAX      0
#define LIFT_MIN        0
#define LIFT_MAX    20000

// ═══════════════════════ PWM Config ═════════════════════════════

const uint32_t PWM_FREQ = 20000;
const uint8_t  PWM_RES  = 8;
const int      PWM_MAX  = 255;

// ═══════════════════════ Scale / I2C ════════════════════════════

#define TCAADDR 0x70

NAU7802 LiftingScale;
NAU7802 LeftGripperScale;
NAU7802 RightGripperScale;

const uint8_t CH_GRIP_L = 7;
const uint8_t CH_LIFT   = 6;
const uint8_t CH_GRIP_R = 5;

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

// Quadrature decode: (prevState<<2)|currState → -1/0/+1
const int8_t QDEC[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

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
  if (enc[MOTOR_GRIP_L].count < GRIP_L_MIN && motorSpeed[MOTOR_GRIP_L] > 0) {
    setMotor(MOTOR_GRIP_L, 0);
    Serial.println("GripL: min limit");
  }
  if (enc[MOTOR_GRIP_L].count >= GRIP_L_MAX && motorSpeed[MOTOR_GRIP_L] < 0) {
    setMotor(MOTOR_GRIP_L, 0);
    Serial.println("GripL: max limit");
  }

  // Gripper Right — brake at limits
  if (enc[MOTOR_GRIP_R].count < GRIP_R_MIN && motorSpeed[MOTOR_GRIP_R] < 0) {
    setMotor(MOTOR_GRIP_R, 0);
    brakeAll();
    Serial.println("GripR: min limit");
  }
  if (enc[MOTOR_GRIP_R].count >= GRIP_R_MAX && motorSpeed[MOTOR_GRIP_R] > 0) {
    setMotor(MOTOR_GRIP_R, 0);
    brakeAll();
    Serial.println("GripR: max limit");
  }

  // Lifting — brake at limits
  if (enc[MOTOR_LIFT].count > LIFT_MAX && motorSpeed[MOTOR_LIFT] > 0) {
    setMotor(MOTOR_LIFT, 0);
    brakeAll();
    Serial.println("Lift: max limit");
  }
  if (enc[MOTOR_LIFT].count <= LIFT_MIN && motorSpeed[MOTOR_LIFT] < 0) {
    setMotor(MOTOR_LIFT, 0);
    brakeAll();
    Serial.println("Lift: min limit");
  }

  // Lichtschranke Endstop
  if (endstopTriggered) {
    endstopTriggered = false;
    setMotor(MOTOR_LIFT, 0);
    brakeAll();
    Serial.println("Lift: endstop!");
  }
}

// ═══════════════════════ Scale Polling ══════════════════════════

void pollScales() {
  static uint32_t tLast = 0;
  if (millis() - tLast < 100) return;
  tLast = millis();

  tcaSelect(CH_LIFT);
  if (LiftingScale.available())
    Serial.printf("Lift: %ld\t", (long)LiftingScale.getReading());

  tcaSelect(CH_GRIP_L);
  if (LeftGripperScale.available())
    Serial.printf("GripL: %ld\t", (long)LeftGripperScale.getReading());

  tcaSelect(CH_GRIP_R);
  if (RightGripperScale.available())
    Serial.printf("GripR: %ld", (long)RightGripperScale.getReading());

  Serial.println();
}

// ═══════════════════════ Serial Commands ════════════════════════

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  M <1-3> <-255..255>  Set motor (1=GripL 2=GripR 3=Lift)");
  Serial.println("  S  Stop (coast)      B  Brake all");
  Serial.println("  E  Show encoders     R  Reset encoders");
  Serial.println("  C  Stop + reset      T  Toggle telemetry");
  Serial.println("  H  Help");
}

void printEncoders() {
  Serial.printf("ENC  GripL=%ld  GripR=%ld  Lift=%ld\n",
    (long)enc[MOTOR_GRIP_L].count,
    (long)enc[MOTOR_GRIP_R].count,
    (long)enc[MOTOR_LIFT].count);
}

bool telemetryOn = false;

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  char c = toupper(cmd.charAt(0));

  switch (c) {
    case 'M': {
      int sp1 = cmd.indexOf(' ');
      int sp2 = (sp1 < 0) ? -1 : cmd.indexOf(' ', sp1 + 1);
      if (sp1 < 0 || sp2 < 0) { Serial.println("Usage: M <1-3> <-255..255>"); return; }
      int idx = cmd.substring(sp1 + 1, sp2).toInt();
      int spd = cmd.substring(sp2 + 1).toInt();
      if (idx < 1 || idx > 3) { Serial.println("Motor index must be 1..3"); return; }
      setMotor(idx - 1, spd);
      Serial.printf("Motor %d -> %d\n", idx, spd);
      break;
    }
    case 'C':
      stopAll();
      for (int i = 0; i < 3; i++) enc[i].count = 0;
      Serial.println("All stopped + encoders reset");
      break;
    case 'S': stopAll();  Serial.println("Coast"); break;
    case 'B': brakeAll(); Serial.println("Brake"); break;
    case 'E': printEncoders(); break;
    case 'R':
      for (int i = 0; i < 3; i++) enc[i].count = 0;
      Serial.println("Encoders reset");
      break;
    case 'T':
      telemetryOn = !telemetryOn;
      Serial.printf("Telemetry %s\n", telemetryOn ? "ON" : "OFF");
      break;
    case 'H': case '?': printHelp(); break;
    default: Serial.printf("Unknown: '%c'  (H for help)\n", c);
  }
}

// ═══════════════════════ Setup ══════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(300);

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
  Wire.begin(22, 23);

  while (true) {
    Wire.beginTransmission(TCAADDR);
    if (Wire.endTransmission() == 0) { Serial.println("Mux OK"); break; }
    Serial.println("Mux not found, retrying...");
    delay(3000);
  }

  tcaSelect(CH_LIFT);
  while (!LiftingScale.begin()) { Serial.println("LiftingScale not found..."); delay(3000); }
  Serial.println("LiftingScale OK");

  tcaSelect(CH_GRIP_L);
  while (!LeftGripperScale.begin()) { Serial.println("LeftGripperScale not found..."); delay(3000); }
  Serial.println("LeftGripperScale OK");

  tcaSelect(CH_GRIP_R);
  while (!RightGripperScale.begin()) { Serial.println("RightGripperScale not found..."); delay(3000); }
  Serial.println("RightGripperScale OK");

  Serial.println("\n=== BRT7K Gripper ready ===");
  printHelp();
}

// ═══════════════════════ Loop ═══════════════════════════════════

void loop() {
  // Serial command parser
  static String buf;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (buf.length() > 0) { processCommand(buf); buf = ""; }
    } else {
      buf += ch;
    }
  }

  checkLimits();
  pollScales();

  if (telemetryOn) {
    static uint32_t tLast = 0;
    if (millis() - tLast >= 200) { tLast = millis(); printEncoders(); }
  }
}
