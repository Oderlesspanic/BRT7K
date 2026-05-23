#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"
/*
 * BRT7K - 3x DC motor control via 2x TB6612FNG
 * Platform: ESP32 (Arduino Core 2.x)
 *
 * Control via serial commands (115200 baud).
 * Encoders are read via interrupt (x4 quadrature decoding).
 *
 * !!! IMPORTANT !!!
 * The STBY pins of BOTH TB6612FNG drivers must be HIGH, otherwise
 * the drivers stay in standby and produce no output. If STBY is not
 * wired to a GPIO, solder it directly to 3.3V.
 * GPIO23 is used here as default.
 *
 * Commands:
 *   M <1-3> <-255..255>  Set motor n to speed (sign = direction)
 *   S                    Stop all motors (coast)
 *   B                    Brake all motors (short brake)
 *   E                    Print encoder counts
 *   R                    Reset encoder counts
 *   T                    Toggle telemetry stream
 *   H                    Help
 */

// ---------------- Pin Definitions ----------------
// Motor 1 -> TB6612FNG_1 channel A
#define M1_PWM   25
#define M1_IN1   26
#define M1_IN2   27
#define M1_ENCA   4
#define M1_ENCB   5

// Motor 2 -> TB6612FNG_1 channel B
#define M2_PWM   14
#define M2_IN1   33   // BIN1
#define M2_IN2   32   // BIN2
#define M2_ENCA  13
#define M2_ENCB  15

// Motor 3 -> TB6612FNG_2 channel A
#define M3_PWM   21
#define M3_IN1   19   // AIN1
#define M3_IN2   18   // AIN2
#define M3_ENCA  2
#define M3_ENCB  16

// STBY_PIN: not wired to GPIO — solder TB6612 STBY directly to 3.3V
// LED_PIN:  GPIO22 = IIC SDA, GPIO23 = IIC SCL — no LED pin available on this pinout

// ---------------- Scale Definitions ----------------
#define TCAADDR 0x70

NAU7802 LiftingScale;      // Multiplexer channel 7
NAU7802 LeftGripperScale;  // Multiplexer channel 6
NAU7802 RightGripperScale; // Multiplexer channel 5
uint8_t leftGripperChannel = 7;
uint8_t rightGripperchannel = 5;
uint8_t liftingChannel = 6;
void tcaSelect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// ---------------- PWM Configuration ----------------
const uint32_t PWM_FREQ = 20000;  // 20 kHz: above audible range
const uint8_t  PWM_RES  = 8;      // 8-bit -> 0..255
const int      PWM_MAX  = 255;

// ---------------- Motor Structure ----------------
struct Motor {
  uint8_t pwm, in1, in2;
};

Motor motor[3] = {
  { M1_PWM, M1_IN1, M1_IN2 },
  { M2_PWM, M2_IN1, M2_IN2 },
  { M3_PWM, M3_IN1, M3_IN2 }
};

int motorSpeed[3] = {0, 0, 0};   // current setpoints (for LED / telemetry)

// ---------------- Encoder Structure ----------------
struct Encoder {
  uint8_t pinA, pinB;
  volatile int32_t count;
  volatile uint8_t lastState;
};

Encoder enc[3] = {
  { M1_ENCA, M1_ENCB, 0, 0 },
  { M2_ENCA, M2_ENCB, 0, 0 },
  { M3_ENCA, M3_ENCB, 0, 0 }
};

// Quadrature state table: index = (prevState << 2) | currState
// prev/curr are 2-bit values (A << 1) | B. Result: -1, 0, or +1.
const int8_t QDEC[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

// ---------------- Encoder ISR (x4 decoding) ----------------
void IRAM_ATTR encoderISR(void* arg) {
  Encoder* e = static_cast<Encoder*>(arg);
  uint8_t s = (digitalRead(e->pinA) << 1) | digitalRead(e->pinB);
  e->count += QDEC[(e->lastState << 2) | s];
  e->lastState = s;
}

// ---------------- Motor Functions ----------------
// speed: -255..255  (negative = reverse, 0 = coast)
void setMotor(int i, int speed) {
  if (i < 0 || i > 2) return;
  speed = constrain(speed, -PWM_MAX, PWM_MAX);
  motorSpeed[i] = speed;

  if (speed == 0) {
    // coast: both IN pins LOW
    digitalWrite(motor[i].in1, LOW);
    digitalWrite(motor[i].in2, LOW);
    ledcWrite(i, 0);
  } else {
    bool fwd = (speed > 0);
    digitalWrite(motor[i].in1, fwd ? HIGH : LOW);
    digitalWrite(motor[i].in2, fwd ? LOW  : HIGH);
    ledcWrite(i, abs(speed));
  }
}

void stopAll() {                 // coast
  for (int i = 0; i < 3; i++) setMotor(i, 0);
}

void brakeAll() {                // active braking (short brake)
  for (int i = 0; i < 3; i++) {
    digitalWrite(motor[i].in1, HIGH);
    digitalWrite(motor[i].in2, HIGH);
    ledcWrite(i, 0);
    motorSpeed[i] = 0;
  }
}

// ---------------- Forward Declarations ----------------
void printHelp();

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  delay(300);

  // STBY is hardwired to 3.3V on this board — no GPIO needed

  // Motors: direction pins + PWM
  for (int i = 0; i < 3; i++) {
    pinMode(motor[i].in1, OUTPUT);
    pinMode(motor[i].in2, OUTPUT);
    digitalWrite(motor[i].in1, LOW);
    digitalWrite(motor[i].in2, LOW);
    ledcSetup(i, PWM_FREQ, PWM_RES);
    ledcAttachPin(motor[i].pwm, i);
    ledcWrite(i, 0);
  }

  // Encoders: inputs + interrupts on both channels (x4)
  // Pullups enabled for open-collector encoder outputs.
  // Use INPUT instead for push-pull encoders.
  for (int i = 0; i < 3; i++) {
    pinMode(enc[i].pinA, INPUT_PULLUP);
    pinMode(enc[i].pinB, INPUT_PULLUP);
    enc[i].lastState = (digitalRead(enc[i].pinA) << 1) | digitalRead(enc[i].pinB);
    attachInterruptArg(enc[i].pinA, encoderISR, &enc[i], CHANGE);
    attachInterruptArg(enc[i].pinB, encoderISR, &enc[i], CHANGE);
  }

  // I2C + Scales
  Wire.begin(22,23);  // SDA, SCL

  while (true) {
    Wire.beginTransmission(TCAADDR);
    if (Wire.endTransmission() == 0) {
      Serial.println("Multiplexer found!");
      break;
    }
    Serial.println("Multiplexer not found, retrying...");
    delay(3000);
  }

  tcaSelect(liftingChannel);
  while (!LiftingScale.begin()) {
    Serial.println("LiftingScale not found, retrying...");
    delay(3000);
  }
  // Tare the weightcell
  Serial.println("LiftingScale OK");

  tcaSelect(leftGripperChannel);
  while (!LeftGripperScale.begin()) {
    Serial.println("LeftGripperScale not found, retrying...");
    delay(3000);
  }
  Serial.println("LeftGripperScale OK");

  tcaSelect(rightGripperchannel);
  while (!RightGripperScale.begin()) {
    Serial.println("RightGripperScale not found, retrying...");
    delay(3000);
  }
  Serial.println("RightGripperScale OK");

  Serial.println("\n=== BRT7K motor control ready ===");
  printHelp();
}

// ---------------- Help / Telemetry ----------------
void printHelp() {
  Serial.println("Commands:");
  Serial.println("  M <1-3> <-255..255>  Set motor speed");
  Serial.println("  S  Stop (coast)      B  Brake");
  Serial.println("  E  Show encoders     R  Reset encoders");
  Serial.println("  T  Toggle telemetry  H  Help");
}

void printEncoders() {
  Serial.printf("ENC  M1=%ld  M2=%ld  M3=%ld\n",
                (long)enc[0].count, (long)enc[1].count, (long)enc[2].count);
}

bool telemetryOn = false;

// ---------------- Command Parser ----------------
void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  char c = toupper(cmd.charAt(0));

  switch (c) {
    case 'M': {
      int sp1 = cmd.indexOf(' ');
      int sp2 = (sp1 < 0) ? -1 : cmd.indexOf(' ', sp1 + 1);
      if (sp1 < 0 || sp2 < 0) {
        Serial.println("Usage: M <1-3> <-255..255>");
        return;
      }
      int idx = cmd.substring(sp1 + 1, sp2).toInt();
      int spd = cmd.substring(sp2 + 1).toInt();
      if (idx < 1 || idx > 3) {
        Serial.println("Motor index must be 1..3");
        return;
      }
      setMotor(idx - 1, spd);
      Serial.printf("Motor %d -> %d\n", idx, spd);
      break;
    }
    case 'C':
      for (int i = 0; i < 3; i++) {
        setMotor(i, 0);
        enc[i].count = 0;
      }
      Serial.println("All motors stopped and encoders reset");
      break;
    case 'S':
      stopAll();
      Serial.println("All motors: coast");
      break;
    case 'B':
      brakeAll();
      Serial.println("All motors: brake");
      break;
    case 'E':
      printEncoders();
      break;
    case 'R':
      for (int i = 0; i < 3; i++) enc[i].count = 0;
      Serial.println("Encoders reset");
      break;
    case 'T':
      telemetryOn = !telemetryOn;
      Serial.printf("Telemetry %s\n", telemetryOn ? "ON" : "OFF");
      break;
    case 'H':
    case '?':
      printHelp();
      break;
    default:
      Serial.printf("Unknown command: '%c' (H for help)\n", c);
  }
}

// ---------------- Loop ----------------
void loop() {
  // read serial commands line by line
  static String buf;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (buf.length() > 0) {
        processCommand(buf);
        buf = "";
      }
    } else {
      buf += ch;
    }
  }

  // optional telemetry
  if (telemetryOn) {
    static uint32_t tLast = 0;
    if (millis() - tLast >= 200) {
      tLast = millis();
      printEncoders();
    }
  }

  // Read scales every 100ms (non-blocking)
  static uint32_t sLast = 0;
  if (millis() - sLast >= 500) {
    sLast = millis();

    tcaSelect(liftingChannel);
    if (LiftingScale.available()) {
      Serial.print("Lifting: ");
      Serial.print(LiftingScale.getReading());
      Serial.print("\t | \t");
    }

    tcaSelect(leftGripperChannel);
    if (LeftGripperScale.available()) {
      Serial.print("Left: ");
      Serial.print(LeftGripperScale.getReading());
      Serial.print("\t | \t");
    }

    tcaSelect(rightGripperchannel);
    if (RightGripperScale.available()) {
      Serial.print("Right: ");
      Serial.print(RightGripperScale.getReading());
    }

    Serial.println();
  }

  // Stop M1 if it exceeds 5000 counts
  if(enc[0].count < -5000 && motorSpeed[0] > 0){
    setMotor(0,0);
    Serial.println("Motor 1 stopped (under -5000 counts)");
  }
  if (enc[0].count >= 0 && motorSpeed[0] < 0){
    setMotor(0,0);
    Serial.println("Motor 1 stopped (above 0 counts)");
  }

    // Stop M1 if it exceeds 5000 counts
  if(enc[1].count < -5000 && motorSpeed[1] < 0){
    setMotor(1,0);
    brakeAll();
    Serial.println("Motor 2 stopped (under -5000 counts)");
  }
  if (enc[1].count >= 0 && motorSpeed[1] > 0){
    setMotor(1,0);
    brakeAll();
    Serial.println("Motor 2 stopped (above 0 counts)");
  }

  if (enc[2].count > 20000 && motorSpeed[2] > 0)
  {
    setMotor(2,0);
    brakeAll();
    Serial.println("Motor 3 stopped (above 24000 counts)");
  }

  if (enc[2].count <= 0 && motorSpeed[2] < 0)
  {
    setMotor(2,0);
    brakeAll();
    Serial.println("Motor 3 stopped (below 0 counts)");
  }
  
  
}
