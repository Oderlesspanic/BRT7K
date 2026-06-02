// Bare-minimum NAU7802 test via TCA9548A mux
// SDA=22  SCL=23   Mux @ 0x70
//   ch6 → LiftingScale
//   ch7 → LeftGripperScale
//   ch5 → RightGripperScale

#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"

#define TCA_ADDR 0x70

NAU7802 scaleLift, scaleLeft, scaleRight;

void tcaSelect(uint8_t ch) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  Wire.begin(22, 23);

  while (true) {
    Wire.beginTransmission(TCA_ADDR);
    if (Wire.endTransmission() == 0) { Serial.println("Mux OK"); break; }
    Serial.println("Mux nicht gefunden, retry...");
    delay(1000);
  }

  tcaSelect(6);
  while (!scaleLift.begin())  { Serial.println("LiftScale nicht gefunden..."); delay(1000); }
  Serial.println("LiftScale OK");

  tcaSelect(7);
  while (!scaleLeft.begin())  { Serial.println("LeftScale nicht gefunden..."); delay(1000); }
  Serial.println("LeftScale OK");

  tcaSelect(5);
  while (!scaleRight.begin()) { Serial.println("RightScale nicht gefunden..."); delay(1000); }
  Serial.println("RightScale OK");
}

void loop() {
  static uint32_t t = 0;
  if (millis() - t < 200) return;
  t = millis();

  tcaSelect(6);
  if (scaleLift.available())
    Serial.printf("Lift: %ld\t", (long)scaleLift.getReading());

  tcaSelect(7);
  if (scaleLeft.available())
    Serial.printf("Left: %ld\t", (long)scaleLeft.getReading());

  tcaSelect(5);
  if (scaleRight.available())
    Serial.printf("Right: %ld", (long)scaleRight.getReading());

  Serial.println();
}
