#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA260.h>

// Adjust these pins to match your wiring if needed.
static constexpr uint8_t INA260_SDA_PIN = 33;
static constexpr uint8_t INA260_SCL_PIN = 32;

Adafruit_INA260 ina260;

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("INA260 simple test node");

  Wire.begin(INA260_SDA_PIN, INA260_SCL_PIN);

  if (!ina260.begin()) {
    Serial.println("INA260 not found on I2C");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("INA260 ready");
}

void loop() {
  const float current_mA = ina260.readCurrent();
  const float bus_voltage_mV = ina260.readBusVoltage();
  const float power_mW = ina260.readPower();

  Serial.print("Current: ");
  Serial.print(current_mA);
  Serial.println(" mA");

  Serial.print("Bus Voltage: ");
  Serial.print(bus_voltage_mV);
  Serial.println(" mV");

  Serial.print("Power: ");
  Serial.print(power_mW);
  Serial.println(" mW");

  Serial.println("-");
  delay(1000);
}
