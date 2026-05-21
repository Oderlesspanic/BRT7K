/*
  M5Stack INA226 Energy Monitor (Unit Ammeter, 10A version)
  with 50 ms power/current integration
  ---------------------------------------------------------
  Platform  : Arduino UNO R3
  Interface : I2C  (SDA = A4, SCL = A5, VCC = 5V, GND = GND)
  Address   : 0x41 (set via solder jumper on the back of the unit)
  Shunt     : 2 mOhm  (fixed on the M5Stack 10A module)

  Library   : "INA226" by Rob Tillaart  (install via Library Manager)
  Plotter   : Tools -> Serial Plotter  (baud rate 115200)

  Integration:
    - Runs every 50 ms (20 Hz) via a non-blocking millis() timer.
    - Trapezoidal rule -> energy_Ws  in Joules     (= Watt-seconds)
                          charge_As in Coulombs   (= Ampere-seconds)
    - Conversions: Wh   = energy_Ws / 3600
                   mAh  = charge_As * 1000 / 3600
    - Send 'r' over the serial monitor to reset the accumulators.
*/

#include <Wire.h>
#include <INA226.h>

INA226 ina(0x41);

const uint16_t INTEGRATION_INTERVAL_MS = 50;    // 50 ms -> 20 Hz
const uint16_t PLOT_INTERVAL_MS        = 100;   // 100 ms -> 10 Hz

// ---- Integration accumulators (use double for precision over time) ----
double energy_Ws = 0.0;   // accumulated energy in Joules  (W * s)
double charge_As = 0.0;   // accumulated charge in Coulombs (A * s)

// ---- State for the timers and the trapezoidal rule ----
uint32_t lastIntegrationMs = 0;
uint32_t lastPlotMs        = 0;
float    prevPower_W       = 0.0f;
float    prevCurrent_A     = 0.0f;

// ---- Latest measured values (shared between integration and plot step) ----
float voltage_V = 0.0f;
float current_A = 0.0f;
float power_W   = 0.0f;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!ina.begin()) {
    Serial.println(F("INA226 not found! Please check the wiring."));
    while (1) { delay(1000); }
  }

  if (ina.setMaxCurrentShunt(10.0, 0.002) != 0) {
    Serial.println(F("Calibration failed!"));
    while (1) { delay(1000); }
  }

  ina.setBusVoltageConversionTime(INA226_1100_us);
  ina.setShuntVoltageConversionTime(INA226_1100_us);
  ina.setAverage(INA226_16_SAMPLES);

  delay(50);

  // Prime the trapezoidal integrator with the first sample so the
  // first 50 ms slice doesn't start from zero.
  prevPower_W   = ina.getPower();
  prevCurrent_A = ina.getCurrent();

  uint32_t now = millis();
  lastIntegrationMs = now;
  lastPlotMs        = now;
}

void loop() {
  uint32_t now = millis();

  // ---------------- 50 ms integration step ----------------
  if (now - lastIntegrationMs >= INTEGRATION_INTERVAL_MS) {
    // Use the actual elapsed time so timing jitter does not bias the integral.
    float dt_s = (now - lastIntegrationMs) / 1000.0f;
    lastIntegrationMs = now;

    // Read fresh values from the INA226
    voltage_V = ina.getBusVoltage();
    current_A = ina.getCurrent();
    power_W   = ina.getPower();

    // Trapezoidal rule:  area += (y_prev + y_now) / 2 * dt
    energy_Ws += 0.5 * (prevPower_W   + power_W)   * dt_s;
    charge_As += 0.5 * (prevCurrent_A + current_A) * dt_s;

    prevPower_W   = power_W;
    prevCurrent_A = current_A;
  }

  // ---------------- 100 ms plot step ----------------
  if (now - lastPlotMs >= PLOT_INTERVAL_MS) {
    lastPlotMs = now;

    float energy_Wh  = energy_Ws / 3600.0f;
    float charge_mAh = charge_As * (1000.0f / 3600.0f);

    Serial.print(F("Voltage_V:"));   Serial.print(voltage_V, 3);
    Serial.print(F(",Current_A:"));  Serial.print(current_A, 3);
    Serial.print(F(",Power_W:"));    Serial.print(power_W,   3);
    Serial.print(F(",Energy_Wh:"));  Serial.print(energy_Wh, 4);
    Serial.print(F(",Charge_mAh:")); Serial.println(charge_mAh, 3);
  }

  // ---------------- Optional: reset accumulators with 'r' ----------------
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') {
      energy_Ws = 0.0;
      charge_As = 0.0;
    }
  }
}
