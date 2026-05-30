#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"
#include <Adafruit_NeoPixel.h> // <-- NeoPixel Library

// --- Pin definitions for the Status LEDs ---
#define LED_RED   18
#define LED_GREEN 19
#define LED_BLUE  23

// --- Pin definitions for NeoPixel Rings ---
#define NEO_PIN_1 4
#define NEO_PIN_2 15
#define NUM_LEDS  32

// --- I2C address of the multiplexer ---
#define TCAADDR 0x70

// --- Scale objects ---
NAU7802 LiftingScale;      // Multiplexer channel 7 (Blink code: 1)
NAU7802 LeftGripperScale;  // Multiplexer channel 6 (Blink code: 2)
NAU7802 RightGripperScale; // Multiplexer channel 5 (Blink code: 3)

// --- NeoPixel objects ---
Adafruit_NeoPixel ring1(NUM_LEDS, NEO_PIN_1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ring2(NUM_LEDS, NEO_PIN_2, NEO_GRB + NEO_KHZ800);

// --- Timer Variables ---
long lastWeightReadTime = 0;
const long weightReadInterval = 100; // Read weights every 100ms


// --------------------------------------------------------
// Helper function: Switches the multiplexer to the desired channel (0-7)
// --------------------------------------------------------
void tcaSelect(uint8_t i) {
  if (i > 7) return; 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// --------------------------------------------------------
// Helper function: Blinks a specific Status LED
// --------------------------------------------------------
void blinkLED(uint8_t pin, uint8_t times, int speedMs) {
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, LOW);

  for (uint8_t i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(speedMs);
    digitalWrite(pin, LOW);
    if (i < times - 1) {
      delay(speedMs);
    }
  }
}

// --------------------------------------------------------
// Helper function: Set all LEDs of a specific ring to a color
// Usage: setRingColor(1, 255, 0, 0); // Sets Ring 1 to Red
// Usage: setRingColor(2, 0, 255, 0); // Sets Ring 2 to Green
// --------------------------------------------------------
void setRingColor(int ringId, uint8_t r, uint8_t g, uint8_t b) {
  if (ringId == 1) {
    for(int i = 0; i < ring1.numPixels(); i++) {
      ring1.setPixelColor(i, ring1.Color(r, g, b));
    }
    ring1.show(); // Push data to the LEDs
  } 
  else if (ringId == 2) {
    for(int i = 0; i < ring2.numPixels(); i++) {
      ring2.setPixelColor(i, ring2.Color(r, g, b));
    }
    ring2.show();
  }
}

// --------------------------------------------------------
// Helper function: Turn off all NeoPixels
// --------------------------------------------------------
void clearRings() {
  ring1.clear();
  ring1.show();
  ring2.clear();
  ring2.show();
}

void setup() {
  Serial.begin(115200);

  // 1. Set Status LEDs as output and turn them off
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, LOW);

  // 2. Initialize NeoPixel Rings
  ring1.begin();
  ring2.begin();
  ring1.setBrightness(50); // Max brightness is 255. Keep it low to save power!
  ring2.setBrightness(50);
  clearRings();            // Make sure they are off at startup

  setRingColor(2,255,0,0);
  setRingColor(1,255,255,255);

  // 3. Start I2C bus
  Wire.begin(); 
  Serial.println("System starting...");

  // --------------------------------------------------------
  // 4. Check if Multiplexer is responding (Retry every ~3 sec)
  // --------------------------------------------------------
  while (true) {
    Wire.beginTransmission(TCAADDR);
    if (Wire.endTransmission() == 0) {
      Serial.println("Multiplexer found! Initializing scales...");
      break; 
    }
    
    Serial.println("CRITICAL ERROR: Multiplexer not found! Retrying...");
    blinkLED(LED_RED, 50, 30); 
  }

  // --------------------------------------------------------
  // 5. Initialize Scale 1 (Lifting) - Blink Code 1
  // --------------------------------------------------------
  while (true) {
    tcaSelect(7);
    blinkLED(LED_BLUE, 1, 300); 
    delay(300);
    
    if (LiftingScale.begin() == true) {
      blinkLED(LED_GREEN, 1, 300); 
      Serial.println("LiftingScale OK");
      break; 
    } 
    
    Serial.println("ERROR: LiftingScale not found! Retrying in 3s...");
    blinkLED(LED_RED, 1, 300); 
    delay(3000); 
  }
  delay(600); 

  // --------------------------------------------------------
  // 6. Initialize Scale 2 (Left) - Blink Code 2
  // --------------------------------------------------------
  while (true) {
    tcaSelect(6);
    blinkLED(LED_BLUE, 2, 300); 
    delay(300);
    
    if (LeftGripperScale.begin() == true) {
      blinkLED(LED_GREEN, 2, 300); 
      Serial.println("LeftGripperScale OK");
      break; 
    } 
    
    Serial.println("ERROR: LeftGripperScale not found! Retrying in 3s...");
    blinkLED(LED_RED, 2, 300); 
    delay(3000); 
  }
  delay(600);

  // --------------------------------------------------------
  // 7. Initialize Scale 3 (Right) - Blink Code 3
  // --------------------------------------------------------
  while (true) {
    tcaSelect(5);
    blinkLED(LED_BLUE, 3, 300); 
    delay(300);
    
    if (RightGripperScale.begin() == true) {
      blinkLED(LED_GREEN, 3, 300); 
      Serial.println("RightGripperScale OK");
      break; 
    } 
    
    Serial.println("ERROR: RightGripperScale not found! Retrying in 3s...");
    blinkLED(LED_RED, 3, 300); 
    delay(3000); 
  }

  // --------------------------------------------------------
  // 8. Success! Solid Green light.
  // --------------------------------------------------------
  Serial.println("All 3 scales successfully initialized!");
  digitalWrite(LED_GREEN, HIGH); 
  delay(1000); 
}

void loop() {


  // Blue LED ON (signaling read activity)
  digitalWrite(LED_BLUE, HIGH);

  // --- Read Lifting Scale ---
  tcaSelect(7);
  if (LiftingScale.available() == true) {
    Serial.print("Lifting: ");
    Serial.print(LiftingScale.getReading());
    Serial.print("\t | \t");
  }

  // --- Read Left Gripper Scale ---
  tcaSelect(6);
  if (LeftGripperScale.available() == true) {
    Serial.print("Left: ");
    Serial.print(LeftGripperScale.getReading());
    Serial.print("\t | \t");
  }

  // --- Read Right Gripper Scale ---
  tcaSelect(5);
  if (RightGripperScale.available() == true) {
    Serial.print("Right: ");
    Serial.print(RightGripperScale.getReading());
  }

  Serial.println(); 

  // Blue LED OFF
  digitalWrite(LED_BLUE, LOW);

  // Short delay
  delay(100); 
}