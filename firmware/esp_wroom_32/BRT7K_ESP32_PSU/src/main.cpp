#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"
#include <Adafruit_NeoPixel.h>

// --- micro-ROS Libraries ---
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/int32.h>

// --- Pin definitions ---
#define LED_RED   18
#define LED_GREEN 19
#define LED_BLUE  23
#define NEO_PIN_1 14
#define NEO_PIN_2 15
#define NUM_LEDS  32
#define TCAADDR   0x70

// --- Hardware Objects ---
NAU7802 LiftingScale;      
NAU7802 LeftGripperScale;  
NAU7802 RightGripperScale; 
Adafruit_NeoPixel ring1(NUM_LEDS, NEO_PIN_1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ring2(NUM_LEDS, NEO_PIN_2, NEO_GRB + NEO_KHZ800);

// --- ROS 2 Objects ---
rcl_publisher_t pub_lift;
rcl_publisher_t pub_left;
rcl_publisher_t pub_right;
rcl_publisher_t pub_heartbeat;
rcl_subscription_t sub_left_ring;
rcl_subscription_t sub_right_ring;

std_msgs__msg__Empty msg_heartbeat;
std_msgs__msg__Int32 msg_lift;
std_msgs__msg__Int32 msg_left;
std_msgs__msg__Int32 msg_right;
std_msgs__msg__Int32 msg_sub_left;
std_msgs__msg__Int32 msg_sub_right;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Error handling macro for ROS
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){errorLoop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// --------------------------------------------------------
// Hardware Helper Functions
// --------------------------------------------------------
void tcaSelect(uint8_t i) {
  if (i > 7) return; 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void blinkLED(uint8_t pin, uint8_t times, int speedMs) {
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, LOW);
  for (uint8_t i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(speedMs);
    digitalWrite(pin, LOW);
    if (i < times - 1) delay(speedMs);
  }
}

void setRingColor(int ringId, uint8_t r, uint8_t g, uint8_t b) {
  if (ringId == 1) {
    for(int i = 0; i < ring1.numPixels(); i++) ring1.setPixelColor(i, ring1.Color(r, g, b));
    ring1.show();
  } else if (ringId == 2) {
    for(int i = 0; i < ring2.numPixels(); i++) ring2.setPixelColor(i, ring2.Color(r, g, b));
    ring2.show();
  }
}

void errorLoop() {
  while(1) {
    blinkLED(LED_RED, 2, 100);
    delay(1000);
  }
}

void announceBoardRole() {
  Serial.begin(115200);

  for (int i = 0; i < 20; i++) {
    Serial.println("BRT7K_ROLE=drive");
    delay(250);
  }
}

// --------------------------------------------------------
// ROS 2 Callbacks
// --------------------------------------------------------

// Callback: When ROS sends a color to the LEFT ring
void left_ring_cb(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  uint8_t r = (msg->data >> 16) & 0xFF;
  uint8_t g = (msg->data >> 8) & 0xFF;
  uint8_t b = msg->data & 0xFF;
  setRingColor(1, r, g, b);
}

// Callback: When ROS sends a color to the RIGHT ring
void right_ring_cb(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  uint8_t r = (msg->data >> 16) & 0xFF;
  uint8_t g = (msg->data >> 8) & 0xFF;
  uint8_t b = msg->data & 0xFF;
  setRingColor(2, r, g, b);
}

// Timer Callback: Reads scales and publishes data to ROS automatically
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    
    // Toggle Blue LED to show ROS activity
    digitalWrite(LED_BLUE, !digitalRead(LED_BLUE)); 
    rcl_publish(&pub_heartbeat, &msg_heartbeat, NULL);

    tcaSelect(7);
    if (LiftingScale.available()) {
      msg_lift.data = LiftingScale.getReading();
      rcl_publish(&pub_lift, &msg_lift, NULL);
    }

    tcaSelect(6);
    if (LeftGripperScale.available()) {
      msg_left.data = LeftGripperScale.getReading();
      rcl_publish(&pub_left, &msg_left, NULL);
    }

    tcaSelect(5);
    if (RightGripperScale.available()) {
      msg_right.data = RightGripperScale.getReading();
      rcl_publish(&pub_right, &msg_right, NULL);
    }
  }
}

// --------------------------------------------------------
// Setup
// --------------------------------------------------------
void setup() {
  announceBoardRole();

  // 1. Configure default Serial for micro-ROS
  set_microros_transports(); 

  // 2. Init LEDs
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  
  // 3. Init NeoPixels
  ring1.begin(); ring2.begin();
  ring1.setBrightness(50); ring2.setBrightness(50);
  setRingColor(1, 0, 0, 0); setRingColor(2, 0, 0, 0);
  
  // 4. Init I2C & Scales (with visual retry logic, NO Serial.prints!)
  Wire.begin(); 
  while (true) {
    Wire.beginTransmission(TCAADDR);
    if (Wire.endTransmission() == 0) break;
    blinkLED(LED_RED, 50, 30); 
  }

  while (true) {
    tcaSelect(7); delay(10);
    if (LiftingScale.begin()) { blinkLED(LED_GREEN, 1, 300); break; }
    blinkLED(LED_RED, 1, 300); delay(2000); 
  }
  while (true) {
    tcaSelect(6); delay(10);
    if (LeftGripperScale.begin()) { blinkLED(LED_GREEN, 2, 300); break; }
    blinkLED(LED_RED, 2, 300); delay(2000); 
  }
  while (true) {
    tcaSelect(5); delay(10);
    if (RightGripperScale.begin()) { blinkLED(LED_GREEN, 3, 300); break; }
    blinkLED(LED_RED, 3, 300); delay(2000); 
  }
  
  // 5. Initialize micro-ROS Data structures
  allocator = rcl_get_default_allocator();

  // Create init_options and node
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "drive", "", &support));

  // Create 3 Publishers
  RCCHECK(rclc_publisher_init_default(&pub_heartbeat, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty), "/esp32_drive/heartbeat"));
  RCCHECK(rclc_publisher_init_default(&pub_lift, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "gripper/lift_weight"));
  RCCHECK(rclc_publisher_init_default(&pub_left, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "gripper/left_weight"));
  RCCHECK(rclc_publisher_init_default(&pub_right, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "gripper/right_weight"));

  // Create 2 Subscribers
  RCCHECK(rclc_subscription_init_default(&sub_left_ring, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "gripper/left_ring_color"));
  RCCHECK(rclc_subscription_init_default(&sub_right_ring, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "gripper/right_ring_color"));

  // Create Timer for publishing (100 ms = 10 Hz)
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  // Create Executor (Handles 1 Timer + 2 Subscribers = 3 Handles)
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_left_ring, &msg_sub_left, &left_ring_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_right_ring, &msg_sub_right, &right_ring_cb, ON_NEW_DATA));

  // Done! System ready.
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_GREEN, HIGH);
}

// --------------------------------------------------------
// Loop
// --------------------------------------------------------
void loop() {
  // Let ROS handle everything
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(5);
}
