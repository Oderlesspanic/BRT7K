#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <INA226.h>

// --- micro-ROS ---
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/battery_state.h>
#include <geometry_msgs/msg/twist.h>

// ─────────────────────────────────────────────
//  Board role announcement (for auto-detecting which board is which in multi-board setups)
// ─────────────────────────────────────────────
#define BRT7K_ROLE "drive"

// ─────────────────────────────────────────────
//  INA226 (I2C on D33/D32)
// ─────────────────────────────────────────────
#define INA226_ADDR      0x41
#define INA226_SDA       33
#define INA226_SCL       32
#define INA226_INIT_ATTEMPTS 5

// ─────────────────────────────────────────────
//  Battery capacity (adjust to your pack)
// ─────────────────────────────────────────────
#define BATTERY_CAPACITY_WH  36.0f   // 18 V * 2 Ah = 36 Wh
#define BATTERY_CAPACITY_AH   2.0f

// ─────────────────────────────────────────────
//  DFRobot AGV Motors
//  Left:  Serial1  RX=D21  TX=D22
//  Right: Serial2  RX=D16  TX=D17
// ─────────────────────────────────────────────
#define MOTOR_LEFT_RX    16
#define MOTOR_LEFT_TX    17
#define MOTOR_RIGHT_RX   21
#define MOTOR_RIGHT_TX   22
#define MOTOR_BAUD       115200
#define LEFT_ID          0x01
#define RIGHT_ID         0x01
#define MAX_RPM          210.0f

// ─────────────────────────────────────────────
//  SHARP Analog TOF (D4)
//  Assumes GP2Y0A21YK (10-80 cm), 3.3 V, 12-bit ADC
//  distance_cm ~ 27.86 / voltage_V  (calibrate to your unit)
// ─────────────────────────────────────────────
#define TOF_PIN            4
#define TOF_OUT_OF_RANGE  -1.0f

// ─────────────────────────────────────────────
//  Robot kinematics (adjust to your platform)
// ─────────────────────────────────────────────
#define WHEEL_RADIUS_M   0.05f   // m
#define WHEEL_BASE_M     0.30f   // m  (distance between wheels)

// ─────────────────────────────────────────────
//  Status LEDs / NeoPixels
// ─────────────────────────────────────────────
#define LED_RED   18
#define LED_GREEN 19
#define LED_BLUE  23
#define NEO_PIN_1 14
#define NEO_PIN_2 15
#define NUM_LEDS  32

// ─────────────────────────────────────────────
//  Hardware objects
// ─────────────────────────────────────────────
Adafruit_NeoPixel ring1(NUM_LEDS, NEO_PIN_1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ring2(NUM_LEDS, NEO_PIN_2, NEO_GRB + NEO_KHZ800);
INA226 ina226(INA226_ADDR);

// ─────────────────────────────────────────────
//  Integration state (written in loop, read in timers)
// ─────────────────────────────────────────────
static float    s_voltage_V   = 0.0f;
static float    s_current_A   = 0.0f;
static float    s_consumed_wh = 0.0f;
static float    s_consumed_ah = 0.0f;
static uint32_t s_last_ms     = 0;
static float    s_tof_cm      = TOF_OUT_OF_RANGE;
static bool     s_ina226_ok   = false;

// ─────────────────────────────────────────────
//  ROS 2 objects
// ─────────────────────────────────────────────
rcl_publisher_t    pub_battery;
rcl_publisher_t    pub_heartbeat;
rcl_publisher_t    pub_tof;
rcl_subscription_t sub_left_ring;
rcl_subscription_t sub_right_ring;
rcl_subscription_t sub_cmd_vel;
rcl_timer_t        timer_1hz;
rcl_timer_t        timer_10hz;

sensor_msgs__msg__BatteryState  msg_battery;
std_msgs__msg__Empty            msg_heartbeat;
std_msgs__msg__Float32          msg_tof;
std_msgs__msg__Int32            msg_sub_left;
std_msgs__msg__Int32            msg_sub_right;
geometry_msgs__msg__Twist       msg_cmd_vel;

rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;

#define RCCHECK(fn)     { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){errorLoop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// ─────────────────────────────────────────────
//  CRC-8 Dallas/Maxim  (poly 0x31, init 0x00)
// ─────────────────────────────────────────────
static uint8_t crc8_dallas(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  while (len--) {
    uint8_t b = *data++;
    for (uint8_t i = 0; i < 8; ++i) {
      if ((crc ^ b) & 0x01) crc = (crc >> 1) ^ 0x8C;
      else                  crc >>= 1;
      b >>= 1;
    }
  }
  return crc;
}

// ─────────────────────────────────────────────
//  Motor UART helpers
// ─────────────────────────────────────────────
static void drainRx(Stream &port) {
  while (port.available()) port.read();
}

static void sendFrame(Stream &port, uint8_t frame[10]) {
  frame[9] = crc8_dallas(frame, 9);
  port.write(frame, 10);
  port.flush();
}

static void motorSwitchToSpeedLoop(Stream &port, uint8_t id) {
  uint8_t f[10] = { id, 0xA0, 0x02, 0x00, 0,0,0,0,0, 0 };
  sendFrame(port, f);
}

static void motorSetSpeedRPM(Stream &port, uint8_t id, float rpm) {
  rpm = constrain(rpm, -MAX_RPM, MAX_RPM);
  int16_t v  = (int16_t)lroundf(rpm * 10.0f);
  uint8_t f[10] = { id, 0x64, (uint8_t)(v >> 8), (uint8_t)(v & 0xFF), 0,0,0,0,0, 0 };
  sendFrame(port, f);
}

// ─────────────────────────────────────────────
//  SHARP Analog TOF
// ─────────────────────────────────────────────
static float readTofCm() {
  float voltage_V = analogRead(TOF_PIN) * 3.3f / 4095.0f;
  if (voltage_V < 0.3f) return TOF_OUT_OF_RANGE;  // out of range (> ~80 cm)
  return 27.86f / voltage_V;
}

// ─────────────────────────────────────────────
//  LED helpers
// ─────────────────────────────────────────────
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
    for (int i = 0; i < ring1.numPixels(); i++) ring1.setPixelColor(i, ring1.Color(r, g, b));
    ring1.show();
  } else if (ringId == 2) {
    for (int i = 0; i < ring2.numPixels(); i++) ring2.setPixelColor(i, ring2.Color(r, g, b));
    ring2.show();
  }
}

void errorLoop() {
  while (1) { blinkLED(LED_RED, 2, 100); delay(1000); }
}

void announceBoardRole() {
  Serial.begin(115200);

  for (int i = 0; i < 20; i++) {
    Serial.print("BRT7K_ROLE=");
    Serial.println(BRT7K_ROLE);
    Serial.flush();
    delay(250);
  }
}

// ─────────────────────────────────────────────
//  ROS 2 callbacks
// ─────────────────────────────────────────────
void left_ring_cb(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  setRingColor(1, (msg->data >> 16) & 0xFF, (msg->data >> 8) & 0xFF, msg->data & 0xFF);
}

void right_ring_cb(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  setRingColor(2, (msg->data >> 16) & 0xFF, (msg->data >> 8) & 0xFF, msg->data & 0xFF);
}

void cmd_vel_cb(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  float lin = (float)msg->linear.x;   // m/s
  float ang = (float)msg->angular.z;  // rad/s

  // Differential drive: v_l/r = lin -/+ ang * wheelbase/2
  // RPM = velocity / (2*pi*r) * 60
  static constexpr float rpm_factor = 60.0f / (2.0f * 3.14159265f * WHEEL_RADIUS_M);
  float rpm_left  = (lin - ang * WHEEL_BASE_M * 0.5f) * rpm_factor;
  float rpm_right = (lin + ang * WHEEL_BASE_M * 0.5f) * rpm_factor;

  drainRx(Serial1);
  motorSetSpeedRPM(Serial1, LEFT_ID,   rpm_left);
  drainRx(Serial2);
  motorSetSpeedRPM(Serial2, RIGHT_ID, -rpm_right);  // right motor physically mirrored
}

// 1 Hz — BatteryState + heartbeat
void timer_1hz_cb(rcl_timer_t * t, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (t == NULL) return;

  digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));

  msg_battery.voltage    = s_voltage_V;
  msg_battery.current    = s_current_A;
  msg_battery.charge     = BATTERY_CAPACITY_AH - s_consumed_ah;
  msg_battery.capacity   = BATTERY_CAPACITY_AH;
  msg_battery.percentage = 1.0f - (s_consumed_wh / BATTERY_CAPACITY_WH);

  RCSOFTCHECK(rcl_publish(&pub_battery,   &msg_battery,   NULL));
  RCSOFTCHECK(rcl_publish(&pub_heartbeat, &msg_heartbeat, NULL));
}

// 10 Hz — TOF distance
void timer_10hz_cb(rcl_timer_t * t, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (t == NULL) return;

  msg_tof.data = s_tof_cm;
  RCSOFTCHECK(rcl_publish(&pub_tof, &msg_tof, NULL));
}

// ─────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────
void setup() {
  announceBoardRole();

  set_microros_transports();

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  ring1.begin(); ring2.begin();
  ring1.setBrightness(50); ring2.setBrightness(50);
  setRingColor(1, 0, 0, 0); setRingColor(2, 0, 0, 0);

  // INA226 on custom I2C pins
  Wire.begin(INA226_SDA, INA226_SCL);
  for (int attempt = 0; attempt < INA226_INIT_ATTEMPTS; attempt++) {
    if (ina226.begin()) {
      s_ina226_ok = true;
      break;
    }

    blinkLED(LED_RED, 1, 300);
    delay(1000);
  }

  if (s_ina226_ok) {
    ina226.setMaxCurrentShunt(10.0, 0.002);
    ina226.setBusVoltageConversionTime(INA226_1100_us);
    ina226.setShuntVoltageConversionTime(INA226_1100_us);
    ina226.setAverage(INA226_16_SAMPLES);
  } else {
    s_voltage_V = NAN;
    s_current_A = NAN;
  }

  s_last_ms = millis();

  // Motors
  Serial1.begin(MOTOR_BAUD, SERIAL_8N1, MOTOR_LEFT_RX,  MOTOR_LEFT_TX);
  Serial2.begin(MOTOR_BAUD, SERIAL_8N1, MOTOR_RIGHT_RX, MOTOR_RIGHT_TX);
  delay(300);
  motorSwitchToSpeedLoop(Serial1, LEFT_ID);
  delay(10);
  motorSwitchToSpeedLoop(Serial2, RIGHT_ID);
  delay(10);
  motorSetSpeedRPM(Serial1, LEFT_ID,  0.0f);
  motorSetSpeedRPM(Serial2, RIGHT_ID, 0.0f);

  // ADC resolution for SHARP TOF
  analogReadResolution(12);

  // BatteryState initial values
  sensor_msgs__msg__BatteryState__init(&msg_battery);
  msg_battery.temperature             = NAN;
  msg_battery.charge                  = BATTERY_CAPACITY_AH;
  msg_battery.capacity                = BATTERY_CAPACITY_AH;
  msg_battery.design_capacity         = BATTERY_CAPACITY_AH;
  msg_battery.percentage              = 1.0f;
  msg_battery.power_supply_status     = 0;
  msg_battery.power_supply_health     = 0;
  msg_battery.power_supply_technology = 0;
  msg_battery.present                 = true;

  // micro-ROS node
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_drive", "", &support));

  // Publishers
  RCCHECK(rclc_publisher_init_default(&pub_battery, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
    "/battery_state"));
  RCCHECK(rclc_publisher_init_default(&pub_heartbeat, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
    "/esp32_drive/heartbeat"));
  RCCHECK(rclc_publisher_init_default(&pub_tof, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/tof_distance_cm"));

  // Subscriptions
  RCCHECK(rclc_subscription_init_default(&sub_left_ring, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/gripper/left_ring_color"));
  RCCHECK(rclc_subscription_init_default(&sub_right_ring, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/gripper/right_ring_color"));
  RCCHECK(rclc_subscription_init_default(&sub_cmd_vel, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));

  // Timers
  RCCHECK(rclc_timer_init_default(&timer_1hz,  &support, RCL_MS_TO_NS(1000), timer_1hz_cb));
  RCCHECK(rclc_timer_init_default(&timer_10hz, &support, RCL_MS_TO_NS(100),  timer_10hz_cb));

  // Executor: 2 timers + 3 subscriptions = 5 handles
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_1hz));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_10hz));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_left_ring,  &msg_sub_left,  &left_ring_cb,  ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_right_ring, &msg_sub_right, &right_ring_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_vel,    &msg_cmd_vel,   &cmd_vel_cb,    ON_NEW_DATA));

  digitalWrite(LED_GREEN, HIGH);
}

// ─────────────────────────────────────────────
//  Loop — fast INA226 + TOF integration (~6 ms)
// ─────────────────────────────────────────────
void loop() {
  uint32_t now   = millis();
  uint32_t dt_ms = now - s_last_ms;

  if (dt_ms > 0) {
    s_last_ms = now;

    if (s_ina226_ok) {
      float v   = ina226.getBusVoltage();
      float cur = ina226.getCurrent();
      s_voltage_V = v;
      s_current_A = cur;

      float dt_h     = dt_ms / 3600000.0f;
      s_consumed_wh += v * cur * dt_h;  // power integral [Wh]
      s_consumed_ah += cur * dt_h;      // current integral [Ah]

      if (s_consumed_wh < 0.0f) s_consumed_wh = 0.0f;
      if (s_consumed_wh > BATTERY_CAPACITY_WH) s_consumed_wh = BATTERY_CAPACITY_WH;
      if (s_consumed_ah < 0.0f) s_consumed_ah = 0.0f;
      if (s_consumed_ah > BATTERY_CAPACITY_AH) s_consumed_ah = BATTERY_CAPACITY_AH;
    }

    s_tof_cm = readTofCm();
  }

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
  delay(5);
}
