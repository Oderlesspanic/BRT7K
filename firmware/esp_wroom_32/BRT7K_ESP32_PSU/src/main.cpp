#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_INA260.h>
#include <string.h>

// --- micro-ROS ---
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/battery_state.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rmw_microros/time_sync.h>

// ─────────────────────────────────────────────
//  Board role announcement (for auto-detecting which board is which in multi-board setups)
// ─────────────────────────────────────────────
#define BRT7K_ROLE "drive"

// ─────────────────────────────────────────────
//  INA260 (I2C on D33/D32)
// ─────────────────────────────────────────────
#define INA260_SDA       33
#define INA260_SCL       32
#define INA260_INIT_ATTEMPTS 5

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
#define MOTOR_RIGHT_RX   22
#define MOTOR_RIGHT_TX   21
#define MOTOR_BAUD       115200
#define LEFT_ID          0x01
#define RIGHT_ID         0x01
#define MAX_RPM          210.0f
#define MOTOR_RAMP_RPM_PER_S 120.0f
#define MOTOR_RAMP_UPDATE_MS 20

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
#define WHEEL_RADIUS_M   0.03656f  // m, matches robot_description wheel_radius
#define WHEEL_BASE_M     0.3212f   // m, matches robot_description 2 * wheel_y
#define MAX_CMD_LINEAR_MPS   0.10f
#define MAX_CMD_ANGULAR_RADPS 0.25f
#define ENCODER_STEPS_PER_REV 65536.0f  // 0–65535 per protocol = one full revolution
#define ODOM_PUBLISH_MS 100

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
Adafruit_INA260 ina260;

// ─────────────────────────────────────────────
//  Integration state (written in loop, read in timers)
// ─────────────────────────────────────────────
static float    s_voltage_V   = 0.0f;
static float    s_current_A   = 0.0f;
static float    s_consumed_wh = 0.0f;
static float    s_consumed_ah = 0.0f;
static uint32_t s_last_ms     = 0;
static float    s_tof_cm      = TOF_OUT_OF_RANGE;
static bool     s_ina260_ok   = false;

static bool     s_odom_ready       = false;
static float    s_last_left_rad    = 0.0f;
static float    s_last_right_rad   = 0.0f;
static uint32_t s_last_odom_ms     = 0;
static float    s_odom_x_m         = 0.0f;
static float    s_odom_y_m         = 0.0f;
static float    s_odom_yaw_rad     = 0.0f;

static float    s_target_left_rpm  = 0.0f;
static float    s_target_right_rpm = 0.0f;
static float    s_current_left_rpm = 0.0f;
static float    s_current_right_rpm = 0.0f;
static uint32_t s_last_motor_ramp_ms = 0;

// ─────────────────────────────────────────────
//  ROS 2 objects
// ─────────────────────────────────────────────
rcl_publisher_t    pub_battery;
rcl_publisher_t    pub_heartbeat;
rcl_publisher_t    pub_tof;
rcl_publisher_t    pub_wheel_odom;
rcl_publisher_t    pub_diagnostics;
rcl_subscription_t sub_left_ring;
rcl_subscription_t sub_right_ring;
rcl_subscription_t sub_cmd_vel;
rcl_timer_t        timer_1hz;
rcl_timer_t        timer_10hz;
rcl_timer_t        timer_odom;

sensor_msgs__msg__BatteryState  msg_battery;
std_msgs__msg__Empty            msg_heartbeat;
std_msgs__msg__Float32          msg_tof;
std_msgs__msg__String           msg_diagnostics;
nav_msgs__msg__Odometry         msg_wheel_odom;
std_msgs__msg__Int32            msg_sub_left;
std_msgs__msg__Int32            msg_sub_right;
geometry_msgs__msg__Twist       msg_cmd_vel;

rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;

static char s_diagnostics_text[180] = "INFO boot";
static bool s_uros_entities_created = false;

enum class AgentState {
  WAITING,
  CONNECTED,
};

static AgentState s_uros_state = AgentState::WAITING;

static void setDiagnostics(const char * level, const char * text) {
  snprintf(s_diagnostics_text, sizeof(s_diagnostics_text), "%s %s", level, text);
}

static void publishDiagnostics() {
  rosidl_runtime_c__String__assign(&msg_diagnostics.data, s_diagnostics_text);
  rcl_ret_t rc = rcl_publish(&pub_diagnostics, &msg_diagnostics, NULL);
  (void)rc;
}

#define RCCHECK(fn)     { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){setDiagnostics("ERROR", #fn); return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc;}
#define RCIGNORE(fn)    { rcl_ret_t temp_rc = fn; (void)temp_rc;}

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

static bool readFrame(Stream &port, uint8_t expected_id, uint8_t expected_cmd, uint8_t frame[10], uint32_t timeout_ms) {
  uint8_t window[10] = {0};
  size_t count = 0;
  uint32_t start = millis();

  while ((millis() - start) < timeout_ms) {
    while (port.available()) {
      uint8_t b = (uint8_t)port.read();
      if (count < 10) {
        window[count++] = b;
      } else {
        memmove(window, window + 1, 9);
        window[9] = b;
      }

      if (count == 10 &&
          window[0] == expected_id &&
          window[1] == expected_cmd &&
          crc8_dallas(window, 9) == window[9]) {
        memcpy(frame, window, 10);
        return true;
      }
    }
    delay(1);
  }

  return false;
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

static float rampToward(float current, float target, float max_delta) {
  float delta = target - current;
  if (delta > max_delta) return current + max_delta;
  if (delta < -max_delta) return current - max_delta;
  return target;
}

static void updateMotorRamp() {
  uint32_t now = millis();
  uint32_t dt_ms = now - s_last_motor_ramp_ms;
  if (dt_ms < MOTOR_RAMP_UPDATE_MS) return;

  s_last_motor_ramp_ms = now;
  float max_delta = MOTOR_RAMP_RPM_PER_S * (dt_ms / 1000.0f);
  s_current_left_rpm = rampToward(s_current_left_rpm, s_target_left_rpm, max_delta);
  s_current_right_rpm = rampToward(s_current_right_rpm, s_target_right_rpm, max_delta);

  drainRx(Serial1);
  motorSetSpeedRPM(Serial1, LEFT_ID, s_current_left_rpm);
  drainRx(Serial2);
  motorSetSpeedRPM(Serial2, RIGHT_ID, -s_current_right_rpm);  // right motor physically mirrored
}

static void stopDriveMotion() {
  s_target_left_rpm = 0.0f;
  s_target_right_rpm = 0.0f;
  s_current_left_rpm = 0.0f;
  s_current_right_rpm = 0.0f;
  drainRx(Serial1);
  motorSetSpeedRPM(Serial1, LEFT_ID, 0.0f);
  drainRx(Serial2);
  motorSetSpeedRPM(Serial2, RIGHT_ID, 0.0f);
}

static bool motorReadMileage(Stream &port, uint8_t id, int32_t &laps, uint16_t &position) {
  drainRx(port);
  uint8_t f[10] = { id, 0x74, 0,0,0,0,0,0,0, 0 };
  sendFrame(port, f);

  uint8_t r[10];
  if (!readFrame(port, id, 0x74, r, 25)) return false;

  uint32_t raw_laps =
    ((uint32_t)r[2] << 24) |
    ((uint32_t)r[3] << 16) |
    ((uint32_t)r[4] << 8)  |
    ((uint32_t)r[5]);
  laps = (int32_t)raw_laps;
  position = ((uint16_t)r[6] << 8) | (uint16_t)r[7];

  return true;  // any uint16_t 0–65535 is valid per protocol
}

static float normalizeAngle(float angle) {
  while (angle > 3.14159265f) angle -= 2.0f * 3.14159265f;
  while (angle < -3.14159265f) angle += 2.0f * 3.14159265f;
  return angle;
}

static void stampNow(nav_msgs__msg__Odometry & msg) {
  int64_t now_ns = rmw_uros_epoch_nanos();
  if (now_ns <= 0) {
    now_ns = (int64_t)millis() * 1000000LL;
  }

  msg.header.stamp.sec = (int32_t)(now_ns / 1000000000LL);
  msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);
}

static void initWheelOdomMessage() {
  nav_msgs__msg__Odometry__init(&msg_wheel_odom);
  rosidl_runtime_c__String__assign(&msg_wheel_odom.header.frame_id, "odom");
  rosidl_runtime_c__String__assign(&msg_wheel_odom.child_frame_id, "base_link");

  for (size_t i = 0; i < 36; i++) {
    msg_wheel_odom.pose.covariance[i] = 0.0;
    msg_wheel_odom.twist.covariance[i] = 0.0;
  }

  msg_wheel_odom.pose.covariance[0] = 0.02;
  msg_wheel_odom.pose.covariance[7] = 0.02;
  msg_wheel_odom.pose.covariance[14] = 1e6;
  msg_wheel_odom.pose.covariance[21] = 1e6;
  msg_wheel_odom.pose.covariance[28] = 1e6;
  msg_wheel_odom.pose.covariance[35] = 0.05;

  msg_wheel_odom.twist.covariance[0] = 0.02;
  msg_wheel_odom.twist.covariance[7] = 1e6;
  msg_wheel_odom.twist.covariance[14] = 1e6;
  msg_wheel_odom.twist.covariance[21] = 1e6;
  msg_wheel_odom.twist.covariance[28] = 1e6;
  msg_wheel_odom.twist.covariance[35] = 0.05;
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

  lin = constrain(lin, -MAX_CMD_LINEAR_MPS, MAX_CMD_LINEAR_MPS);
  ang = constrain(ang, -MAX_CMD_ANGULAR_RADPS, MAX_CMD_ANGULAR_RADPS);

  if (fabsf(lin) < 0.001f && fabsf(ang) < 0.001f) {
    stopDriveMotion();
    setDiagnostics("INFO", "cmd_vel stop sofort ausgefuehrt");
    return;
  }

  // Differential drive: v_l/r = lin -/+ ang * wheelbase/2
  // RPM = velocity / (2*pi*r) * 60
  static constexpr float rpm_factor = 60.0f / (2.0f * 3.14159265f * WHEEL_RADIUS_M);
  float rpm_left  = (lin - ang * WHEEL_BASE_M * 0.5f) * rpm_factor;
  float rpm_right = (lin + ang * WHEEL_BASE_M * 0.5f) * rpm_factor;

  s_target_left_rpm = constrain(rpm_left, -MAX_RPM, MAX_RPM);
  s_target_right_rpm = constrain(rpm_right, -MAX_RPM, MAX_RPM);
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

  if (s_ina260_ok) {
    setDiagnostics("INFO", "running");
  } else {
    setDiagnostics("WARN", "running, INA260 fehlt");
  }

  RCSOFTCHECK(rcl_publish(&pub_battery,   &msg_battery,   NULL));
  RCSOFTCHECK(rcl_publish(&pub_heartbeat, &msg_heartbeat, NULL));
  publishDiagnostics();
}

// 10 Hz — TOF distance
void timer_10hz_cb(rcl_timer_t * t, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (t == NULL) return;

  msg_tof.data = s_tof_cm;
  RCSOFTCHECK(rcl_publish(&pub_tof, &msg_tof, NULL));
}

// 10 Hz — wheel odometry from motor encoder mileage
void timer_odom_cb(rcl_timer_t * t, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (t == NULL) return;

  int32_t left_laps = 0;
  int32_t right_laps = 0;
  uint16_t left_position = 0;
  uint16_t right_position = 0;

  if (!motorReadMileage(Serial1, LEFT_ID, left_laps, left_position)) {
    setDiagnostics("ERROR", "linker Motor liefert keine Encoder-Mileage");
    return;
  }
  if (!motorReadMileage(Serial2, RIGHT_ID, right_laps, right_position)) {
    setDiagnostics("ERROR", "rechter Motor liefert keine Encoder-Mileage");
    return;
  }

  const float left_revolutions =
    (float)left_laps + ((float)left_position / ENCODER_STEPS_PER_REV);
  const float right_revolutions =
    (float)right_laps + ((float)right_position / ENCODER_STEPS_PER_REV);

  const float left_rad = left_revolutions * 2.0f * 3.14159265f;
  const float right_rad = -right_revolutions * 2.0f * 3.14159265f;  // right motor is mounted mirrored
  const uint32_t now_ms = millis();

  if (!s_odom_ready) {
    s_odom_ready = true;
    s_last_left_rad = left_rad;
    s_last_right_rad = right_rad;
    s_last_odom_ms = now_ms;
  }

  const uint32_t dt_ms = now_ms - s_last_odom_ms;
  if (dt_ms == 0) return;

  const float dt_s = dt_ms / 1000.0f;
  const float delta_left_m = (left_rad - s_last_left_rad) * WHEEL_RADIUS_M;
  const float delta_right_m = (right_rad - s_last_right_rad) * WHEEL_RADIUS_M;
  const float delta_center_m = 0.5f * (delta_left_m + delta_right_m);
  const float delta_yaw_rad = (delta_right_m - delta_left_m) / WHEEL_BASE_M;
  const float mid_yaw_rad = s_odom_yaw_rad + 0.5f * delta_yaw_rad;

  s_odom_x_m += delta_center_m * cosf(mid_yaw_rad);
  s_odom_y_m += delta_center_m * sinf(mid_yaw_rad);
  s_odom_yaw_rad = normalizeAngle(s_odom_yaw_rad + delta_yaw_rad);

  s_last_left_rad = left_rad;
  s_last_right_rad = right_rad;
  s_last_odom_ms = now_ms;

  const float linear_velocity_mps = delta_center_m / dt_s;
  const float angular_velocity_radps = delta_yaw_rad / dt_s;
  const float half_yaw = 0.5f * s_odom_yaw_rad;

  stampNow(msg_wheel_odom);
  msg_wheel_odom.pose.pose.position.x = s_odom_x_m;
  msg_wheel_odom.pose.pose.position.y = s_odom_y_m;
  msg_wheel_odom.pose.pose.position.z = 0.0;
  msg_wheel_odom.pose.pose.orientation.x = 0.0;
  msg_wheel_odom.pose.pose.orientation.y = 0.0;
  msg_wheel_odom.pose.pose.orientation.z = sinf(half_yaw);
  msg_wheel_odom.pose.pose.orientation.w = cosf(half_yaw);
  msg_wheel_odom.twist.twist.linear.x = linear_velocity_mps;
  msg_wheel_odom.twist.twist.linear.y = 0.0;
  msg_wheel_odom.twist.twist.linear.z = 0.0;
  msg_wheel_odom.twist.twist.angular.x = 0.0;
  msg_wheel_odom.twist.twist.angular.y = 0.0;
  msg_wheel_odom.twist.twist.angular.z = angular_velocity_radps;

  RCSOFTCHECK(rcl_publish(&pub_wheel_odom, &msg_wheel_odom, NULL));
}

// ─────────────────────────────────────────────
//  micro-ROS lifecycle
// ─────────────────────────────────────────────
static bool createEntities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  rmw_uros_sync_session(1000);
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
  RCCHECK(rclc_publisher_init_default(&pub_wheel_odom, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "/wheel/odometry"));
  RCCHECK(rclc_publisher_init_default(&pub_diagnostics, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/esp32_drive/diagnostics"));

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
  RCCHECK(rclc_timer_init_default(&timer_odom, &support, RCL_MS_TO_NS(ODOM_PUBLISH_MS), timer_odom_cb));

  // Executor: 3 timers + 3 subscriptions = 6 handles
  RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_1hz));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_10hz));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_odom));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_left_ring,  &msg_sub_left,  &left_ring_cb,  ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_right_ring, &msg_sub_right, &right_ring_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_vel,    &msg_cmd_vel,   &cmd_vel_cb,    ON_NEW_DATA));

  s_uros_entities_created = true;
  digitalWrite(LED_GREEN, HIGH);
  setDiagnostics("INFO", "micro-ROS verbunden, drive heartbeat und wheel odometry aktiv");
  publishDiagnostics();
  return true;
}

static void destroyEntities() {
  if (!s_uros_entities_created) return;

  rmw_context_t* rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCIGNORE(rcl_timer_fini(&timer_1hz));
  RCIGNORE(rcl_timer_fini(&timer_10hz));
  RCIGNORE(rcl_timer_fini(&timer_odom));
  RCIGNORE(rcl_subscription_fini(&sub_left_ring, &node));
  RCIGNORE(rcl_subscription_fini(&sub_right_ring, &node));
  RCIGNORE(rcl_subscription_fini(&sub_cmd_vel, &node));
  RCIGNORE(rcl_publisher_fini(&pub_battery, &node));
  RCIGNORE(rcl_publisher_fini(&pub_heartbeat, &node));
  RCIGNORE(rcl_publisher_fini(&pub_tof, &node));
  RCIGNORE(rcl_publisher_fini(&pub_wheel_odom, &node));
  RCIGNORE(rcl_publisher_fini(&pub_diagnostics, &node));
  RCIGNORE(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));

  s_uros_entities_created = false;
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

  // INA260 on custom I2C pins
  Wire.begin(INA260_SDA, INA260_SCL);
  for (int attempt = 0; attempt < INA260_INIT_ATTEMPTS; attempt++) {
    if (ina260.begin()) {
      s_ina260_ok = true;
      break;
    }

    blinkLED(LED_RED, 1, 300);
    delay(1000);
  }

  if (s_ina260_ok) {
    setDiagnostics("INFO", "INA260 initialisiert");
  } else {
    s_voltage_V = NAN;
    s_current_A = NAN;
    setDiagnostics("WARN", "INA260 nicht gefunden, drive startet ohne Batteriemessung");
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
  s_last_motor_ramp_ms = millis();

  // ADC resolution for SHARP TOF
  analogReadResolution(12);

  // BatteryState initial values
  sensor_msgs__msg__BatteryState__init(&msg_battery);
  std_msgs__msg__String__init(&msg_diagnostics);
  msg_battery.temperature             = NAN;
  msg_battery.charge                  = BATTERY_CAPACITY_AH;
  msg_battery.capacity                = BATTERY_CAPACITY_AH;
  msg_battery.design_capacity         = BATTERY_CAPACITY_AH;
  msg_battery.percentage              = 1.0f;
  msg_battery.power_supply_status     = 0;
  msg_battery.power_supply_health     = 0;
  msg_battery.power_supply_technology = 0;
  msg_battery.present                 = true;
  initWheelOdomMessage();

  digitalWrite(LED_GREEN, LOW);
  setDiagnostics("INFO", "Drive Hardware initialisiert, warte auf micro-ROS Agent");
}

// ─────────────────────────────────────────────
//  Loop — fast INA260 + TOF integration (~6 ms)
// ─────────────────────────────────────────────
void loop() {
  uint32_t now   = millis();
  uint32_t dt_ms = now - s_last_ms;

  if (dt_ms > 0) {
    s_last_ms = now;

    if (s_ina260_ok) {
      float v   = ina260.readBusVoltage() / 1000.0f;
      float cur = ina260.readCurrent() / 1000.0f;
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

  updateMotorRamp();

  switch (s_uros_state) {
    case AgentState::WAITING: {
      static uint32_t tRetry = 0;
      if (millis() - tRetry >= 500) {
        tRetry = millis();
        if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) {
          if (createEntities()) {
            s_uros_state = AgentState::CONNECTED;
          } else {
            destroyEntities();
            digitalWrite(LED_GREEN, LOW);
            setDiagnostics("WARN", "micro-ROS init fehlgeschlagen, warte weiter");
          }
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
          stopDriveMotion();
          digitalWrite(LED_GREEN, LOW);
          setDiagnostics("WARN", "micro-ROS Agent verloren, Drive gestoppt");
          s_uros_state = AgentState::WAITING;
          break;
        }
      }

      RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
      break;
    }
  }

  delay(5);
}
