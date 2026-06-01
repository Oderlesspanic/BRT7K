#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA260.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { errorLoop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

// Adjust these pins to match your wiring if needed.
static constexpr uint8_t INA260_SDA_PIN = 33;
static constexpr uint8_t INA260_SCL_PIN = 32;
static constexpr uint32_t PUBLISH_PERIOD_MS = 1000;

Adafruit_INA260 ina260;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t publish_timer;
rcl_publisher_t current_pub;
rcl_publisher_t voltage_pub;
rcl_publisher_t power_pub;

std_msgs__msg__Float32 current_msg;
std_msgs__msg__Float32 voltage_msg;
std_msgs__msg__Float32 power_msg;

static void errorLoop() {
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}

static void publishIna260(rcl_timer_t * timer, int64_t last_call_time) {
  (void)timer;
  (void)last_call_time;

  current_msg.data = ina260.readCurrent();
  voltage_msg.data = ina260.readBusVoltage();
  power_msg.data = ina260.readPower();

  RCSOFTCHECK(rcl_publish(&current_pub, &current_msg, NULL));
  RCSOFTCHECK(rcl_publish(&voltage_pub, &voltage_msg, NULL));
  RCSOFTCHECK(rcl_publish(&power_pub, &power_msg, NULL));
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  delay(200);

  set_microros_transports();

  Wire.begin(INA260_SDA_PIN, INA260_SCL_PIN);

  if (!ina260.begin()) {
    errorLoop();
  }

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  rmw_uros_sync_session(1000);
  RCCHECK(rclc_node_init_default(&node, "ina260_test_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &current_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/ina260/current_ma"));

  RCCHECK(rclc_publisher_init_default(
    &voltage_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/ina260/bus_voltage_mv"));

  RCCHECK(rclc_publisher_init_default(
    &power_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/ina260/power_mw"));

  RCCHECK(rclc_timer_init_default(
    &publish_timer,
    &support,
    RCL_MS_TO_NS(PUBLISH_PERIOD_MS),
    publishIna260));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &publish_timer));
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  static uint32_t last_blink = 0;
  static bool led_state = false;
  if (millis() - last_blink >= 1000) {
    last_blink = millis();
    led_state = !led_state;
    digitalWrite(LED_BUILTIN, led_state);
  }
  delay(10);
}
