#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA260.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/battery_state.h>

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
rcl_publisher_t battery_pub;

sensor_msgs__msg__BatteryState battery_msg;

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

  // INA260 returns mA and mV — convert to A and V for BatteryState
  battery_msg.current = ina260.readCurrent()    / 1000.0f;  // mA → A
  battery_msg.voltage = ina260.readBusVoltage() / 1000.0f;  // mV → V

  RCSOFTCHECK(rcl_publish(&battery_pub, &battery_msg, NULL));
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

  sensor_msgs__msg__BatteryState__init(&battery_msg);
  battery_msg.temperature             = NAN;
  battery_msg.charge                  = NAN;
  battery_msg.capacity                = NAN;
  battery_msg.design_capacity         = NAN;
  battery_msg.percentage              = NAN;
  battery_msg.power_supply_status     = 0;  // UNKNOWN
  battery_msg.power_supply_health     = 0;  // UNKNOWN
  battery_msg.power_supply_technology = 0;  // UNKNOWN
  battery_msg.present                 = true;

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  rmw_uros_sync_session(1000);
  RCCHECK(rclc_node_init_default(&node, "ina260_test_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &battery_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
    "/battery_state"));

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
