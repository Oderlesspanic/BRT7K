#include "monitor_node.h"
#include "ina226_10.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/battery_state.h>
#include <std_msgs/msg/empty.h>

#include <stdio.h>
#include <math.h>

#define NODE_NAME              "esp32_monitor"
#define BATTERY_TOPIC          "/esp32_monitor/battery_state"
#define HEARTBEAT_TOPIC        "/esp32_monitor/heartbeat"

#define PUBLISH_PERIOD_MS      1000
#define TASK_STACK_SIZE        8192
#define TASK_PRIORITY          5

static rcl_publisher_t battery_publisher;
static rcl_publisher_t heartbeat_publisher;

static sensor_msgs__msg__BatteryState battery_msg;
static std_msgs__msg__Empty heartbeat_msg;

static void monitor_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(
        &node,
        NODE_NAME,
        "",
        &support
    );

    rclc_publisher_init_default(
        &battery_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
        BATTERY_TOPIC
    );

    rclc_publisher_init_default(
        &heartbeat_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
        HEARTBEAT_TOPIC
    );

    ina226_10_init();

    battery_msg.power_supply_status =
        sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_UNKNOWN;

    battery_msg.power_supply_health =
        sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNKNOWN;

    battery_msg.power_supply_technology =
        sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_UNKNOWN;

    battery_msg.present = true;

    while (1) {
        float voltage = 0.0f;
        float current = 0.0f;
        float power = 0.0f;

        ina226_10_read_bus_voltage(&voltage);
        ina226_10_read_current(&current);
        ina226_10_read_power(&power);

        battery_msg.voltage = voltage;
        battery_msg.current = current;
        battery_msg.charge = nanf("");
        battery_msg.capacity = nanf("");
        battery_msg.design_capacity = nanf("");
        battery_msg.percentage = nanf("");

        rcl_publish(&battery_publisher, &battery_msg, NULL);
        rcl_publish(&heartbeat_publisher, &heartbeat_msg, NULL);

        vTaskDelay(pdMS_TO_TICKS(PUBLISH_PERIOD_MS));
    }
}

void monitor_node_start(void)
{
    xTaskCreate(
        monitor_task,
        "monitor_task",
        TASK_STACK_SIZE,
        NULL,
        TASK_PRIORITY,
        NULL
    );
}