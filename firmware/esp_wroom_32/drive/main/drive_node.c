/*
 * drive_node.c
 *
 *  Created on: 14.05.2026
 *      Author: jan
 */


 /*
  * drive_node.c
  */

 #include "drive_node.h"

 #include <stdio.h>
 #include <stdlib.h>
 #include <stdbool.h>
 #include <math.h>

 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"

 #include "esp_log.h"
 #include "esp_timer.h"
 #include "driver/uart.h"
 #include "driver/gpio.h"

 #include <rcl/rcl.h>
 #include <rclc/rclc.h>
 #include <rclc/executor.h>
 #include <rmw_microros/rmw_microros.h>


 #include <std_msgs/msg/empty.h>
 #include <std_msgs/msg/string.h>
 #include <geometry_msgs/msg/twist.h>
 #include <sensor_msgs/msg/joint_state.h>
 #include <rosidl_runtime_c/string_functions.h>

 #include "motor.h"
 #include "urdf_parser.h"

 #define LEFT_UART   UART_NUM_1
 #define RIGHT_UART  UART_NUM_2

 #define LEFT_TX_PIN   GPIO_NUM_17
 #define LEFT_RX_PIN   GPIO_NUM_16

 #define RIGHT_TX_PIN  GPIO_NUM_25
 #define RIGHT_RX_PIN  GPIO_NUM_26

 #define CMD_TIMEOUT_MS       300
 #define JOINT_PUB_PERIOD_MS  20

 #define LEFT_INVERT          1
 #define RIGHT_INVERT        -1

 #define ROBOT_DESCRIPTION_BUFFER_SIZE 16000

 static const char *TAG = "DRIVE_NODE";

 static motor_t motor_left = {
     .uart = LEFT_UART,
     .tx_pin = LEFT_TX_PIN,
     .rx_pin = LEFT_RX_PIN,
     .id = 0x01
 };

 static motor_t motor_right = {
     .uart = RIGHT_UART,
     .tx_pin = RIGHT_TX_PIN,
     .rx_pin = RIGHT_RX_PIN,
     .id = 0x01
 };

 static wheel_geometry_t wheel_geometry = {
     .wheel_radius_m = 0.03656f,
     .wheel_diameter_m = 0.07312f,
     .wheel_separation_m = 0.32120f,
     .valid = false
 };

 static rcl_publisher_t joint_pub;
 static rcl_publisher_t heartbeat_pub;
 static rcl_subscription_t cmd_sub;
 static rcl_subscription_t robot_description_sub;

 static sensor_msgs__msg__JointState joint_msg;
 static geometry_msgs__msg__Twist cmd_msg;
 static std_msgs__msg__String robot_description_msg;

 static rcl_timer_t joint_timer;
 static rcl_timer_t watchdog_timer;
 static rcl_timer_t heartbeat_timer;
 static std_msgs__msg__Empty heartbeat_msg;

 static int64_t last_cmd_time_ms = 0;

 static void cmd_vel_callback(const void *msgin)
 {
     const geometry_msgs__msg__Twist *msg =
         (const geometry_msgs__msg__Twist *)msgin;

     float v = (float)msg->linear.x;
     float w = (float)msg->angular.z;

     float v_left =
         v - (w * wheel_geometry.wheel_separation_m / 2.0f);

     float v_right =
         v + (w * wheel_geometry.wheel_separation_m / 2.0f);

     int16_t rpm_left = (int16_t)(
         (v_left / (2.0f * (float)M_PI * wheel_geometry.wheel_radius_m)) * 60.0f
     );

     int16_t rpm_right = (int16_t)(
         (v_right / (2.0f * (float)M_PI * wheel_geometry.wheel_radius_m)) * 60.0f
     );

     rpm_left *= LEFT_INVERT;
     rpm_right *= RIGHT_INVERT;

     motor_set_rpm(&motor_left, rpm_left);
     motor_set_rpm(&motor_right, rpm_right);

     last_cmd_time_ms = esp_timer_get_time() / 1000;
 }

 static void robot_description_callback(const void *msgin)
 {
     const std_msgs__msg__String *msg =
         (const std_msgs__msg__String *)msgin;

     if (msg->data.data == NULL) {
         return;
     }

     wheel_geometry_t parsed;

     if (urdf_parse_wheel_geometry(msg->data.data, &parsed)) {
         wheel_geometry = parsed;

         ESP_LOGI(TAG, "URDF wheel_radius:     %.5f m", wheel_geometry.wheel_radius_m);
         ESP_LOGI(TAG, "URDF wheel_diameter:   %.5f m", wheel_geometry.wheel_diameter_m);
         ESP_LOGI(TAG, "URDF wheel_separation: %.5f m", wheel_geometry.wheel_separation_m);
     } else {
         ESP_LOGW(TAG, "URDF konnte nicht ausgewertet werden, nutze Default-Werte");
     }
 }

 static void joint_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
 {
     (void)last_call_time;

     if (timer == NULL) {
         return;
     }

     motor_feedback_t fb_l;
     motor_feedback_t fb_r;

     if (!motor_read_encoder(&motor_left, &fb_l)) {
         return;
     }

     if (!motor_read_encoder(&motor_right, &fb_r)) {
         return;
     }

     double left_pos = (double)(fb_l.position_rad * LEFT_INVERT);
     double right_pos = (double)(fb_r.position_rad * RIGHT_INVERT);

     int64_t now_us = esp_timer_get_time();

     joint_msg.header.stamp.sec = (int32_t)(now_us / 1000000);
     joint_msg.header.stamp.nanosec = (uint32_t)((now_us % 1000000) * 1000);

     joint_msg.position.data[0] = left_pos;
     joint_msg.position.data[1] = right_pos;

     rcl_publish(&joint_pub, &joint_msg, NULL);
 }

 static void watchdog_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
 {
     (void)last_call_time;

     if (timer == NULL) {
         return;
     }

     int64_t now_ms = esp_timer_get_time() / 1000;

     if ((now_ms - last_cmd_time_ms) > CMD_TIMEOUT_MS) {
         motor_stop(&motor_left);
         motor_stop(&motor_right);
     }
 }

 static void heartbeat_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
 {
     (void)last_call_time;

     if (timer == NULL) {
         return;
     }

     rcl_publish(&heartbeat_pub, &heartbeat_msg, NULL);
 }

 static void init_joint_state_message(void)
 {
     sensor_msgs__msg__JointState__init(&joint_msg);

     joint_msg.name.capacity = 2;
     joint_msg.name.size = 2;
     joint_msg.name.data =
         malloc(2 * sizeof(rosidl_runtime_c__String));

     rosidl_runtime_c__String__init(&joint_msg.name.data[0]);
     rosidl_runtime_c__String__init(&joint_msg.name.data[1]);

     rosidl_runtime_c__String__assign(
         &joint_msg.name.data[0],
         "left_wheel_link_joint"
     );

     rosidl_runtime_c__String__assign(
         &joint_msg.name.data[1],
         "right_wheel_link_joint"
     );

     joint_msg.position.capacity = 2;
     joint_msg.position.size = 2;
     joint_msg.position.data = malloc(2 * sizeof(double));

     joint_msg.velocity.capacity = 0;
     joint_msg.velocity.size = 0;
     joint_msg.velocity.data = NULL;

     joint_msg.effort.capacity = 0;
     joint_msg.effort.size = 0;
     joint_msg.effort.data = NULL;
 }

 static void init_robot_description_message(void)
 {
     std_msgs__msg__String__init(&robot_description_msg);

     robot_description_msg.data.data =
         malloc(ROBOT_DESCRIPTION_BUFFER_SIZE);

     robot_description_msg.data.capacity =
         ROBOT_DESCRIPTION_BUFFER_SIZE;

     robot_description_msg.data.size = 0;
 }

 static void micro_ros_task(void *arg)
 {
     (void)arg;

     //set_microros_transports();

     rcl_allocator_t allocator = rcl_get_default_allocator();

     rclc_support_t support;
     rcl_node_t node;
     rclc_executor_t executor;

     rclc_support_init(&support, 0, NULL, &allocator);

     rclc_node_init_default(
         &node,
         "drive_node",
         "",
         &support
     );

     rclc_publisher_init_default(
         &joint_pub,
         &node,
         ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
         "/joint_states"
     );

     rclc_publisher_init_default(
         &heartbeat_pub,
         &node,
         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
         "/esp32_drive/heartbeat"
     );

     rclc_subscription_init_default(
         &cmd_sub,
         &node,
         ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
         "/cmd_vel"
     );

     rcl_subscription_options_t robot_description_options =
         rcl_subscription_get_default_options();

     robot_description_options.qos.durability =
         RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

     robot_description_options.qos.reliability =
         RMW_QOS_POLICY_RELIABILITY_RELIABLE;

     robot_description_options.qos.history =
         RMW_QOS_POLICY_HISTORY_KEEP_LAST;

     robot_description_options.qos.depth = 1;

     rcl_subscription_init(
         &robot_description_sub,
         &node,
         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
         "/robot_description",
         &robot_description_options
     );

     rclc_timer_init_default(
         &joint_timer,
         &support,
         RCL_MS_TO_NS(JOINT_PUB_PERIOD_MS),
         joint_timer_callback
     );

     rclc_timer_init_default(
         &watchdog_timer,
         &support,
         RCL_MS_TO_NS(50),
         watchdog_timer_callback
     );

     rclc_timer_init_default(
         &heartbeat_timer,
         &support,
         RCL_MS_TO_NS(500),
         heartbeat_timer_callback
     );

     init_joint_state_message();
     init_robot_description_message();
     std_msgs__msg__Empty__init(&heartbeat_msg);
     geometry_msgs__msg__Twist__init(&cmd_msg);

     last_cmd_time_ms = esp_timer_get_time() / 1000;

     rclc_executor_init(&executor, &support.context, 5, &allocator);

     rclc_executor_add_subscription(
         &executor,
         &cmd_sub,
         &cmd_msg,
         &cmd_vel_callback,
         ON_NEW_DATA
     );

     rclc_executor_add_subscription(
         &executor,
         &robot_description_sub,
         &robot_description_msg,
         &robot_description_callback,
         ON_NEW_DATA
     );

     rclc_executor_add_timer(&executor, &joint_timer);
     rclc_executor_add_timer(&executor, &watchdog_timer);
     rclc_executor_add_timer(&executor, &heartbeat_timer);

     ESP_LOGI(TAG, "micro-ROS drive_node gestartet");

     while (true) {
         rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
         vTaskDelay(pdMS_TO_TICKS(10));
     }
 }

 void drive_node_start(void)
 {
     motor_init(&motor_left);
     motor_init(&motor_right);

     ESP_LOGI(TAG, "Motor UARTs initialisiert");

     vTaskDelay(pdMS_TO_TICKS(100));

     motor_set_speed_loop(&motor_left);
     motor_set_speed_loop(&motor_right);

     vTaskDelay(pdMS_TO_TICKS(100));

     motor_stop(&motor_left);
     motor_stop(&motor_right);

     xTaskCreate(
         micro_ros_task,
         "micro_ros_task",
         16000,
         NULL,
         5,
         NULL
     );
 }