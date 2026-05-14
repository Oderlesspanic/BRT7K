#include "gripper_node.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <control_msgs/msg/gripper_command.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/empty.h>

#include "gripper_hardware.h"
#include "gripper_pins.h"

#define NODE_NAME              "esp32_gripper"
#define COMMAND_TOPIC          "/esp32_gripper/command"
#define STATE_TOPIC            "/esp32_gripper/is_closed"
#define HEARTBEAT_TOPIC        "/esp32_gripper/heartbeat"

#define HEARTBEAT_PERIOD_MS    500
#define CONTROL_PERIOD_MS      20
#define TASK_STACK_SIZE        14000
#define TASK_PRIORITY          5

#define OPEN_GAP_M             ((float)CONFIG_GRIPPER_OPEN_GAP_MM / 1000.0f)
#define CLOSED_GAP_M           ((float)CONFIG_GRIPPER_CLOSED_GAP_MM / 1000.0f)

#define ARM_CLOSED_TICKS       2400
#define LIFT_TOP_TICKS         3200
#define POSITION_TOLERANCE     20

#define HOMING_SPEED           0.28f
#define ARM_MAX_SPEED          0.55f
#define LIFT_SPEED             0.45f
#define ARM_KP                 0.0015f
#define LOAD_CELL_SCALE_N      0.001f

typedef enum {
    GRIPPER_STATE_HOMING_ARMS = 0,
    GRIPPER_STATE_HOMING_LIFT,
    GRIPPER_STATE_READY,
    GRIPPER_STATE_LOWERING,
    GRIPPER_STATE_OPENING,
    GRIPPER_STATE_CLOSING,
    GRIPPER_STATE_LIFTING,
    GRIPPER_STATE_HOLDING,
    GRIPPER_STATE_ERROR
} gripper_state_t;

static const char *TAG = "GRIPPER_NODE";

static gripper_hw_t gripper_hw;
static gripper_state_t gripper_state = GRIPPER_STATE_HOMING_ARMS;

static rcl_publisher_t heartbeat_pub;
static rcl_publisher_t state_pub;
static rcl_subscription_t command_sub;
static rcl_timer_t heartbeat_timer;

static std_msgs__msg__Empty heartbeat_msg;
static std_msgs__msg__Bool state_msg;
static control_msgs__msg__GripperCommand command_msg;

static volatile bool command_pending = false;
static float target_gap_m = OPEN_GAP_M;
static float target_effort_n = 0.0f;
static int32_t target_arm_ticks = 0;

static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

static int32_t gap_to_arm_ticks(float gap_m)
{
    const float gap_range = OPEN_GAP_M - CLOSED_GAP_M;

    if (fabsf(gap_range) < 0.0001f) {
        return ARM_CLOSED_TICKS;
    }

    const float normalized_close =
        clamp_float((OPEN_GAP_M - gap_m) / gap_range, 0.0f, 1.0f);

    return (int32_t)(normalized_close * ARM_CLOSED_TICKS);
}

static float load_cell_force_n(gripper_axis_id_t axis_id)
{
    return (float)gripper_hw_read_load_cell_raw(&gripper_hw, axis_id) * LOAD_CELL_SCALE_N;
}

static float axis_position_speed(int32_t target_ticks, int32_t current_ticks)
{
    const int32_t error = target_ticks - current_ticks;

    if (abs(error) <= POSITION_TOLERANCE) {
        return 0.0f;
    }

    return clamp_float((float)error * ARM_KP, -ARM_MAX_SPEED, ARM_MAX_SPEED);
}

static bool arm_target_reached(void)
{
    const int32_t left_error =
        target_arm_ticks - gripper_hw_encoder_count(&gripper_hw, GRIPPER_AXIS_LEFT_ARM);
    const int32_t right_error =
        target_arm_ticks - gripper_hw_encoder_count(&gripper_hw, GRIPPER_AXIS_RIGHT_ARM);

    return abs(left_error) <= POSITION_TOLERANCE &&
        abs(right_error) <= POSITION_TOLERANCE;
}

static bool arm_open_command(void)
{
    return target_arm_ticks <= POSITION_TOLERANCE;
}

static bool effort_reached(void)
{
    if (target_effort_n <= 0.0f) {
        return false;
    }

    const float left_force = fabsf(load_cell_force_n(GRIPPER_AXIS_LEFT_ARM));
    const float right_force = fabsf(load_cell_force_n(GRIPPER_AXIS_RIGHT_ARM));

    return left_force >= target_effort_n || right_force >= target_effort_n;
}

static void move_arms_to_target(void)
{
    const int32_t left_ticks =
        gripper_hw_encoder_count(&gripper_hw, GRIPPER_AXIS_LEFT_ARM);
    const int32_t right_ticks =
        gripper_hw_encoder_count(&gripper_hw, GRIPPER_AXIS_RIGHT_ARM);

    gripper_hw_set_axis_speed(
        &gripper_hw,
        GRIPPER_AXIS_LEFT_ARM,
        axis_position_speed(target_arm_ticks, left_ticks)
    );

    gripper_hw_set_axis_speed(
        &gripper_hw,
        GRIPPER_AXIS_RIGHT_ARM,
        axis_position_speed(target_arm_ticks, right_ticks)
    );
}

static void publish_closed_state(bool closed)
{
    state_msg.data = closed;
    rcl_publish(&state_pub, &state_msg, NULL);
}

static void lower_step(void)
{
    const bool bottom_sensor =
        gripper_hw_min_sensor_active(&gripper_hw, GRIPPER_AXIS_LIFT);
    const int32_t lift_ticks =
        gripper_hw_encoder_count(&gripper_hw, GRIPPER_AXIS_LIFT);

    if (bottom_sensor || lift_ticks <= 0) {
        gripper_hw_stop_axis(&gripper_hw, GRIPPER_AXIS_LIFT);
        gripper_hw_reset_encoder(&gripper_hw, GRIPPER_AXIS_LIFT);
        gripper_state = arm_open_command() ? GRIPPER_STATE_OPENING : GRIPPER_STATE_CLOSING;
        return;
    }

    gripper_hw_set_axis_speed(&gripper_hw, GRIPPER_AXIS_LIFT, -LIFT_SPEED);
}

static void open_arms_step(void)
{
    move_arms_to_target();

    if (arm_target_reached()) {
        gripper_hw_stop_axis(&gripper_hw, GRIPPER_AXIS_LEFT_ARM);
        gripper_hw_stop_axis(&gripper_hw, GRIPPER_AXIS_RIGHT_ARM);
        publish_closed_state(false);
        gripper_state = GRIPPER_STATE_READY;
        ESP_LOGI(TAG, "Greifer geoeffnet");
    }
}

static void home_arms_step(void)
{
    const bool left_home =
        gripper_hw_min_sensor_active(&gripper_hw, GRIPPER_AXIS_LEFT_ARM);
    const bool right_home =
        gripper_hw_min_sensor_active(&gripper_hw, GRIPPER_AXIS_RIGHT_ARM);

    gripper_hw_set_axis_speed(
        &gripper_hw,
        GRIPPER_AXIS_LEFT_ARM,
        left_home ? 0.0f : -HOMING_SPEED
    );

    gripper_hw_set_axis_speed(
        &gripper_hw,
        GRIPPER_AXIS_RIGHT_ARM,
        right_home ? 0.0f : -HOMING_SPEED
    );

    if (left_home && right_home) {
        gripper_hw_stop_axis(&gripper_hw, GRIPPER_AXIS_LEFT_ARM);
        gripper_hw_stop_axis(&gripper_hw, GRIPPER_AXIS_RIGHT_ARM);
        gripper_hw_reset_encoder(&gripper_hw, GRIPPER_AXIS_LEFT_ARM);
        gripper_hw_reset_encoder(&gripper_hw, GRIPPER_AXIS_RIGHT_ARM);
        gripper_state = GRIPPER_STATE_HOMING_LIFT;
        ESP_LOGI(TAG, "Greiferarme referenziert");
    }
}

static void home_lift_step(void)
{
    if (gripper_hw_min_sensor_active(&gripper_hw, GRIPPER_AXIS_LIFT)) {
        gripper_hw_stop_axis(&gripper_hw, GRIPPER_AXIS_LIFT);
        gripper_hw_reset_encoder(&gripper_hw, GRIPPER_AXIS_LIFT);
        gripper_state = GRIPPER_STATE_READY;
        ESP_LOGI(TAG, "Hebeplattform referenziert");
        return;
    }

    gripper_hw_set_axis_speed(&gripper_hw, GRIPPER_AXIS_LIFT, -HOMING_SPEED);
}

static void close_arms_step(void)
{
    move_arms_to_target();

    if (arm_target_reached() || effort_reached()) {
        gripper_hw_stop_axis(&gripper_hw, GRIPPER_AXIS_LEFT_ARM);
        gripper_hw_stop_axis(&gripper_hw, GRIPPER_AXIS_RIGHT_ARM);
        publish_closed_state(true);
        gripper_state = GRIPPER_STATE_LIFTING;
        ESP_LOGI(TAG, "Objekt gegriffen, hebe Plattform");
    }
}

static void lift_step(void)
{
    const bool top_sensor =
        gripper_hw_max_sensor_active(&gripper_hw, GRIPPER_AXIS_LIFT);
    const int32_t lift_ticks =
        gripper_hw_encoder_count(&gripper_hw, GRIPPER_AXIS_LIFT);

    if (top_sensor || lift_ticks >= LIFT_TOP_TICKS) {
        gripper_hw_stop_axis(&gripper_hw, GRIPPER_AXIS_LIFT);
        gripper_state = GRIPPER_STATE_HOLDING;
        ESP_LOGI(TAG, "Hebeplattform oben");
        return;
    }

    gripper_hw_set_axis_speed(&gripper_hw, GRIPPER_AXIS_LIFT, LIFT_SPEED);
}

static void control_step(void)
{
    if (command_pending && gripper_state >= GRIPPER_STATE_READY) {
        command_pending = false;
        target_arm_ticks = gap_to_arm_ticks(target_gap_m);
        gripper_state = GRIPPER_STATE_LOWERING;
        publish_closed_state(false);
        ESP_LOGI(
            TAG,
            "Neuer Greifbefehl: gap=%.3f m effort=%.2f N target_ticks=%ld",
            target_gap_m,
            target_effort_n,
            (long)target_arm_ticks
        );
    }

    switch (gripper_state) {
        case GRIPPER_STATE_HOMING_ARMS:
            home_arms_step();
            break;

        case GRIPPER_STATE_HOMING_LIFT:
            home_lift_step();
            break;

        case GRIPPER_STATE_READY:
            gripper_hw_stop_all(&gripper_hw);
            break;

        case GRIPPER_STATE_LOWERING:
            lower_step();
            break;

        case GRIPPER_STATE_OPENING:
            open_arms_step();
            break;

        case GRIPPER_STATE_CLOSING:
            close_arms_step();
            break;

        case GRIPPER_STATE_LIFTING:
            lift_step();
            break;

        case GRIPPER_STATE_HOLDING:
            gripper_hw_stop_all(&gripper_hw);
            break;

        case GRIPPER_STATE_ERROR:
        default:
            gripper_hw_stop_all(&gripper_hw);
            break;
    }
}

static void control_task(void *arg)
{
    (void)arg;

    while (true) {
        control_step();
        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

static void command_callback(const void *msgin)
{
    const control_msgs__msg__GripperCommand *msg =
        (const control_msgs__msg__GripperCommand *)msgin;

    target_gap_m = clamp_float((float)msg->position, CLOSED_GAP_M, OPEN_GAP_M);
    target_effort_n = (float)msg->max_effort;
    command_pending = true;
}

static void heartbeat_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;

    if (timer == NULL) {
        return;
    }

    rcl_publish(&heartbeat_pub, &heartbeat_msg, NULL);
}

static void micro_ros_task(void *arg)
{
    (void)arg;

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rclc_executor_t executor;

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(
        &node,
        NODE_NAME,
        "",
        &support
    );

    rclc_publisher_init_default(
        &heartbeat_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
        HEARTBEAT_TOPIC
    );

    rclc_publisher_init_default(
        &state_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        STATE_TOPIC
    );

    rclc_subscription_init_default(
        &command_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, GripperCommand),
        COMMAND_TOPIC
    );

    rclc_timer_init_default(
        &heartbeat_timer,
        &support,
        RCL_MS_TO_NS(HEARTBEAT_PERIOD_MS),
        heartbeat_timer_callback
    );

    std_msgs__msg__Empty__init(&heartbeat_msg);
    std_msgs__msg__Bool__init(&state_msg);
    control_msgs__msg__GripperCommand__init(&command_msg);

    rclc_executor_init(&executor, &support.context, 2, &allocator);

    rclc_executor_add_subscription(
        &executor,
        &command_sub,
        &command_msg,
        &command_callback,
        ON_NEW_DATA
    );

    rclc_executor_add_timer(&executor, &heartbeat_timer);

    ESP_LOGI(TAG, "micro-ROS gripper node gestartet");

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void gripper_node_start(void)
{
    gripper_hw_init(&gripper_hw, &GRIPPER_PINS);
    gripper_hw_stop_all(&gripper_hw);

    xTaskCreate(
        control_task,
        "gripper_control_task",
        8192,
        NULL,
        TASK_PRIORITY,
        NULL
    );

    xTaskCreate(
        micro_ros_task,
        "gripper_uros_task",
        TASK_STACK_SIZE,
        NULL,
        TASK_PRIORITY,
        NULL
    );
}
