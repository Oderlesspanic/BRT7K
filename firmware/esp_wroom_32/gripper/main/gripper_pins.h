#ifndef GRIPPER_PINS_H
#define GRIPPER_PINS_H

#include "driver/gpio.h"

typedef struct {
    gpio_num_t in1_pin;
    gpio_num_t in2_pin;
    gpio_num_t pwm_pin;
    gpio_num_t encoder_a_pin;
    gpio_num_t encoder_b_pin;
    gpio_num_t home_sensor_pin;
    gpio_num_t hx711_dout_pin;
    gpio_num_t hx711_sck_pin;
} gripper_arm_pins_t;

typedef struct {
    gpio_num_t in1_pin;
    gpio_num_t in2_pin;
    gpio_num_t pwm_pin;
    gpio_num_t encoder_a_pin;
    gpio_num_t encoder_b_pin;
    gpio_num_t bottom_sensor_pin;
    gpio_num_t top_sensor_pin;
    gpio_num_t hx711_dout_pin;
    gpio_num_t hx711_sck_pin;
} gripper_lift_pins_t;

typedef struct {
    gpio_num_t standby_pin;
    gripper_arm_pins_t left_arm;
    gripper_arm_pins_t right_arm;
    gripper_lift_pins_t lift;
} gripper_pins_t;

/*
 * Pseudo pin assignment until the final wiring is known.
 *
 * Notes for the real ESP32-WROOM-32 pinout:
 * - GPIO34-GPIO39 are input-only, so they are good encoder inputs but cannot drive outputs.
 * - GPIO0, GPIO2, GPIO12 and GPIO15 are boot strapping pins; avoid them for active external
 *   circuits if the board must boot reliably without special pullups/pulldowns.
 * - GPIO1 and GPIO3 are UART0 TX/RX. Keep them free if micro-ROS uses USB serial over UART0.
 * - SparkFun HX711 boards use DOUT and SCK GPIO lines. They are not I2C devices.
 */
static const gripper_pins_t GRIPPER_PINS = {
    .standby_pin = GPIO_NUM_4,

    .left_arm = {
        .in1_pin = GPIO_NUM_16,
        .in2_pin = GPIO_NUM_17,
        .pwm_pin = GPIO_NUM_18,
        .encoder_a_pin = GPIO_NUM_34,
        .encoder_b_pin = GPIO_NUM_35,
        .home_sensor_pin = GPIO_NUM_32,
        .hx711_dout_pin = GPIO_NUM_25,
        .hx711_sck_pin = GPIO_NUM_26,
    },

    .right_arm = {
        .in1_pin = GPIO_NUM_19,
        .in2_pin = GPIO_NUM_21,
        .pwm_pin = GPIO_NUM_22,
        .encoder_a_pin = GPIO_NUM_36,
        .encoder_b_pin = GPIO_NUM_39,
        .home_sensor_pin = GPIO_NUM_33,
        .hx711_dout_pin = GPIO_NUM_27,
        .hx711_sck_pin = GPIO_NUM_14,
    },

    .lift = {
        .in1_pin = GPIO_NUM_23,
        .in2_pin = GPIO_NUM_5,
        .pwm_pin = GPIO_NUM_13,
        .encoder_a_pin = GPIO_NUM_12,
        .encoder_b_pin = GPIO_NUM_15,
        .bottom_sensor_pin = GPIO_NUM_2,
        .top_sensor_pin = GPIO_NUM_0,
        .hx711_dout_pin = GPIO_NUM_3,
        .hx711_sck_pin = GPIO_NUM_1,
    },
};

#endif
