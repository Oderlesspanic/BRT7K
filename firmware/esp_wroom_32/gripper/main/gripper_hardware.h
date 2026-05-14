#ifndef GRIPPER_HARDWARE_H
#define GRIPPER_HARDWARE_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/ledc.h"
#include "driver/gpio.h"

#include "gripper_pins.h"

typedef enum {
    GRIPPER_AXIS_LEFT_ARM = 0,
    GRIPPER_AXIS_RIGHT_ARM,
    GRIPPER_AXIS_LIFT,
    GRIPPER_AXIS_COUNT
} gripper_axis_id_t;

typedef struct {
    gpio_num_t in1_pin;
    gpio_num_t in2_pin;
    gpio_num_t pwm_pin;
    gpio_num_t encoder_a_pin;
    gpio_num_t encoder_b_pin;
    gpio_num_t min_sensor_pin;
    gpio_num_t max_sensor_pin;
    gpio_num_t hx711_dout_pin;
    gpio_num_t hx711_sck_pin;
    ledc_channel_t pwm_channel;
    volatile int32_t encoder_count;
    volatile uint8_t last_encoder_state;
} gripper_axis_t;

typedef struct {
    gpio_num_t standby_pin;
    gripper_axis_t axes[GRIPPER_AXIS_COUNT];
} gripper_hw_t;

void gripper_hw_init(gripper_hw_t *hw, const gripper_pins_t *pins);
void gripper_hw_enable(gripper_hw_t *hw, bool enabled);
void gripper_hw_set_axis_speed(gripper_hw_t *hw, gripper_axis_id_t axis_id, float speed);
void gripper_hw_stop_axis(gripper_hw_t *hw, gripper_axis_id_t axis_id);
void gripper_hw_stop_all(gripper_hw_t *hw);
bool gripper_hw_min_sensor_active(const gripper_hw_t *hw, gripper_axis_id_t axis_id);
bool gripper_hw_max_sensor_active(const gripper_hw_t *hw, gripper_axis_id_t axis_id);
int32_t gripper_hw_encoder_count(const gripper_hw_t *hw, gripper_axis_id_t axis_id);
void gripper_hw_reset_encoder(gripper_hw_t *hw, gripper_axis_id_t axis_id);
int32_t gripper_hw_read_load_cell_raw(const gripper_hw_t *hw, gripper_axis_id_t axis_id);

#endif
