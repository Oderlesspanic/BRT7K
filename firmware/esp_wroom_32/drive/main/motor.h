/*
 * motor.h
 *
 *  Created on: 14.05.2026
 *      Author: jan
 */

#ifndef MAIN_MOTOR_H_
#define MAIN_MOTOR_H_


#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"
#include "driver/gpio.h"

#define MOTOR_MAX_RPM 210

typedef struct {
    uart_port_t uart;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    uint8_t id;
} motor_t;

typedef struct {
    int32_t laps;
    uint16_t position_raw;
    float position_rad;
    uint8_t fault;
    bool valid;
} motor_feedback_t;

void motor_init(const motor_t *motor);
void motor_set_speed_loop(const motor_t *motor);
void motor_stop(const motor_t *motor);
void motor_set_rpm(const motor_t *motor, int16_t rpm);
bool motor_read_encoder(const motor_t *motor, motor_feedback_t *fb);
float motor_position_rad(const motor_feedback_t *fb);



#endif /* MAIN_MOTOR_H_ */
