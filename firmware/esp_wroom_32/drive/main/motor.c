/*
 * motor.c
 *
 *  Created on: 14.05.2026
 *      Author: jan
 */


 #include "motor.h"

 #include <math.h>
 #include <stddef.h>

 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "driver/uart.h"

 static uint8_t crc8_maxim(const uint8_t *data, size_t len)
 {
     uint8_t crc = 0x00;

     for (size_t i = 0; i < len; i++) {
         uint8_t inbyte = data[i];

         for (uint8_t j = 0; j < 8; j++) {
             uint8_t mix = (crc ^ inbyte) & 0x01;
             crc >>= 1;

             if (mix) {
                 crc ^= 0x8C;
             }

             inbyte >>= 1;
         }
     }

     return crc;
 }

 static void motor_finalize_packet(uint8_t packet[10])
 {
     packet[9] = crc8_maxim(packet, 9);
 }

 static void motor_send_raw(const motor_t *motor, const uint8_t packet[10])
 {
     uart_write_bytes(motor->uart, packet, 10);
 }

 void motor_init(const motor_t *motor)
 {
     uart_config_t uart_config = {
         .baud_rate = 115200,
         .data_bits = UART_DATA_8_BITS,
         .parity = UART_PARITY_DISABLE,
         .stop_bits = UART_STOP_BITS_1,
         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
         .source_clk = UART_SCLK_DEFAULT,
     };

     uart_driver_install(motor->uart, 1024, 1024, 0, NULL, 0);
     uart_param_config(motor->uart, &uart_config);

     uart_set_pin(
         motor->uart,
         motor->tx_pin,
         motor->rx_pin,
         UART_PIN_NO_CHANGE,
         UART_PIN_NO_CHANGE
     );
 }

 void motor_set_speed_loop(const motor_t *motor)
 {
     uint8_t packet[10] = {
         motor->id, 0xA0, 0x02, 0x00, 0x00,
         0x00, 0x00, 0x00, 0x00, 0x00
     };

     motor_finalize_packet(packet);
     motor_send_raw(motor, packet);
 }

 void motor_stop(const motor_t *motor)
 {
     uint8_t packet[10] = {
         motor->id, 0x64, 0x00, 0x00, 0x00,
         0x00, 0x00, 0x00, 0x00, 0x50
     };

     motor_send_raw(motor, packet);
 }

 void motor_set_rpm(const motor_t *motor, int16_t rpm)
 {
     if (rpm > MOTOR_MAX_RPM) rpm = MOTOR_MAX_RPM;
     if (rpm < -MOTOR_MAX_RPM) rpm = -MOTOR_MAX_RPM;

     int16_t value = rpm * 10;

     uint8_t packet[10] = {
         motor->id,
         0x64,
         (uint8_t)((value >> 8) & 0xFF),
         (uint8_t)(value & 0xFF),
         0x00,
         0x00,
         0x00,
         0x00,
         0x00,
         0x00
     };

     motor_finalize_packet(packet);
     motor_send_raw(motor, packet);
 }

 bool motor_read_encoder(const motor_t *motor, motor_feedback_t *fb)
 {
     uint8_t tx[10] = {
         motor->id, 0x74, 0x00, 0x00, 0x00,
         0x00, 0x00, 0x00, 0x00, 0x00
     };

     motor_finalize_packet(tx);

     uart_flush_input(motor->uart);
     uart_write_bytes(motor->uart, tx, 10);

     uint8_t rx[10];

     int len = uart_read_bytes(
         motor->uart,
         rx,
         10,
         pdMS_TO_TICKS(20)
     );

     if (len != 10) {
         fb->valid = false;
         return false;
     }

     if (crc8_maxim(rx, 9) != rx[9]) {
         fb->valid = false;
         return false;
     }

     if (rx[0] != motor->id || rx[1] != 0x74) {
         fb->valid = false;
         return false;
     }

     fb->laps =
         ((int32_t)rx[2] << 24) |
         ((int32_t)rx[3] << 16) |
         ((int32_t)rx[4] << 8)  |
         ((int32_t)rx[5]);

     fb->position_raw =
         ((uint16_t)rx[6] << 8) |
         ((uint16_t)rx[7]);

     fb->fault = rx[8];
     fb->position_rad = motor_position_rad(fb);
     fb->valid = true;

     return true;
 }

 float motor_position_rad(const motor_feedback_t *fb)
 {
     float revolutions =
         (float)fb->laps + ((float)fb->position_raw / 32767.0f);

     return revolutions * 2.0f * (float)M_PI;
 }