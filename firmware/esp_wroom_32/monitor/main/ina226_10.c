/*
 * ina226_10.c
 *
 *  Created on: 14.05.2026
 *      Author: jan
 */


 #include "ina226_10.h"

 #include "driver/i2c.h"
 #include "freertos/FreeRTOS.h"

 #include <stdint.h>

 #define I2C_PORT              I2C_NUM_0
 #define I2C_TIMEOUT_MS        100

 #define INA226_I2C_ADDR       0x40

 #define INA226_REG_CONFIG     0x00
 #define INA226_REG_BUS_V      0x02
 #define INA226_REG_POWER      0x03
 #define INA226_REG_CURRENT    0x04
 #define INA226_REG_CALIB      0x05

 #define INA226_CONFIG_VALUE   0x4127

 #define SHUNT_RESISTOR_OHM    0.01f
 #define CURRENT_LSB_A         0.0005f

 static void ina226_write_register(uint8_t reg, uint16_t value)
 {
     uint8_t data[3];

     data[0] = reg;
     data[1] = (uint8_t)(value >> 8);
     data[2] = (uint8_t)(value & 0xFF);

     i2c_master_write_to_device(
         I2C_PORT,
         INA226_I2C_ADDR,
         data,
         3,
         pdMS_TO_TICKS(I2C_TIMEOUT_MS)
     );
 }

 static int16_t ina226_read_register(uint8_t reg)
 {
     uint8_t data[2] = {0, 0};

     i2c_master_write_read_device(
         I2C_PORT,
         INA226_I2C_ADDR,
         &reg,
         1,
         data,
         2,
         pdMS_TO_TICKS(I2C_TIMEOUT_MS)
     );

     return (int16_t)((data[0] << 8) | data[1]);
 }

 void ina226_10_init(void)
 {
     ina226_write_register(INA226_REG_CONFIG, INA226_CONFIG_VALUE);

     float calibration_f = 0.00512f / (CURRENT_LSB_A * SHUNT_RESISTOR_OHM);
     uint16_t calibration = (uint16_t)calibration_f;

     ina226_write_register(INA226_REG_CALIB, calibration);
 }

 void ina226_10_read_bus_voltage(float *voltage_v)
 {
     int16_t raw = ina226_read_register(INA226_REG_BUS_V);
     *voltage_v = raw * 0.00125f;
 }

 void ina226_10_read_current(float *current_a)
 {
     int16_t raw = ina226_read_register(INA226_REG_CURRENT);
     *current_a = raw * CURRENT_LSB_A;
 }

 void ina226_10_read_power(float *power_w)
 {
     int16_t raw = ina226_read_register(INA226_REG_POWER);
     *power_w = raw * CURRENT_LSB_A * 25.0f;
 }