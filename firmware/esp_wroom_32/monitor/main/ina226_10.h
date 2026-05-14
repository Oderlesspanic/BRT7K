/*
 * ina226_10.h
 *
 *  Created on: 14.05.2026
 *      Author: jan
 */

#ifndef MAIN_INA226_10_H_
#define MAIN_INA226_10_H_

#define INA226_I2C_ADDR 0x40

void ina226_10_init(void);

void ina226_10_read_bus_voltage(float *voltage_v);
void ina226_10_read_current(float *current_a);
void ina226_10_read_power(float *power_w);


#endif /* MAIN_INA226_10_H_ */
