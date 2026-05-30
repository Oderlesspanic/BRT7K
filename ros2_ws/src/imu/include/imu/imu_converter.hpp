#pragma once

#include <cstdint>

int16_t bytes_to_int16(uint8_t high_byte, uint8_t low_byte);
double accel_raw_to_ms2(int16_t raw, double accel_scale);
double gyro_raw_to_rads(int16_t raw, double gyro_scale);
double temp_raw_to_celsius(int16_t raw);
