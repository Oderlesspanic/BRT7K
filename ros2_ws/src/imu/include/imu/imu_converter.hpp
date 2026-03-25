#pragma once

#include <cstdint>

double accel_raw_to_ms2(int16_t raw, double accel_scale);
double gyro_raw_to_rads(int16_t raw, double gyro_scale);
double temp_raw_to_celsius(int16_t raw);