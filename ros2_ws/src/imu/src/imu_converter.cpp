
#include <cmath>
#include "imu/imu_converter.hpp"

int16_t bytes_to_int16(uint8_t high, uint8_t low)
{
    return static_cast<int16_t>((high << 8) | low);
}

double accel_raw_to_ms2(int16_t raw, double accel_scale)
{
    constexpr double g = 9.80665;      
    return (static_cast<double>(raw) / accel_scale) * g;
}

double gyro_raw_to_rads(int16_t raw, double gyro_scale)
{
    constexpr double deg_to_rad = M_PI / 180.0;
    return (static_cast<double>(raw) / gyro_scale) * deg_to_rad;
}

double temp_raw_to_celsius(int16_t raw)
{
    return (static_cast<double>(raw) / 340.0) + 36.53;
}