#include "imu/imu_converter.hpp"

int16_t bytes_to_int16(uint8_t high_byte, uint8_t low_byte)
{
    return static_cast<int16_t>((static_cast<uint16_t>(high_byte) << 8) | low_byte);
}

double accel_raw_to_ms2(int16_t raw, double accel_scale)
{
    constexpr double g = 9.80665;
    return (static_cast<double>(raw) / accel_scale) * g;
}

double gyro_raw_to_rads(int16_t raw, double gyro_scale)
{
    constexpr double pi = 3.14159265358979323846;
    constexpr double deg_to_rad = pi / 180.0;
    return (static_cast<double>(raw) / gyro_scale) * deg_to_rad;
}

double temp_raw_to_celsius(int16_t raw)
{
    return (static_cast<double>(raw) / 340.0) + 36.53;
}
