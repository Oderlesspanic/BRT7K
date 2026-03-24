# pragma once

#include <array>
#include <string>

struct MPU6050Config
{
    std::string i2c_bus;
    int i2c_address;

    double accel_scale;
    double gyro_scale;

    double accel_offset_x;
    double accel_offset_y;
    double accel_offset_z;

    double gyro_offset_x;
    double gyro_offset_y;
    double gyro_offset_z;

    std::string frame_id;
    double update_rate;

    std::array<double, 9> angular_velocity_covariance;
    std::array<double, 9> linear_acceleration_covariance;
    std::array<double, 9> orientation_covariance;
};
