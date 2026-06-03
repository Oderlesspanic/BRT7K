#pragma once

#include <array>
#include <string>

struct MPU6050Config
{
    std::string i2c_bus{"/dev/i2c-1"};
    int i2c_address{104};

    double accel_scale{16384.0};
    double gyro_scale{131.0};

    double accel_offset_x{0.0};
    double accel_offset_y{0.0};
    double accel_offset_z{0.0};

    double gyro_offset_x{0.0};
    double gyro_offset_y{0.0};
    double gyro_offset_z{0.0};

    bool gyro_auto_calibration{true};
    bool accel_auto_calibration{false};
    int calibration_samples{100};

    std::string frame_id{"imu_link"};
    double update_rate{100.0};

    std::array<double, 9> angular_velocity_covariance{};
    std::array<double, 9> linear_acceleration_covariance{};
    std::array<double, 9> orientation_covariance{};
};
