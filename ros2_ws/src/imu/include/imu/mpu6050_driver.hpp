#pragma once

#include <memory>
#include <optional>
#include <vector>
#include <cstdint>

#include "imu/i2c_bus.hpp"
#include "imu/mpu6050_config.hpp"

struct ImuRawData {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
};


class MPU6050Driver
{
public:
    explicit MPU6050Driver(const MPU6050Config& config);

    bool initialize();
    bool read_raw_data(int16_t& ax, int16_t& ay, int16_t& az,
                       int16_t& gx, int16_t& gy, int16_t& gz);

private:
    MPU6050Config config_;
    int fd_;
};