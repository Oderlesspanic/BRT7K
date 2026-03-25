#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include "imu/i2c_bus.hpp"
#include "imu/mpu6050_config.hpp"

struct ImuRawData
{
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
    MPU6050Driver(std::shared_ptr<I2CBus> bus, const MPU6050Config& config);

    bool initialize();
    std::optional<ImuRawData> read_imu();

private:
    static constexpr uint8_t REG_PWR_MGMT_1    = 0x6B;
    static constexpr uint8_t REG_ACCEL_CONFIG  = 0x1C;
    static constexpr uint8_t REG_GYRO_CONFIG   = 0x1B;
    static constexpr uint8_t REG_ACCEL_XOUT_H  = 0x3B;

    std::shared_ptr<I2CBus> bus_;
    MPU6050Config config_;
};