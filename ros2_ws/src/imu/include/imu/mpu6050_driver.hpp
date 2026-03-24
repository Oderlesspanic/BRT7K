#pragma once

#include <memory>
#include <optional>
#include <vector>
#include <cstdint>

#include "imu/i2c_bus.hpp"


struct ImuRawData {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
};

class MPU6050Driver {
public:
    explicit MPU6050Driver(std::shared_ptr<II2CBus> bus);

    std::optional<ImuRawData> read_imu();

private:
    std::shared_ptr<II2CBus> bus_;
};