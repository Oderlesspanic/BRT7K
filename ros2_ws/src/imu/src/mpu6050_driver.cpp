#include "imu/mpu6050_driver.hpp"

MPU6050Driver::MPU6050Driver(std::shared_ptr<II2CBus> bus)
    : bus_(bus) {}

std::optional<ImuRawData> MPU6050Driver::read_imu() {
    std::vector<uint8_t> data(2);

    if (!bus_->read_bytes(0x3B, data)) {
        return std::nullopt;
    }

    ImuRawData d{};
    d.accel_x = (data[0] << 8) | data[1];

    return d;
}