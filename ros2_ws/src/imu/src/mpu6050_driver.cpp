#include "imu/mpu6050_driver.hpp"
#include <unistd.h>

MPU6050Driver::MPU6050Driver(std::shared_ptr<I2CBus> bus, const MPU6050Config& config)
    : bus_(bus), config_(config)
{
}

bool MPU6050Driver::initialize()
{
    if (!bus_ || !bus_->isOpen())
    {
        return false;
    }

    // Sensor aufwecken
    if (!bus_->writeByte(REG_PWR_MGMT_1, 0x00))
    {
        return false;
    }

    // 50ms warten bis Sensor stabil – MPU6050 braucht nach Power-On Zeit
    usleep(50000);

    // Beschleunigung: ±2g
    if (!bus_->writeByte(REG_ACCEL_CONFIG, 0x00))
    {
        return false;
    }

    // Gyroskop: ±250 °/s
    if (!bus_->writeByte(REG_GYRO_CONFIG, 0x00))
    {
        return false;
    }

    return true;
}

std::optional<ImuRawData> MPU6050Driver::read_imu()
{
    if (!bus_ || !bus_->isOpen())
    {
        return std::nullopt;
    }

    uint8_t data[14];

    if (!bus_->readBytes(REG_ACCEL_XOUT_H, data, 14))
    {
        return std::nullopt;
    }

    ImuRawData imu_data{};

    imu_data.accel_x = static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
    imu_data.accel_y = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | data[3]);
    imu_data.accel_z = static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) | data[5]);

    imu_data.gyro_x = static_cast<int16_t>((static_cast<uint16_t>(data[8]) << 8) | data[9]);
    imu_data.gyro_y = static_cast<int16_t>((static_cast<uint16_t>(data[10]) << 8) | data[11]);
    imu_data.gyro_z = static_cast<int16_t>((static_cast<uint16_t>(data[12]) << 8) | data[13]);

    return imu_data;
}