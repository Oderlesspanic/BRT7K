#include "imu/i2c_bus.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

I2CBus::I2CBus(const std::string& device_path, int device_address)
    : device_path_(device_path),
      device_address_(device_address),
      file_descriptor_(-1)
{
}

I2CBus::~I2CBus()
{
    closeBus();
}

bool I2CBus::openBus()
{
    if (file_descriptor_ >= 0)
        return true;

    file_descriptor_ = open(device_path_.c_str(), O_RDWR);
    if (file_descriptor_ < 0)
        return false;

    if (ioctl(file_descriptor_, I2C_SLAVE, device_address_) < 0)
    {
        close(file_descriptor_);
        file_descriptor_ = -1;
        return false;
    }

    return true;
}

void I2CBus::closeBus()
{
    if (file_descriptor_ >= 0)
    {
        close(file_descriptor_);
        file_descriptor_ = -1;
    }
}

bool I2CBus::writeByte(uint8_t reg, uint8_t value)
{
    if (file_descriptor_ < 0)
        return false;

    uint8_t buffer[2] = {reg, value};

    return (write(file_descriptor_, buffer, 2) == 2);
}

bool I2CBus::readByte(uint8_t reg, uint8_t& value)
{
    return readBytes(reg, &value, 1);
}

bool I2CBus::readBytes(uint8_t start_reg, uint8_t* buffer, std::size_t length)
{
    if (file_descriptor_ < 0)
        return false;

    if (write(file_descriptor_, &start_reg, 1) != 1)
        return false;

    return (read(file_descriptor_, buffer, length) == (int)length);
}

bool I2CBus::isOpen() const
{
    return file_descriptor_ >= 0;
}