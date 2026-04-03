#include "mmc5603/i2c_bus.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdio>

I2CBus::I2CBus(const std::string& device, int address)
: device_(device), address_(address), fd_(-1) {}

I2CBus::~I2CBus() {
    close_bus();
}

bool I2CBus::open_bus() {
    fd_ = open(device_.c_str(), O_RDWR);
    if (fd_ < 0) {
        perror("I2C open failed");
        return false;
    }

    if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
        perror("I2C ioctl failed");
        return false;
    }

    return true;
}

void I2CBus::close_bus() {
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}

bool I2CBus::write_byte(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return write(fd_, buf, 2) == 2;
}

bool I2CBus::read_bytes(uint8_t reg, uint8_t* buffer, size_t length) {
    if (write(fd_, &reg, 1) != 1) return false;
    return read(fd_, buffer, length) == (int)length;
}