#include "hardware_supervisor/i2c_monitor.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <sstream>
#include <iomanip>


I2CMonitor::I2CMonitor(
    const std::string & name,
    int bus,
    int address)
:   name_(name), 
    bus_(bus), 
    address_(address)
{
}

std::string I2CMonitor::name() const
{
    return name_;
}

bool I2CMonitor::is_present() const
{
    return check_i2c_device();
}

bool I2CMonitor::is_alive() const
{
    return check_i2c_device();
}

std::string I2CMonitor::message() const
{
    std::ostringstream oss;
    oss << "I2C device at bus " << bus_ 
        << ", address 0x" 
        << std::hex << std::uppercase << address_;
    
    if (check_i2c_device()) {
        return oss.str() + " is present and alive.";
    }

    return oss.str() + " is not responding.";
}

bool I2CMonitor::check_i2c_device() const
{
    std::string device_path = "/dev/i2c-" + std::to_string(bus_);

    int fd = open(device_path.c_str(), O_RDWR);
    if (fd < 0) {
        return false;
    }

    if (ioctl(fd, I2C_SLAVE, address_) < 0) {
        close(fd);
        return false;
    }

    // Try to read a byte to check if the device is responsive
    unsigned char buf;
    bool success = read(fd, &buf, 1) >= 0;

    close(fd);
    return success;
}
