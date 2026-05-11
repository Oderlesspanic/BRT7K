#pragma once 

#include <string>

#include "hardware_supervisor/device_monitor.hpp"

class I2CMonitor : public DeviceMonitor
{
public:
    I2CMonitor(
        const std::string & name,
        int bus,
        int address
    );

    std::string name() const override;
    bool is_present() const override;
    bool is_alive() const override;
    std::string message() const override;

private:
    bool check_i2c_device() const;

    std::string name_;
    int bus_;
    int address_;
};
    
    