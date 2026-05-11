# pragma once 

#include <string>
#include <filesystem>

#include "hardware_supervisor/device_monitor.hpp"

class UsbMonitor : public DeviceMonitor
{
public:
    UsbMonitor(const std::string & name, const std::string & path);

    std::string name() const override;
    bool is_present() const override;
    bool is_alive() const override;
    std::string message() const override;

private:
    std::string name_;
    std::filesystem::path path_;
};