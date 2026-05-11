#pragma once

#include <string>

class DeviceMonitor
{
public:
    virtual ~DeviceMonitor() = default;

    virtual std::string name() const = 0;
    virtual bool is_present() const = 0;
    virtual bool is_alive() const = 0;
    virtual std::string message() const = 0;
};