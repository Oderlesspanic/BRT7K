#pragma once

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "hardware_supervisor/device_monitor.hpp"

class DeviceStatusBuilder
{
public:
  static diagnostic_msgs::msg::DiagnosticStatus build(
    const DeviceMonitor & monitor);
};