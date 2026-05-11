#pragma once

#include <memory>
#include <vector>

#include "rclcpp/time.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include "hardware_supervisor/device_monitor.hpp"

class HardwareSupervisor
{
public:
    void add_monitor(std::shared_ptr<DeviceMonitor> monitor);

    diagnostic_msgs::msg::DiagnosticArray build_status_msg(
        const rclcpp::Time & stamp,
        const std::string & hardware_id
    ) const;

private:
    std::vector<std::shared_ptr<DeviceMonitor>> monitors_;
};
