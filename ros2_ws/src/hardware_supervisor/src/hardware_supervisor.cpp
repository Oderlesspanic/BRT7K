#include "hardware_supervisor/hardware_supervisor.hpp"

#include "hardware_supervisor/device_status_builder.hpp"

void HardwareSupervisor::add_monitor(
  std::shared_ptr<DeviceMonitor> monitor)
{
  monitors_.push_back(monitor);
}

diagnostic_msgs::msg::DiagnosticArray
HardwareSupervisor::build_status_msg(
  const rclcpp::Time & stamp,
  const std::string & hardware_id) const
{
  diagnostic_msgs::msg::DiagnosticArray msg;

  msg.header.stamp = stamp;

  for (const auto & monitor : monitors_) {

    auto status =
      DeviceStatusBuilder::build(*monitor);

    status.hardware_id =
      hardware_id + "/" + monitor->name();

    msg.status.push_back(status);
  }

  return msg;
}