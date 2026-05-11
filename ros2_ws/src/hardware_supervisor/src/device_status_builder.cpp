#include "hardware_supervisor/device_status_builder.hpp"

diagnostic_msgs::msg::DiagnosticStatus DeviceStatusBuilder::build(
  const DeviceMonitor & monitor)
{
  diagnostic_msgs::msg::DiagnosticStatus status;

  status.name = monitor.name();
  status.hardware_id = monitor.name();

  if (!monitor.is_present()) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "not present: " + monitor.message();
  } else if (!monitor.is_alive()) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    status.message = "not alive: " + monitor.message();
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = monitor.message();
  }

  return status;
}