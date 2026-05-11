#include "hardware_supervisor/composite_device_monitor.hpp"

#include <sstream>

CompositeDeviceMonitor::CompositeDeviceMonitor(const std::string & name)
: name_(name)
{
}

void CompositeDeviceMonitor::add_monitor(std::shared_ptr<DeviceMonitor> monitor)
{
  monitors_.push_back(monitor);
}

std::string CompositeDeviceMonitor::name() const
{
  return name_;
}

bool CompositeDeviceMonitor::is_present() const
{
  for (const auto & monitor : monitors_) {
    if (!monitor->is_present()) {
      return false;
    }
  }

  return true;
}

bool CompositeDeviceMonitor::is_alive() const
{
  for (const auto & monitor : monitors_) {
    if (!monitor->is_alive()) {
      return false;
    }
  }

  return true;
}

std::string CompositeDeviceMonitor::message() const
{
  std::ostringstream ss;

  for (std::size_t i = 0; i < monitors_.size(); ++i) {
    if (i > 0) {
      ss << " | ";
    }

    ss << monitors_[i]->message();
  }

  return ss.str();
}