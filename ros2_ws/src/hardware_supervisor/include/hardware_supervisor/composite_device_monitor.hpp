#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_supervisor/device_monitor.hpp"

class CompositeDeviceMonitor : public DeviceMonitor
{
public:
  explicit CompositeDeviceMonitor(const std::string & name);

  void add_monitor(std::shared_ptr<DeviceMonitor> monitor);

  std::string name() const override;
  bool is_present() const override;
  bool is_alive() const override;
  std::string message() const override;

private:
  std::string name_;
  std::vector<std::shared_ptr<DeviceMonitor>> monitors_;
};