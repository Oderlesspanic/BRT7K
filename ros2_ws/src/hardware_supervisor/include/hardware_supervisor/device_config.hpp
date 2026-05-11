#pragma once

#include <string>

struct DeviceConfig
{
  std::string name;
  std::string physical_type;       // "usb", "i2c", "none"
  std::string communication_type;  // "topic", "esp32_heartbeat"
  std::string path;
  std::string topic;
  std::string topic_type;
  double timeout = 1.0;

  int i2c_bus = -1;
  int i2c_address = -1;
};