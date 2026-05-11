#pragma once

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include "hardware_supervisor/device_config.hpp"
#include "hardware_supervisor/hardware_supervisor.hpp"

class HardwareSupervisorNode : public rclcpp::Node
{
public:
  HardwareSupervisorNode();

private:
  void load_config();
  void create_monitors();
  void publish_status();

  std::string hardware_id_;
  double update_rate_hz_ = 2.0;

  std::vector<DeviceConfig> device_configs_;

  HardwareSupervisor supervisor_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};