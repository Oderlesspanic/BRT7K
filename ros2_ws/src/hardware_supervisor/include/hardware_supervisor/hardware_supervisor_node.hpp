#pragma once

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "std_msgs/msg/string.hpp"

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
  std_msgs::msg::String build_system_status_msg(
    const diagnostic_msgs::msg::DiagnosticArray & diagnostics) const;

  std::string hardware_id_;
  double update_rate_hz_ = 2.0;

  std::vector<DeviceConfig> device_configs_;

  HardwareSupervisor supervisor_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
