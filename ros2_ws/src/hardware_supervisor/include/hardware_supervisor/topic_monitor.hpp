#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "hardware_supervisor/device_monitor.hpp"

class TopicMonitor : public DeviceMonitor
{
public:
  TopicMonitor(
    rclcpp::Node * node,
    const std::string & name,
    const std::string & topic,
    const std::string & topic_type,
    double timeout);

  std::string name() const override;
  bool is_present() const override;
  bool is_alive() const override;
  std::string message() const override;

private:
  rclcpp::Node * node_;

  std::string name_;
  std::string topic_;
  std::string topic_type_;
  double timeout_;

  rclcpp::Time last_msg_time_;
  bool msg_received_ = false;

  rclcpp::GenericSubscription::SharedPtr subscription_;
};