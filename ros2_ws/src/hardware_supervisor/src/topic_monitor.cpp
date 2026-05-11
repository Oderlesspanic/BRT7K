#include "hardware_supervisor/topic_monitor.hpp"

#include <sstream>

TopicMonitor::TopicMonitor(
  rclcpp::Node * node,
  const std::string & name,
  const std::string & topic,
  const std::string & topic_type,
  double timeout)
: node_(node),
  name_(name),
  topic_(topic),
  topic_type_(topic_type),
  timeout_(timeout),
  last_msg_time_(node->now())
{
  auto callback =
    [this](std::shared_ptr<rclcpp::SerializedMessage>) {
      last_msg_time_ = node_->now();
      msg_received_ = true;
    };

  subscription_ = node_->create_generic_subscription(
    topic_,
    topic_type_,
    rclcpp::QoS(10),
    callback);
}

std::string TopicMonitor::name() const
{
  return name_;
}

bool TopicMonitor::is_present() const
{
  return msg_received_;
}

bool TopicMonitor::is_alive() const
{
  if (!msg_received_) {
    return false;
  }

  const double age = (node_->now() - last_msg_time_).seconds();
  return age <= timeout_;
}

std::string TopicMonitor::message() const
{
  if (!msg_received_) {
    return "topic never received: " + topic_;
  }

  const double age = (node_->now() - last_msg_time_).seconds();

  if (age > timeout_) {
    std::ostringstream ss;
    ss << "topic timeout: " << topic_
       << ", age=" << age
       << "s, timeout=" << timeout_ << "s";
    return ss.str();
  }

  return "topic OK: " + topic_;
}