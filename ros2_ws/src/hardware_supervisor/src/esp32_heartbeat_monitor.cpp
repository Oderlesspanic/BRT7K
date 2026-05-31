#include "hardware_supervisor/esp32_heartbeat_monitor.hpp"

#include <filesystem>
#include <sstream>

ESP32HeartbeatMonitor::ESP32HeartbeatMonitor(
    rclcpp::Node * node,
    const std::string & name,
    const std::string & device_path,
    const std::string & heartbeat_topic,
    double timeout
) : node_(node),
    name_(name),
    device_path_(device_path),
    heartbeat_topic_(heartbeat_topic),
    timeout_(timeout)
{
    heartbeat_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
        heartbeat_topic_,
        rclcpp::QoS(10),
        std::bind(&ESP32HeartbeatMonitor::heartbeat_callback, this, std::placeholders::_1)
    );
}

std::string ESP32HeartbeatMonitor::name() const
{
    return name_;
}

bool ESP32HeartbeatMonitor::is_present() const
{
    return std::filesystem::exists(device_path_) || heartbeat_received_;
}

bool ESP32HeartbeatMonitor::is_alive() const
{
    if(!is_present()) {
        return false;
    }

    if(!heartbeat_received_) {
        return false;
    }

    const double age = (node_->now() - last_heartbeat_).seconds();
    return age <= timeout_;
}

std::string ESP32HeartbeatMonitor::message() const
{
    const bool device_path_exists = std::filesystem::exists(device_path_);

    if(!device_path_exists && !heartbeat_received_) {
        return "ESP32 "+ name_ + " is missing at " + device_path_;
    }

    if(!heartbeat_received_) {
        return "ESP32 " + name_ + " heartbeat has never been received.";
    }

    const double age = (node_->now() - last_heartbeat_).seconds();
    if(age > timeout_) {
        std::ostringstream ss;
        ss << "ESP32 heartbeat timeout " 
        << heartbeat_topic_ 
        << ", age = " << age 
        << " seconds ago, which exceeds the timeout= " << timeout_ << " seconds.";
        return ss.str();
    }

    if(!device_path_exists) {
        return "Device heartbeat is recent, but " + device_path_ + " does not exist.";
    }

    return "Device is alive and heartbeat is recent.";
}

void ESP32HeartbeatMonitor::heartbeat_callback(const std_msgs::msg::Empty::SharedPtr)
{
    last_heartbeat_ = node_->now();
    heartbeat_received_ = true;
}
