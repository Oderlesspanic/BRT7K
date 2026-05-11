#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

#include "hardware_supervisor/device_monitor.hpp"

class ESP32HeartbeatMonitor : public DeviceMonitor
{
    public:
        ESP32HeartbeatMonitor(
            rclcpp::Node *node,
            const std::string & name,
            const std::string & device_path,
            const std::string & heartbeat_topic,
            double timeout
        );
    
        std::string name() const override;
        bool is_present() const override;
        bool is_alive() const override;
        std::string message() const override;
    private:
        void heartbeat_callback(const std_msgs::msg::Empty::SharedPtr msg);

        rclcpp::Node *node_;
        std::string name_;
        std::string device_path_;
        std::string heartbeat_topic_;
        double timeout_;
        rclcpp::Time last_heartbeat_;
        bool heartbeat_received_ = false;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;

    };
