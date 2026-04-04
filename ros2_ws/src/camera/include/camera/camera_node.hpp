#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "camera/libcamera_driver.hpp"

class CameraNode : public rclcpp::Node
{
public:
    CameraNode();
    ~CameraNode() override;

private:
    void timer_callback();

    std::shared_ptr<LibcameraDriver> driver_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string frame_id_;
    std::string topic_name_;
    int width_;
    int height_;
    int fps_;
};