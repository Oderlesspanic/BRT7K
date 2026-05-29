#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
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
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string frame_id_;
    std::string topic_name_;
    std::string camera_info_topic_;
    std::string distortion_model_;
    std::vector<double> distortion_coefficients_;
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    int width_;
    int height_;
    int fps_;
};
