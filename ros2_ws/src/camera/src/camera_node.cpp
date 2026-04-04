#include "camera/camera_node.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

using namespace std::chrono_literals;

CameraNode::CameraNode()
: Node("camera_node")
{
    frame_id_ = this->declare_parameter<std::string>("frame_id", "camera_optical_frame");
    topic_name_ = this->declare_parameter<std::string>("topic_name", "/camera/image_raw");
    width_ = this->declare_parameter<int>("width", 640);
    height_ = this->declare_parameter<int>("height", 480);
    fps_ = this->declare_parameter<int>("fps", 30);

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name_, 10);

    driver_ = std::make_shared<LibcameraDriver>();

    if (!driver_->initialize(width_, height_, fps_)) {
        throw std::runtime_error("LibcameraDriver initialize() failed");
    }

    if (!driver_->start()) {
        throw std::runtime_error("LibcameraDriver start() failed");
    }

    auto period = std::chrono::milliseconds(1000 / fps_);
    timer_ = this->create_wall_timer(period, std::bind(&CameraNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "camera_node gestartet");
}

CameraNode::~CameraNode()
{
    if (driver_) {
        driver_->stop();
    }
}

void CameraNode::timer_callback()
{
    std::vector<uint8_t> frame;
    uint64_t timestamp_ns = 0;

    if (!driver_->capture_frame(frame, timestamp_ns)) {
        RCLCPP_WARN(this->get_logger(), "Kein Frame erhalten");
        return;
    }

    sensor_msgs::msg::Image msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = frame_id_;
    msg.height = static_cast<uint32_t>(height_);
    msg.width = static_cast<uint32_t>(width_);
    msg.encoding = "rgb8";
    msg.is_bigendian = false;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(width_ * 3);
    msg.data = std::move(frame);

    image_pub_->publish(msg);
}