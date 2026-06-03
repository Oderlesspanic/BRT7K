#include "camera/camera_node.hpp"

#include <chrono>
#include <utility>

using namespace std::chrono_literals;

CameraNode::CameraNode()
: Node("camera_node")
{
    camera_running_ = false;
    frame_id_ = this->declare_parameter<std::string>("frame_id", "camera_optical_link");
    topic_name_ = this->declare_parameter<std::string>("topic_name", "/camera/image_raw");
    camera_info_topic_ = this->declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
    width_ = this->declare_parameter<int>("width", 640);
    height_ = this->declare_parameter<int>("height", 480);
    fps_ = this->declare_parameter<int>("fps", 30);
    fx_ = this->declare_parameter<double>("fx", 0.0);
    fy_ = this->declare_parameter<double>("fy", 0.0);
    cx_ = this->declare_parameter<double>("cx", 0.0);
    cy_ = this->declare_parameter<double>("cy", 0.0);
    distortion_model_ = this->declare_parameter<std::string>("distortion_model", "plumb_bob");
    distortion_coefficients_ = this->declare_parameter<std::vector<double>>(
        "distortion_coefficients",
        std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});

    if (fx_ <= 0.0) {
        fx_ = static_cast<double>(width_);
    }
    if (fy_ <= 0.0) {
        fy_ = static_cast<double>(width_);
    }
    if (cx_ <= 0.0) {
        cx_ = static_cast<double>(width_) * 0.5;
    }
    if (cy_ <= 0.0) {
        cy_ = static_cast<double>(height_) * 0.5;
    }

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name_, 10);
    camera_info_pub_ =
        this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/camera/status_text", 10);

    driver_ = std::make_shared<LibcameraDriver>();
    publish_status("INFO", "initialisiere libcamera");

    if (!driver_->initialize(width_, height_, fps_)) {
        publish_status("ERROR", "LibcameraDriver initialize() failed: Kamera nicht gefunden oder libcamera konnte nicht initialisiert werden");
        RCLCPP_ERROR(this->get_logger(), "%s", status_text_.c_str());
        status_timer_ = this->create_wall_timer(1s, std::bind(&CameraNode::status_timer_callback, this));
        return;
    }

    width_ = driver_->width();
    height_ = driver_->height();

    if (!driver_->start()) {
        publish_status("ERROR", "LibcameraDriver start() failed: Kamera wurde gefunden, Stream konnte aber nicht starten");
        RCLCPP_ERROR(this->get_logger(), "%s", status_text_.c_str());
        status_timer_ = this->create_wall_timer(1s, std::bind(&CameraNode::status_timer_callback, this));
        return;
    }

    camera_running_ = true;
    auto period = std::chrono::milliseconds(1000 / fps_);
    timer_ = this->create_wall_timer(period, std::bind(&CameraNode::timer_callback, this));
    status_timer_ = this->create_wall_timer(1s, std::bind(&CameraNode::status_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "camera_node gestartet");
    publish_status("INFO", "camera_node gestartet");
}

CameraNode::~CameraNode()
{
    if (driver_) {
        driver_->stop();
    }
}

void CameraNode::timer_callback()
{
    if (!camera_running_) {
        return;
    }

    std::vector<uint8_t> frame;
    uint64_t timestamp_ns = 0;

    if (!driver_->capture_frame(frame, timestamp_ns)) {
        RCLCPP_WARN(this->get_logger(), "Kein Frame erhalten");
        publish_status("WARN", "Kein Frame erhalten");
        return;
    }

    sensor_msgs::msg::Image msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = frame_id_;
    msg.height = static_cast<uint32_t>(height_);
    msg.width = static_cast<uint32_t>(width_);
    msg.encoding = "bgr8";
    msg.is_bigendian = false;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(width_ * 3);
    msg.data = std::move(frame);

    image_pub_->publish(msg);

    sensor_msgs::msg::CameraInfo info_msg;
    info_msg.header = msg.header;
    info_msg.height = msg.height;
    info_msg.width = msg.width;
    info_msg.distortion_model = distortion_model_;
    info_msg.d = distortion_coefficients_;
    info_msg.k = {
        fx_, 0.0, cx_,
        0.0, fy_, cy_,
        0.0, 0.0, 1.0};
    info_msg.r = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0};
    info_msg.p = {
        fx_, 0.0, cx_, 0.0,
        0.0, fy_, cy_, 0.0,
        0.0, 0.0, 1.0, 0.0};
    camera_info_pub_->publish(info_msg);
}

void CameraNode::status_timer_callback()
{
    if (status_text_.empty() && camera_running_) {
        publish_status("INFO", "running");
        return;
    }

    std_msgs::msg::String msg;
    msg.data = status_text_;
    status_pub_->publish(msg);
}

void CameraNode::publish_status(const std::string & level, const std::string & message)
{
    status_text_ = level + " " + message;

    if (status_pub_) {
        std_msgs::msg::String msg;
        msg.data = status_text_;
        status_pub_->publish(msg);
    }
}
