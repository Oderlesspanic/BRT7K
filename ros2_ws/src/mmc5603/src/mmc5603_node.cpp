#include "mmc5603/mmc5603_node.hpp"
#include "mmc5603/mmc5603_convert.hpp"
#include <algorithm>
#include <chrono>
#include <vector>

using namespace std::chrono_literals;

MMC5603Node::MMC5603Node() : Node("mmc5603_node") {

    if (!load_parameters()) {
        RCLCPP_FATAL(this->get_logger(), "MMC5603 Parameter sind ungueltig");
        return;
    }

    bus_ = std::make_shared<I2CBus>(i2c_bus_device_, device_address_);

    if (!bus_->open_bus()) {
        RCLCPP_FATAL(this->get_logger(), "I2C konnte nicht geöffnet werden");
        return;
    }

    driver_ = std::make_shared<MMC5603Driver>(bus_);

    if (!driver_->initialize()) {
        RCLCPP_FATAL(this->get_logger(), "MMC5603 konnte nicht initialisiert werden");
        return;
    }

    mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("mag/data_raw", 10);

    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / publish_rate_)),
        std::bind(&MMC5603Node::timer_callback, this));
}

bool MMC5603Node::load_parameters() {
    this->declare_parameter<std::string>("i2c_bus", "/dev/i2c-1");
    this->declare_parameter<int>("device_address", 48);
    this->declare_parameter<std::string>("frame_id", "mag_link");
    this->declare_parameter<double>("publish_rate", 50.0);

    this->declare_parameter<double>("offset_x", 0.0);
    this->declare_parameter<double>("offset_y", 0.0);
    this->declare_parameter<double>("offset_z", 0.0);

    this->declare_parameter<double>("scale_x", 1.0);
    this->declare_parameter<double>("scale_y", 1.0);
    this->declare_parameter<double>("scale_z", 1.0);

    this->declare_parameter<std::vector<double>>(
        "magnetic_field_covariance",
        {1.0e-8, 0.0, 0.0,
         0.0, 1.0e-8, 0.0,
         0.0, 0.0, 1.0e-8});

    i2c_bus_device_ = this->get_parameter("i2c_bus").as_string();
    device_address_ = this->get_parameter("device_address").as_int();
    frame_id_ = this->get_parameter("frame_id").as_string();
    publish_rate_ = this->get_parameter("publish_rate").as_double();

    offset_x_ = this->get_parameter("offset_x").as_double();
    offset_y_ = this->get_parameter("offset_y").as_double();
    offset_z_ = this->get_parameter("offset_z").as_double();

    scale_x_ = this->get_parameter("scale_x").as_double();
    scale_y_ = this->get_parameter("scale_y").as_double();
    scale_z_ = this->get_parameter("scale_z").as_double();

    auto covariance = this->get_parameter("magnetic_field_covariance").as_double_array();

    if (publish_rate_ <= 0.0 || covariance.size() != covariance_.size()) {
        return false;
    }

    std::copy(covariance.begin(), covariance.end(), covariance_.begin());
    return true;
}

void MMC5603Node::timer_callback() {

    MMC5603RawData raw{};

    if (!driver_->trigger_measurement()) return;
    if (!driver_->wait_for_measurement(10ms)) return;
    if (!driver_->read_raw_data(raw)) return;


    sensor_msgs::msg::MagneticField msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;

    auto mag = MMC5603Convert::raw_to_tesla(
        raw,
        offset_x_, offset_y_, offset_z_,
        scale_x_, scale_y_, scale_z_
    );

    msg.magnetic_field.x = mag.x_tesla;
    msg.magnetic_field.y = mag.y_tesla;
    msg.magnetic_field.z = mag.z_tesla;
    msg.magnetic_field_covariance = covariance_;

    mag_pub_->publish(msg);
}
