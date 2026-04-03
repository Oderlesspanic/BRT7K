#include "mmc5603/mmc5603_node.hpp"
#include "mmc5603/mmc5603_convert.hpp"
#include <chrono>

using namespace std::chrono_literals;

MMC5603Node::MMC5603Node() : Node("mmc5603_node") {

    this->declare_parameter("i2c_bus", "/dev/i2c-1");
    this->declare_parameter("device_address", 48);
    this->declare_parameter("frame_id", "mag_link");
    this->declare_parameter("publish_rate", 50.0);

    std::string i2c_bus = this->get_parameter("i2c_bus").as_string();
    int address = this->get_parameter("device_address").as_int();
    frame_id_ = this->get_parameter("frame_id").as_string();
    double rate = this->get_parameter("publish_rate").as_double();

    bus_ = std::make_shared<I2CBus>(i2c_bus, address);

    if (!bus_->open_bus()) {
        RCLCPP_FATAL(this->get_logger(), "I2C konnte nicht geöffnet werden");
        return;
    }

    driver_ = std::make_shared<MMC5603Driver>(bus_);
    driver_->initialize();

    mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("mag", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000.0 / rate)),
        std::bind(&MMC5603Node::timer_callback, this));
}

void MMC5603Node::timer_callback() {

    MMC5603RawData raw{};

    if (!driver_->trigger_measurement()) return;
    if (!driver_->read_raw_data(raw)) return;


    sensor_msgs::msg::MagneticField msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;

    auto mag = MMC5603Convert::raw_to_tesla(
    raw,
    0.0, 0.0, 0.0,   // offsets
    1.0, 1.0, 1.0    // scaling
    );

    msg.magnetic_field.x = mag.x_tesla;
    msg.magnetic_field.y = mag.y_tesla;
    msg.magnetic_field.z = mag.z_tesla;

    mag_pub_->publish(msg);
}