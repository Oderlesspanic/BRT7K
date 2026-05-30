#pragma once
#include <array>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "mmc5603/i2c_bus.hpp"
#include "mmc5603/mmc5603_driver.hpp"

class MMC5603Node : public rclcpp::Node {
public:
    MMC5603Node();

private:
    void timer_callback();
    bool load_parameters();

    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<I2CBus> bus_;
    std::shared_ptr<MMC5603Driver> driver_;

    std::string i2c_bus_device_;
    int device_address_;
    std::string frame_id_;
    double publish_rate_;

    double offset_x_;
    double offset_y_;
    double offset_z_;

    double scale_x_;
    double scale_y_;
    double scale_z_;

    std::array<double, 9> covariance_;
};
