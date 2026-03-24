#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "mpu6050_config.hpp"
#include "mpu6050_driver.hpp"

class MPU6050Node : public rclcpp::Node
{
public:
    MPU6050Node();

private:
    void declare_parameters();
    MPU6050Config load_config();
    void timer_callback();

    MPU6050Config config_;
    std::shared_ptr<MPU6050Driver> driver_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};