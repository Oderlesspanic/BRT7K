#pragma once

#include "rclcpp/rclcpp.hpp"

namespace imu
{

class MPU6050Node : public rclcpp::Node
{
public:
    MPU6050Node();
    ~MPU6050Node();
};

} // namespace imu