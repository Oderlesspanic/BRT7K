#include "imu/mpu6050_node.hpp"

namespace imu
{

MPU6050Node::MPU6050Node()
: Node("imu")
{
    RCLCPP_INFO(this->get_logger(), "MPU6050 node gestartet");
}

MPU6050Node::~MPU6050Node()
{
}

} // namespace imu