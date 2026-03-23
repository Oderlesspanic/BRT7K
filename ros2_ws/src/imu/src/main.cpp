#include "rclcpp/rclcpp.hpp"
#include "imu/mpu6050_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<imu::MPU6050Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}