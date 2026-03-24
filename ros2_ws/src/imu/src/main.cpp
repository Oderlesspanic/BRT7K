#include "rclcpp/rclcpp.hpp"
#include "mpu6050_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPU6050Node>());
    rclcpp::shutdown();
    return 0;
}