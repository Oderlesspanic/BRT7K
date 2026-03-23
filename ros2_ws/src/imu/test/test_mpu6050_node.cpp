#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "imu/mpu6050_node.hpp"

TEST(MPU6050NodeTest, CanBeConstructed)
{
    if (!rclcpp::ok()) {
        int argc = 0;
        char ** argv = nullptr;
        rclcpp::init(argc, argv);
    }

    auto node = std::make_shared<imu::MPU6050Node>();
    ASSERT_NE(node, nullptr);
    EXPECT_EQ(node->get_name(), std::string("imu"));

    rclcpp::shutdown();
}