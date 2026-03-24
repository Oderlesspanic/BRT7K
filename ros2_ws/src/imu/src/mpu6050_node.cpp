#include "mpu6050_node.hpp"

#include <chrono>
#include <stdexcept>

using namespace std::chrono_literals;

MPU6050Node::MPU6050Node()
: Node("mpu6050_node")
{
    config_ = load_config();

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

    driver_ = std::make_shared<MPU6050Driver>(config_);

    if (!driver_->initialize()) {
        RCLCPP_ERROR(this->get_logger(), "MPU6050 konnte nicht initialisiert werden");
        throw std::runtime_error("MPU6050 init failed");
    }

    auto period = std::chrono::duration<double>(1.0 / config_.update_rate);

    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&MPU6050Node::timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "MPU6050 Node gestartet");
}

#include <vector>
#include <array>
#include <stdexcept>
#include <algorithm>

MPU6050Config MPU6050Node::load_config()
{
    MPU6050Config config;

    config.i2c_bus = this->get_parameter("i2c_bus").as_string();
    config.i2c_address = this->get_parameter("i2c_address").as_int();
    config.accel_scale = this->get_parameter("accel_scale").as_double();
    config.gyro_scale = this->get_parameter("gyro_scale").as_double();
    config.frame_id = this->get_parameter("frame_id").as_string();
    config.update_rate = this->get_parameter("update_rate").as_double();

    config.accel_offset_x = this->get_parameter("accel_offset_x").as_double();
    config.accel_offset_y = this->get_parameter("accel_offset_y").as_double();
    config.accel_offset_z = this->get_parameter("accel_offset_z").as_double();

    config.gyro_offset_x = this->get_parameter("gyro_offset_x").as_double();
    config.gyro_offset_y = this->get_parameter("gyro_offset_y").as_double();
    config.gyro_offset_z = this->get_parameter("gyro_offset_z").as_double();

    auto ang_cov = this->get_parameter("angular_velocity_covariance").as_double_array();
    auto lin_cov = this->get_parameter("linear_acceleration_covariance").as_double_array();
    auto ori_cov = this->get_parameter("orientation_covariance").as_double_array();

    if (ang_cov.size() != 9 || lin_cov.size() != 9 || ori_cov.size() != 9) {
        throw std::runtime_error("Covariance arrays muessen genau 9 Werte haben");
    }

    std::copy(ang_cov.begin(), ang_cov.end(), config.angular_velocity_covariance.begin());
    std::copy(lin_cov.begin(), lin_cov.end(), config.linear_acceleration_covariance.begin());
    std::copy(ori_cov.begin(), ori_cov.end(), config.orientation_covariance.begin());

    return config;
}