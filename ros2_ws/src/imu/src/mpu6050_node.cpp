#include "imu/mpu6050_node.hpp"
#include "imu/imu_converter.hpp"

#include <algorithm>
#include <chrono>
#include <unistd.h>
#include <vector>

MPU6050Node::MPU6050Node()
: Node("mpu6050_node"), initialized_(false)
{
    declare_parameters();
    config_ = load_config();

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

    auto bus = std::make_shared<I2CBus>(config_.i2c_bus, config_.i2c_address);

    if (!bus->openBus())
    {
        RCLCPP_ERROR(this->get_logger(), "I2C-Bus konnte nicht geoeffnet werden");
        return;
    }

    driver_ = std::make_shared<MPU6050Driver>(bus, config_);

    if (!driver_->initialize())
    {
        RCLCPP_ERROR(this->get_logger(), "MPU6050 konnte nicht initialisiert werden");
        return;
    }

    if (config_.update_rate <= 0.0)
    {
        RCLCPP_ERROR(this->get_logger(), "update_rate muss groesser als 0 sein");
        return;
    }

    auto period = std::chrono::duration<double>(1.0 / config_.update_rate);

    bool calibrate = this->get_parameter("calibrate_on_start").as_bool();
    if (calibrate)
    {
        run_startup_calibration();
    }

    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&MPU6050Node::timer_callback, this)
    );

    initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "MPU6050 Node gestartet");
}

void MPU6050Node::declare_parameters()
{
    this->declare_parameter<std::string>("i2c_bus", "/dev/i2c-1");
    this->declare_parameter<int>("i2c_address", 104);
    this->declare_parameter<bool>("calibrate_on_start", false);
    this->declare_parameter<int>("calibration_samples", 200);

    this->declare_parameter<double>("accel_scale", 16384.0);
    this->declare_parameter<double>("gyro_scale", 131.0);

    this->declare_parameter<double>("accel_offset_x", 0.0);
    this->declare_parameter<double>("accel_offset_y", 0.0);
    this->declare_parameter<double>("accel_offset_z", 0.0);

    this->declare_parameter<double>("gyro_offset_x", 0.0);
    this->declare_parameter<double>("gyro_offset_y", 0.0);
    this->declare_parameter<double>("gyro_offset_z", 0.0);

    this->declare_parameter<std::string>("frame_id", "imu_link");
    this->declare_parameter<double>("update_rate", 100.0);

    this->declare_parameter<std::vector<double>>(
        "angular_velocity_covariance",
        {0.01, 0.0, 0.0,
         0.0, 0.01, 0.0,
         0.0, 0.0, 0.01});

    this->declare_parameter<std::vector<double>>(
        "linear_acceleration_covariance",
        {0.04, 0.0, 0.0,
         0.0, 0.04, 0.0,
         0.0, 0.0, 0.04});

    this->declare_parameter<std::vector<double>>(
        "orientation_covariance",
        {-1.0, 0.0, 0.0,
          0.0, 0.0, 0.0,
          0.0, 0.0, 0.0});
}

MPU6050Config MPU6050Node::load_config()
{
    MPU6050Config config;

    config.i2c_bus = this->get_parameter("i2c_bus").as_string();
    config.i2c_address = this->get_parameter("i2c_address").as_int();

    config.accel_scale = this->get_parameter("accel_scale").as_double();
    config.gyro_scale = this->get_parameter("gyro_scale").as_double();

    config.accel_offset_x = this->get_parameter("accel_offset_x").as_double();
    config.accel_offset_y = this->get_parameter("accel_offset_y").as_double();
    config.accel_offset_z = this->get_parameter("accel_offset_z").as_double();

    config.gyro_offset_x = this->get_parameter("gyro_offset_x").as_double();
    config.gyro_offset_y = this->get_parameter("gyro_offset_y").as_double();
    config.gyro_offset_z = this->get_parameter("gyro_offset_z").as_double();

    config.frame_id = this->get_parameter("frame_id").as_string();
    config.update_rate = this->get_parameter("update_rate").as_double();

    auto ang_cov = this->get_parameter("angular_velocity_covariance").as_double_array();
    auto lin_cov = this->get_parameter("linear_acceleration_covariance").as_double_array();
    auto ori_cov = this->get_parameter("orientation_covariance").as_double_array();

    if (ang_cov.size() == 9)
    {
        std::copy(ang_cov.begin(), ang_cov.end(), config.angular_velocity_covariance.begin());
    }

    if (lin_cov.size() == 9)
    {
        std::copy(lin_cov.begin(), lin_cov.end(), config.linear_acceleration_covariance.begin());
    }

    if (ori_cov.size() == 9)
    {
        std::copy(ori_cov.begin(), ori_cov.end(), config.orientation_covariance.begin());
    }

    return config;
}

void MPU6050Node::run_startup_calibration()
{
    int n = this->get_parameter("calibration_samples").as_int();
    if (n <= 0) { return; }

    RCLCPP_INFO(this->get_logger(),
        "Starte Kalibrierung mit %d Samples – Roboter muss stillstehen!", n);

    double sum_ax = 0, sum_ay = 0, sum_az = 0;
    double sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int collected = 0;

    for (int i = 0; i < n; ++i)
    {
        auto raw = driver_->read_imu();
        if (!raw.has_value())
        {
            RCLCPP_WARN(this->get_logger(), "Kalibrierung: Lesefehler bei Sample %d", i);
            usleep(10000);
            continue;
        }

        sum_ax += accel_raw_to_ms2(raw->accel_x, config_.accel_scale);
        sum_ay += accel_raw_to_ms2(raw->accel_y, config_.accel_scale);
        sum_az += accel_raw_to_ms2(raw->accel_z, config_.accel_scale);
        sum_gx += gyro_raw_to_rads(raw->gyro_x, config_.gyro_scale);
        sum_gy += gyro_raw_to_rads(raw->gyro_y, config_.gyro_scale);
        sum_gz += gyro_raw_to_rads(raw->gyro_z, config_.gyro_scale);
        ++collected;

        usleep(5000); // 5ms zwischen Samples → ~200 Hz
    }

    if (collected < n / 2)
    {
        RCLCPP_ERROR(this->get_logger(),
            "Kalibrierung fehlgeschlagen: zu wenige Samples (%d/%d)", collected, n);
        return;
    }

    constexpr double G = 9.80665;
    config_.accel_offset_x = sum_ax / collected;
    config_.accel_offset_y = sum_ay / collected;
    // Z-Achse: Gravitation abziehen (Sensor sollte ~9.81 m/s² messen)
    config_.accel_offset_z = sum_az / collected - G;
    config_.gyro_offset_x  = sum_gx / collected;
    config_.gyro_offset_y  = sum_gy / collected;
    config_.gyro_offset_z  = sum_gz / collected;

    RCLCPP_WARN(this->get_logger(),
        "Kalibrierung abgeschlossen (%d Samples). In mpu6050.yaml eintragen:\n"
        "  accel_offset_x: %.6f\n"
        "  accel_offset_y: %.6f\n"
        "  accel_offset_z: %.6f\n"
        "  gyro_offset_x:  %.6f\n"
        "  gyro_offset_y:  %.6f\n"
        "  gyro_offset_z:  %.6f",
        collected,
        config_.accel_offset_x, config_.accel_offset_y, config_.accel_offset_z,
        config_.gyro_offset_x,  config_.gyro_offset_y,  config_.gyro_offset_z);
}

void MPU6050Node::timer_callback()
{
    if (!initialized_)
    {
        return;
    }

    auto raw = driver_->read_imu();

    if (!raw.has_value())
    {
        RCLCPP_WARN(this->get_logger(), "Konnte MPU6050-Daten nicht lesen");
        return;
    }

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = config_.frame_id;

    msg.linear_acceleration.x =
        accel_raw_to_ms2(raw->accel_x, config_.accel_scale) - config_.accel_offset_x;
    msg.linear_acceleration.y =
        accel_raw_to_ms2(raw->accel_y, config_.accel_scale) - config_.accel_offset_y;
    msg.linear_acceleration.z =
        accel_raw_to_ms2(raw->accel_z, config_.accel_scale) - config_.accel_offset_z;

    msg.angular_velocity.x =
        gyro_raw_to_rads(raw->gyro_x, config_.gyro_scale) - config_.gyro_offset_x;
    msg.angular_velocity.y =
        gyro_raw_to_rads(raw->gyro_y, config_.gyro_scale) - config_.gyro_offset_y;
    msg.angular_velocity.z =
        gyro_raw_to_rads(raw->gyro_z, config_.gyro_scale) - config_.gyro_offset_z;

    msg.angular_velocity_covariance = config_.angular_velocity_covariance;
    msg.linear_acceleration_covariance = config_.linear_acceleration_covariance;
    msg.orientation_covariance = config_.orientation_covariance;

    imu_pub_->publish(msg);
}