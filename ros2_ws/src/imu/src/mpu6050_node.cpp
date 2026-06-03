#include "imu/mpu6050_node.hpp"
#include "imu/imu_converter.hpp"

#include <algorithm>
#include <chrono>
#include <thread>
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

    calibrate_startup_offsets();

    auto period = std::chrono::duration<double>(1.0 / config_.update_rate);

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

    this->declare_parameter<double>("accel_scale", 16384.0);
    this->declare_parameter<double>("gyro_scale", 131.0);

    this->declare_parameter<double>("accel_offset_x", 0.0);
    this->declare_parameter<double>("accel_offset_y", 0.0);
    this->declare_parameter<double>("accel_offset_z", 0.0);

    this->declare_parameter<double>("gyro_offset_x", 0.0);
    this->declare_parameter<double>("gyro_offset_y", 0.0);
    this->declare_parameter<double>("gyro_offset_z", 0.0);

    this->declare_parameter<bool>("gyro_auto_calibration", true);
    this->declare_parameter<bool>("accel_auto_calibration", false);
    this->declare_parameter<int>("calibration_samples", 100);

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

    config.gyro_auto_calibration = this->get_parameter("gyro_auto_calibration").as_bool();
    config.accel_auto_calibration = this->get_parameter("accel_auto_calibration").as_bool();
    config.calibration_samples = this->get_parameter("calibration_samples").as_int();

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

void MPU6050Node::calibrate_startup_offsets()
{
    if (!config_.gyro_auto_calibration && !config_.accel_auto_calibration)
    {
        return;
    }

    const int sample_count = std::max(10, config_.calibration_samples);
    double accel_x_sum = 0.0;
    double accel_y_sum = 0.0;
    double accel_z_sum = 0.0;
    double gyro_x_sum = 0.0;
    double gyro_y_sum = 0.0;
    double gyro_z_sum = 0.0;
    int valid_samples = 0;

    RCLCPP_INFO(
        this->get_logger(),
        "IMU Startkalibrierung: %d Samples, Roboter muss stillstehen",
        sample_count
    );

    for (int i = 0; i < sample_count; ++i)
    {
        auto raw = driver_->read_imu();
        if (!raw.has_value())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        accel_x_sum += accel_raw_to_ms2(raw->accel_x, config_.accel_scale);
        accel_y_sum += accel_raw_to_ms2(raw->accel_y, config_.accel_scale);
        accel_z_sum += accel_raw_to_ms2(raw->accel_z, config_.accel_scale);
        gyro_x_sum += gyro_raw_to_rads(raw->gyro_x, config_.gyro_scale);
        gyro_y_sum += gyro_raw_to_rads(raw->gyro_y, config_.gyro_scale);
        gyro_z_sum += gyro_raw_to_rads(raw->gyro_z, config_.gyro_scale);
        valid_samples++;

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (valid_samples < std::max(5, sample_count / 2))
    {
        RCLCPP_WARN(
            this->get_logger(),
            "IMU Startkalibrierung abgebrochen: nur %d/%d gueltige Samples",
            valid_samples,
            sample_count
        );
        return;
    }

    if (config_.gyro_auto_calibration)
    {
        config_.gyro_offset_x = gyro_x_sum / valid_samples;
        config_.gyro_offset_y = gyro_y_sum / valid_samples;
        config_.gyro_offset_z = gyro_z_sum / valid_samples;
    }

    if (config_.accel_auto_calibration)
    {
        constexpr double gravity = 9.80665;
        config_.accel_offset_x = accel_x_sum / valid_samples;
        config_.accel_offset_y = accel_y_sum / valid_samples;
        config_.accel_offset_z = (accel_z_sum / valid_samples) - gravity;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "IMU Offsets: gyro=(%.5f, %.5f, %.5f), accel=(%.4f, %.4f, %.4f)",
        config_.gyro_offset_x,
        config_.gyro_offset_y,
        config_.gyro_offset_z,
        config_.accel_offset_x,
        config_.accel_offset_y,
        config_.accel_offset_z
    );
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
