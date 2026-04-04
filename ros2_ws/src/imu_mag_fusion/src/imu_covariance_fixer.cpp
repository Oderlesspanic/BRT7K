#include <array>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuCovarianceFixer : public rclcpp::Node
{
public:
    ImuCovarianceFixer() : Node("imu_covariance_fixer")
    {
        this->declare_parameter<std::string>("input_topic", "/imu/data");
        this->declare_parameter<std::string>("output_topic", "/imu/data_fixed");

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

        this->declare_parameter<bool>("overwrite_orientation_covariance", false);

        this->declare_parameter<std::vector<double>>(
            "orientation_covariance",
            {0.0025, 0.0, 0.0,
             0.0, 0.0025, 0.0,
             0.0, 0.0, 0.0025});

        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();

        auto ang = this->get_parameter("angular_velocity_covariance").as_double_array();
        auto lin = this->get_parameter("linear_acceleration_covariance").as_double_array();
        auto ori = this->get_parameter("orientation_covariance").as_double_array();

        overwrite_orientation_covariance_ =
            this->get_parameter("overwrite_orientation_covariance").as_bool();

        if (ang.size() != 9 || lin.size() != 9 || ori.size() != 9) {
            RCLCPP_FATAL(this->get_logger(), "All covariance arrays must have exactly 9 elements.");
            throw std::runtime_error("Invalid covariance size");
        }

        std::copy(ang.begin(), ang.end(), angular_velocity_covariance_.begin());
        std::copy(lin.begin(), lin.end(), linear_acceleration_covariance_.begin());
        std::copy(ori.begin(), ori.end(), orientation_covariance_.begin());

        pub_ = this->create_publisher<sensor_msgs::msg::Imu>(output_topic_, 10);
        sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            input_topic_,
            10,
            std::bind(&ImuCovarianceFixer::imuCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "imu_covariance_fixer started");
        RCLCPP_INFO(this->get_logger(), "Input topic:  %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic_.c_str());
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        sensor_msgs::msg::Imu out = *msg;

        out.angular_velocity_covariance = angular_velocity_covariance_;
        out.linear_acceleration_covariance = linear_acceleration_covariance_;

        if (overwrite_orientation_covariance_) {
            out.orientation_covariance = orientation_covariance_;
        }

        pub_->publish(out);
    }

    std::string input_topic_;
    std::string output_topic_;
    bool overwrite_orientation_covariance_;

    std::array<double, 9> angular_velocity_covariance_{};
    std::array<double, 9> linear_acceleration_covariance_{};
    std::array<double, 9> orientation_covariance_{};

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuCovarianceFixer>());
    rclcpp::shutdown();
    return 0;
}