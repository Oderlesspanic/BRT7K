#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

class WheelOdometryNode : public rclcpp::Node
{
public:
    WheelOdometryNode();

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    bool getJointIndex(
        const sensor_msgs::msg::JointState::SharedPtr msg,
        const std::string & joint_name,
        size_t & index) const;

    void publishOdometry(const rclcpp::Time & stamp, double linear_velocity, double angular_velocity);
    void publishTf(const rclcpp::Time & stamp);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::string left_wheel_joint_;
    std::string right_wheel_joint_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string joint_state_topic_;
    std::string odom_topic_;

    double wheel_radius_;
    double wheel_separation_;
    bool publish_tf_;

    bool first_message_;
    double last_left_position_;
    double last_right_position_;
    rclcpp::Time last_stamp_;

    double x_;
    double y_;
    double theta_;
};