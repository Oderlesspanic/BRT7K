#include "odometry/wheel_odometry_node.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "tf2/LinearMath/Quaternion.h"

WheelOdometryNode::WheelOdometryNode()
: Node("wheel_odometry_node"),
  first_message_(true),
  last_left_position_(0.0),
  last_right_position_(0.0),
  x_(0.0),
  y_(0.0),
  theta_(0.0)
{
    left_wheel_joint_ = this->declare_parameter<std::string>("left_wheel_joint", "left_wheel_joint");
    right_wheel_joint_ = this->declare_parameter<std::string>("right_wheel_joint", "right_wheel_joint");

    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");

    joint_state_topic_ = this->declare_parameter<std::string>("joint_state_topic", "/joint_states");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/wheel/odometry");

    wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.05);
    wheel_separation_ = this->declare_parameter<double>("wheel_separation", 0.30);

    publish_tf_ = this->declare_parameter<bool>("publish_tf", true);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        joint_state_topic_,
        50,
        std::bind(&WheelOdometryNode::jointStateCallback, this, std::placeholders::_1));

    if (publish_tf_) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    RCLCPP_INFO(this->get_logger(), "wheel_odometry_node gestartet");
    RCLCPP_INFO(this->get_logger(), "Left joint:  %s", left_wheel_joint_.c_str());
    RCLCPP_INFO(this->get_logger(), "Right joint: %s", right_wheel_joint_.c_str());
}

bool WheelOdometryNode::getJointIndex(
    const sensor_msgs::msg::JointState::SharedPtr msg,
    const std::string & joint_name,
    size_t & index) const
{
    auto it = std::find(msg->name.begin(), msg->name.end(), joint_name);

    if (it == msg->name.end()) {
        return false;
    }

    index = static_cast<size_t>(std::distance(msg->name.begin(), it));
    return true;
}

void WheelOdometryNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    size_t left_index = 0;
    size_t right_index = 0;

    if (!getJointIndex(msg, left_wheel_joint_, left_index) ||
        !getJointIndex(msg, right_wheel_joint_, right_index))
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            3000,
            "Wheel joints nicht in /joint_states gefunden");
        return;
    }

    if (msg->position.size() <= left_index || msg->position.size() <= right_index) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            3000,
            "JointState positions unvollstaendig");
        return;
    }

    rclcpp::Time current_stamp = msg->header.stamp;
    if (current_stamp.nanoseconds() == 0) {
        current_stamp = this->now();
    }

    const double left_position = msg->position[left_index];
    const double right_position = msg->position[right_index];

    if (first_message_) {
        last_left_position_ = left_position;
        last_right_position_ = right_position;
        last_stamp_ = current_stamp;
        first_message_ = false;
        return;
    }

    const double dt = (current_stamp - last_stamp_).seconds();
    if (dt <= 0.0) {
        return;
    }

    const double delta_left = left_position - last_left_position_;
    const double delta_right = right_position - last_right_position_;

    const double distance_left = wheel_radius_ * delta_left;
    const double distance_right = wheel_radius_ * delta_right;

    const double distance_center = 0.5 * (distance_left + distance_right);
    const double delta_theta = (distance_right - distance_left) / wheel_separation_;

    const double theta_mid = theta_ + delta_theta * 0.5;

    x_ += distance_center * std::cos(theta_mid);
    y_ += distance_center * std::sin(theta_mid);
    theta_ += delta_theta;

    const double linear_velocity = distance_center / dt;
    const double angular_velocity = delta_theta / dt;

    publishOdometry(current_stamp, linear_velocity, angular_velocity);

    if (publish_tf_) {
        publishTf(current_stamp);
    }

    last_left_position_ = left_position;
    last_right_position_ = right_position;
    last_stamp_ = current_stamp;
}

void WheelOdometryNode::publishOdometry(
    const rclcpp::Time & stamp,
    double linear_velocity,
    double angular_velocity)
{
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);

    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = linear_velocity;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_velocity;

    odom_msg.pose.covariance[0] = 0.01;
    odom_msg.pose.covariance[7] = 0.01;
    odom_msg.pose.covariance[14] = 99999.0;
    odom_msg.pose.covariance[21] = 99999.0;
    odom_msg.pose.covariance[28] = 99999.0;
    odom_msg.pose.covariance[35] = 0.03;

    odom_msg.twist.covariance[0] = 0.02;
    odom_msg.twist.covariance[7] = 99999.0;
    odom_msg.twist.covariance[14] = 99999.0;
    odom_msg.twist.covariance[21] = 99999.0;
    odom_msg.twist.covariance[28] = 99999.0;
    odom_msg.twist.covariance[35] = 0.04;

    odom_pub_->publish(odom_msg);
}

void WheelOdometryNode::publishTf(const rclcpp::Time & stamp)
{
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);

    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);
}
