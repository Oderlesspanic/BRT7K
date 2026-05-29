#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "interfaces/msg/managed_corner.hpp"
#include "interfaces/msg/managed_corner_array.hpp"
#include "interfaces/msg/object_place_command.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

class ObjectTaskExecutorNode : public rclcpp::Node
{
public:
  ObjectTaskExecutorNode();
  ~ObjectTaskExecutorNode() override;

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  void objectsCallback(
    const vision_msgs::msg::Detection3DArray::SharedPtr msg);

  void cornersCallback(
    const interfaces::msg::ManagedCornerArray::SharedPtr msg);

  void commandCallback(
    const interfaces::msg::ObjectPlaceCommand::SharedPtr msg);

  void executeCommand(
    interfaces::msg::ObjectPlaceCommand command);

  bool resolveCommand(
    const interfaces::msg::ObjectPlaceCommand & command,
    vision_msgs::msg::Detection3D & object,
    interfaces::msg::ManagedCorner & corner);

  bool navigateTo(
    const geometry_msgs::msg::Point & point,
    double offset_x,
    double offset_y,
    const std::string & label);

  geometry_msgs::msg::PoseStamped makeGoalPose(
    const geometry_msgs::msg::Point & point,
    double offset_x,
    double offset_y) const;

  void publishActuatorCommand(
    const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr & publisher,
    const std::string & command,
    const std::string & label);

  std::string getObjectName(
    const vision_msgs::msg::Detection3D & detection) const;

  void sleepForActuator();

  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr objects_sub_;
  rclcpp::Subscription<interfaces::msg::ManagedCornerArray>::SharedPtr corners_sub_;
  rclcpp::Subscription<interfaces::msg::ObjectPlaceCommand>::SharedPtr command_sub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr platform_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_client_;

  std::vector<vision_msgs::msg::Detection3D> objects_;
  std::vector<interfaces::msg::ManagedCorner> corners_;
  mutable std::mutex data_mutex_;

  std::thread worker_;
  std::atomic_bool task_running_{false};

  std::string map_frame_;
  std::string gripper_close_command_;
  std::string gripper_open_command_;
  std::string platform_up_command_;
  std::string platform_down_command_;

  double goal_yaw_;
  double pickup_offset_x_;
  double pickup_offset_y_;
  double dropoff_offset_x_;
  double dropoff_offset_y_;
  double actuator_settle_seconds_;
};
