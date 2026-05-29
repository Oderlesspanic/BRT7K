#include "object_task_executor/object_task_executor_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <future>

using namespace std::chrono_literals;

ObjectTaskExecutorNode::ObjectTaskExecutorNode()
: Node("object_task_executor_node")
{
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<std::string>("objects_topic", "/objects");
  this->declare_parameter<std::string>("corners_topic", "/corners");
  this->declare_parameter<std::string>("command_topic", "/object_place_command");
  this->declare_parameter<std::string>("gripper_command_topic", "/gripper/command");
  this->declare_parameter<std::string>("platform_command_topic", "/platform/command");
  this->declare_parameter<std::string>("status_topic", "/object_task_executor/status");
  this->declare_parameter<std::string>("gripper_close_command", "close");
  this->declare_parameter<std::string>("gripper_open_command", "open");
  this->declare_parameter<std::string>("platform_up_command", "up");
  this->declare_parameter<std::string>("platform_down_command", "down");
  this->declare_parameter<double>("goal_yaw", 0.0);
  this->declare_parameter<double>("pickup_offset_x", 0.0);
  this->declare_parameter<double>("pickup_offset_y", 0.0);
  this->declare_parameter<double>("dropoff_offset_x", 0.0);
  this->declare_parameter<double>("dropoff_offset_y", 0.0);
  this->declare_parameter<double>("actuator_settle_seconds", 1.0);

  map_frame_ = this->get_parameter("map_frame").as_string();
  gripper_close_command_ = this->get_parameter("gripper_close_command").as_string();
  gripper_open_command_ = this->get_parameter("gripper_open_command").as_string();
  platform_up_command_ = this->get_parameter("platform_up_command").as_string();
  platform_down_command_ = this->get_parameter("platform_down_command").as_string();
  goal_yaw_ = this->get_parameter("goal_yaw").as_double();
  pickup_offset_x_ = this->get_parameter("pickup_offset_x").as_double();
  pickup_offset_y_ = this->get_parameter("pickup_offset_y").as_double();
  dropoff_offset_x_ = this->get_parameter("dropoff_offset_x").as_double();
  dropoff_offset_y_ = this->get_parameter("dropoff_offset_y").as_double();
  actuator_settle_seconds_ = this->get_parameter("actuator_settle_seconds").as_double();

  objects_sub_ =
    this->create_subscription<vision_msgs::msg::Detection3DArray>(
      this->get_parameter("objects_topic").as_string(),
      10,
      std::bind(&ObjectTaskExecutorNode::objectsCallback, this, std::placeholders::_1));

  corners_sub_ =
    this->create_subscription<interfaces::msg::ManagedCornerArray>(
      this->get_parameter("corners_topic").as_string(),
      10,
      std::bind(&ObjectTaskExecutorNode::cornersCallback, this, std::placeholders::_1));

  command_sub_ =
    this->create_subscription<interfaces::msg::ObjectPlaceCommand>(
      this->get_parameter("command_topic").as_string(),
      10,
      std::bind(&ObjectTaskExecutorNode::commandCallback, this, std::placeholders::_1));

  gripper_command_pub_ =
    this->create_publisher<std_msgs::msg::String>(
      this->get_parameter("gripper_command_topic").as_string(),
      10);

  platform_command_pub_ =
    this->create_publisher<std_msgs::msg::String>(
      this->get_parameter("platform_command_topic").as_string(),
      10);

  status_pub_ =
    this->create_publisher<std_msgs::msg::String>(
      this->get_parameter("status_topic").as_string(),
      10);

  navigate_client_ =
    rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  RCLCPP_INFO(this->get_logger(), "Object task executor gestartet");
}

ObjectTaskExecutorNode::~ObjectTaskExecutorNode()
{
  if (worker_.joinable()) {
    worker_.join();
  }
}

void ObjectTaskExecutorNode::objectsCallback(
  const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  objects_ = msg->detections;
}

void ObjectTaskExecutorNode::cornersCallback(
  const interfaces::msg::ManagedCornerArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  corners_ = msg->corners;
}

void ObjectTaskExecutorNode::commandCallback(
  const interfaces::msg::ObjectPlaceCommand::SharedPtr msg)
{
  if (task_running_.exchange(true)) {
    RCLCPP_WARN(this->get_logger(), "Objektauftrag ignoriert: Es laeuft bereits ein Auftrag");
    return;
  }

  if (worker_.joinable()) {
    worker_.join();
  }

  worker_ = std::thread(&ObjectTaskExecutorNode::executeCommand, this, *msg);
}

void ObjectTaskExecutorNode::executeCommand(
  interfaces::msg::ObjectPlaceCommand command)
{
  vision_msgs::msg::Detection3D object;
  interfaces::msg::ManagedCorner corner;

  if (!resolveCommand(command, object, corner)) {
    task_running_ = false;
    return;
  }

  std_msgs::msg::String status;
  status.data = "running";
  status_pub_->publish(status);

  if (!navigateTo(object.bbox.center.position, pickup_offset_x_, pickup_offset_y_, "Objekt")) {
    task_running_ = false;
    return;
  }

  publishActuatorCommand(gripper_command_pub_, gripper_close_command_, "Greifer schliessen");
  sleepForActuator();

  publishActuatorCommand(platform_command_pub_, platform_up_command_, "Plattform anheben");
  sleepForActuator();

  if (!navigateTo(corner.center_map, dropoff_offset_x_, dropoff_offset_y_, "Ziel-Ecke")) {
    task_running_ = false;
    return;
  }

  publishActuatorCommand(platform_command_pub_, platform_down_command_, "Plattform absenken");
  sleepForActuator();

  publishActuatorCommand(gripper_command_pub_, gripper_open_command_, "Greifer oeffnen");

  status.data = "done";
  status_pub_->publish(status);
  RCLCPP_INFO(this->get_logger(), "Objektauftrag abgeschlossen");
  task_running_ = false;
}

bool ObjectTaskExecutorNode::resolveCommand(
  const interfaces::msg::ObjectPlaceCommand & command,
  vision_msgs::msg::Detection3D & object,
  interfaces::msg::ManagedCorner & corner)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  bool object_found = false;
  if (command.object_id >= 0 && static_cast<size_t>(command.object_id) < objects_.size()) {
    object = objects_[command.object_id];
    object_found = true;
  }

  if (!object_found) {
    for (const auto & candidate : objects_) {
      if (getObjectName(candidate) == command.object_name) {
        object = candidate;
        object_found = true;
        break;
      }
    }
  }

  if (!object_found) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Objektauftrag abgebrochen: Objekt '%s' mit ID %d nicht gefunden",
      command.object_name.c_str(),
      command.object_id);
    return false;
  }

  for (const auto & candidate : corners_) {
    if (candidate.corner_uid == command.corner_uid) {
      corner = candidate;
      return true;
    }
  }

  RCLCPP_ERROR(
    this->get_logger(),
    "Objektauftrag abgebrochen: Ecke '%s' nicht gefunden",
    command.corner_uid.c_str());
  return false;
}

bool ObjectTaskExecutorNode::navigateTo(
  const geometry_msgs::msg::Point & point,
  double offset_x,
  double offset_y,
  const std::string & label)
{
  if (!navigate_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "Nav2 Action Server 'navigate_to_pose' nicht erreichbar");
    return false;
  }

  NavigateToPose::Goal goal;
  goal.pose = makeGoalPose(point, offset_x, offset_y);

  RCLCPP_INFO(
    this->get_logger(),
    "Fahre zu %s: x=%.2f y=%.2f",
    label.c_str(),
    goal.pose.pose.position.x,
    goal.pose.pose.position.y);

  auto goal_handle_future = navigate_client_->async_send_goal(goal);
  if (goal_handle_future.wait_for(5s) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "Nav2 Goal fuer %s wurde nicht angenommen", label.c_str());
    return false;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Nav2 hat das Goal fuer %s abgelehnt", label.c_str());
    return false;
  }

  auto result_future = navigate_client_->async_get_result(goal_handle);
  result_future.wait();

  const auto result = result_future.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(this->get_logger(), "Nav2 Goal fuer %s fehlgeschlagen", label.c_str());
    return false;
  }

  return true;
}

geometry_msgs::msg::PoseStamped ObjectTaskExecutorNode::makeGoalPose(
  const geometry_msgs::msg::Point & point,
  double offset_x,
  double offset_y) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = this->now();
  pose.header.frame_id = map_frame_;

  pose.pose.position.x = point.x + offset_x;
  pose.pose.position.y = point.y + offset_y;
  pose.pose.position.z = 0.0;

  pose.pose.orientation.z = std::sin(goal_yaw_ * 0.5);
  pose.pose.orientation.w = std::cos(goal_yaw_ * 0.5);

  return pose;
}

void ObjectTaskExecutorNode::publishActuatorCommand(
  const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr & publisher,
  const std::string & command,
  const std::string & label)
{
  std_msgs::msg::String msg;
  msg.data = command;
  publisher->publish(msg);
  RCLCPP_INFO(this->get_logger(), "%s: %s", label.c_str(), command.c_str());
}

std::string ObjectTaskExecutorNode::getObjectName(
  const vision_msgs::msg::Detection3D & detection) const
{
  if (detection.results.empty()) {
    return "unknown";
  }

  return detection.results[0].hypothesis.class_id;
}

void ObjectTaskExecutorNode::sleepForActuator()
{
  rclcpp::sleep_for(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(actuator_settle_seconds_)));
}
