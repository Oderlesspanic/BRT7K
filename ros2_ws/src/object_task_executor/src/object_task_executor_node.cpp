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
  this->declare_parameter<std::string>("robot_pose_topic", "/amcl_pose");
  this->declare_parameter<std::string>("cancel_service", "/object_task_executor/cancel_task");
  this->declare_parameter<std::string>(
    "update_object_pose_service",
    "/object_manager/update_object_pose");
  this->declare_parameter<std::string>("set_pos_topic", "/esp32_gripper/set_pos");
  this->declare_parameter<std::string>("status_topic", "/object_task_executor/status");
  this->declare_parameter<int>("gripper_left_close_pos", -6500);
  this->declare_parameter<int>("gripper_left_open_pos",  0);
  this->declare_parameter<int>("gripper_right_close_pos", 6500);
  this->declare_parameter<int>("gripper_right_open_pos",  0);
  this->declare_parameter<int>("lifting_up_pos",   20000);
  this->declare_parameter<int>("lifting_down_pos",     0);
  this->declare_parameter<double>("goal_yaw", 0.0);
  this->declare_parameter<double>("pickup_offset_x", 0.0);
  this->declare_parameter<double>("pickup_offset_y", 0.0);
  this->declare_parameter<double>("dropoff_offset_x", 0.0);
  this->declare_parameter<double>("dropoff_offset_y", 0.0);
  this->declare_parameter<double>("actuator_settle_seconds", 1.0);

  map_frame_ = this->get_parameter("map_frame").as_string();
  gripper_left_close_pos_ = this->get_parameter("gripper_left_close_pos").as_int();
  gripper_left_open_pos_ = this->get_parameter("gripper_left_open_pos").as_int();
  gripper_right_close_pos_ = this->get_parameter("gripper_right_close_pos").as_int();
  gripper_right_open_pos_ = this->get_parameter("gripper_right_open_pos").as_int();
  lifting_up_pos_ = this->get_parameter("lifting_up_pos").as_int();
  lifting_down_pos_ = this->get_parameter("lifting_down_pos").as_int();
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

  robot_pose_sub_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      this->get_parameter("robot_pose_topic").as_string(),
      10,
      std::bind(&ObjectTaskExecutorNode::robotPoseCallback, this, std::placeholders::_1));

  set_pos_pub_ =
    this->create_publisher<std_msgs::msg::Int32>(
      this->get_parameter("set_pos_topic").as_string(),
      10);

  status_pub_ =
    this->create_publisher<std_msgs::msg::String>(
      this->get_parameter("status_topic").as_string(),
      10);

  navigate_client_ =
    rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  update_object_pose_client_ =
    this->create_client<interfaces::srv::UpdateObjectPose>(
      this->get_parameter("update_object_pose_service").as_string());

  cancel_task_service_ =
    this->create_service<std_srvs::srv::Trigger>(
      this->get_parameter("cancel_service").as_string(),
      std::bind(
        &ObjectTaskExecutorNode::cancelTaskCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

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

void ObjectTaskExecutorNode::robotPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  latest_robot_pose_ = msg->pose.pose;
}

void ObjectTaskExecutorNode::cancelTaskCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  requestCancel();
  response->success = true;
  response->message = task_running_ ? "Abbruch angefordert" : "Kein aktiver Auftrag";
}

void ObjectTaskExecutorNode::executeCommand(
  interfaces::msg::ObjectPlaceCommand command)
{
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    active_command_ = command;
    object_picked_up_ = false;
  }
  cancel_requested_ = false;

  vision_msgs::msg::Detection3D object;
  interfaces::msg::ManagedCorner corner;

  if (!resolveCommand(command, object, corner)) {
    publishStatus("failed");
    std::lock_guard<std::mutex> lock(task_mutex_);
    active_command_.reset();
    task_running_ = false;
    return;
  }

  publishStatus("running");

  if (!navigateTo(object.bbox.center.position, pickup_offset_x_, pickup_offset_y_, "Objekt")) {
    if (cancel_requested_) {
      publishStatus("canceled");
    } else {
      publishStatus("failed");
    }
    std::lock_guard<std::mutex> lock(task_mutex_);
    active_command_.reset();
    task_running_ = false;
    return;
  }

  if (cancel_requested_) {
    publishStatus("canceled");
    std::lock_guard<std::mutex> lock(task_mutex_);
    active_command_.reset();
    task_running_ = false;
    return;
  }

  publishGripperClose();
  sleepForActuator();

  publishSetPos(3, lifting_up_pos_, "Plattform anheben");
  sleepForActuator();
  object_picked_up_ = true;

  if (cancel_requested_) {
    dropCurrentObjectAtCurrentPose();
    publishStatus("canceled");
    std::lock_guard<std::mutex> lock(task_mutex_);
    active_command_.reset();
    task_running_ = false;
    return;
  }

  if (!navigateTo(corner.center_map, dropoff_offset_x_, dropoff_offset_y_, "Ziel-Ecke")) {
    if (cancel_requested_) {
      dropCurrentObjectAtCurrentPose();
      publishStatus("canceled");
    } else {
      publishStatus("failed");
    }
    std::lock_guard<std::mutex> lock(task_mutex_);
    active_command_.reset();
    task_running_ = false;
    return;
  }

  publishSetPos(3, lifting_down_pos_, "Plattform absenken");
  sleepForActuator();

  publishGripperOpen();
  object_picked_up_ = false;

  publishStatus("done");
  RCLCPP_INFO(this->get_logger(), "Objektauftrag abgeschlossen");
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    active_command_.reset();
  }
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

  if (cancel_requested_) {
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

  if (cancel_requested_) {
    navigate_client_->async_cancel_goal(goal_handle);
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    active_goal_handle_ = goal_handle;
  }

  auto result_future = navigate_client_->async_get_result(goal_handle);
  while (result_future.wait_for(100ms) != std::future_status::ready) {
    if (cancel_requested_) {
      navigate_client_->async_cancel_goal(goal_handle);
      std::lock_guard<std::mutex> lock(task_mutex_);
      active_goal_handle_.reset();
      return false;
    }
  }

  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    active_goal_handle_.reset();
  }

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

// Codierung: motor * 100000 + encoderPos + 50000  (motor: 1=GripL, 2=GripR, 3=Lift)
void ObjectTaskExecutorNode::publishSetPos(int32_t motor, int32_t encoder_pos, const std::string & label)
{
  std_msgs::msg::Int32 msg;
  msg.data = motor * 100000 + encoder_pos + 50000;
  set_pos_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "%s: M%d→%d", label.c_str(), (int)motor, (int)encoder_pos);
}

void ObjectTaskExecutorNode::publishGripperClose()
{
  publishSetPos(1, gripper_left_close_pos_,  "Greifer Links schliessen");
  publishSetPos(2, gripper_right_close_pos_, "Greifer Rechts schliessen");
}

void ObjectTaskExecutorNode::publishGripperOpen()
{
  publishSetPos(1, gripper_left_open_pos_,  "Greifer Links oeffnen");
  publishSetPos(2, gripper_right_open_pos_, "Greifer Rechts oeffnen");
}

void ObjectTaskExecutorNode::publishStatus(const std::string & status)
{
  std_msgs::msg::String msg;
  msg.data = status;
  status_pub_->publish(msg);
}

void ObjectTaskExecutorNode::requestCancel()
{
  cancel_requested_ = true;

  GoalHandleNavigateToPose::SharedPtr goal_handle;
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    goal_handle = active_goal_handle_;
  }

  if (goal_handle) {
    navigate_client_->async_cancel_goal(goal_handle);
  }
}

void ObjectTaskExecutorNode::dropCurrentObjectAtCurrentPose()
{
  std::optional<interfaces::msg::ObjectPlaceCommand> command;
  std::optional<geometry_msgs::msg::Pose> pose;

  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    command = active_command_;
    pose = latest_robot_pose_;
  }

  publishSetPos(3, lifting_down_pos_, "Plattform absenken");
  sleepForActuator();

  publishGripperOpen();
  sleepForActuator();
  object_picked_up_ = false;

  if (command && pose) {
    updateObjectManagerPose(*command, *pose);
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Objekt abgesetzt, aber keine aktive Objekt-/Roboterpose zum Speichern verfuegbar");
  }
}

void ObjectTaskExecutorNode::updateObjectManagerPose(
  const interfaces::msg::ObjectPlaceCommand & command,
  const geometry_msgs::msg::Pose & pose)
{
  if (!update_object_pose_client_->wait_for_service(2s)) {
    RCLCPP_WARN(this->get_logger(), "Object-Manager-Update-Service nicht erreichbar");
    return;
  }

  auto request = std::make_shared<interfaces::srv::UpdateObjectPose::Request>();
  request->object_id = command.object_id;
  request->object_name = command.object_name;
  request->pose = pose;

  update_object_pose_client_->async_send_request(request);
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
