#include "mapping/map_saver_node.hpp"

using namespace std::chrono_literals;

MapSaverNode::MapSaverNode()
: Node("map_saver_node")
{
  this->declare_parameter<std::string>("map_path", "/home/pi/BRT7K/ros2_ws/maps/map/arena_map");

  this->declare_parameter<bool>("save_on_start", false);
  this->declare_parameter<bool>("shutdown_after_save", true);

  map_path_ = this->get_parameter("map_path").as_string();

  save_on_start_ = this->get_parameter("save_on_start").as_bool();
  shutdown_after_save_ = this->get_parameter("shutdown_after_save").as_bool();

  save_map_client_ = this->create_client<slam_toolbox::srv::SaveMap>("/slam_toolbox/save_map");

  serialize_client_ = this->create_client<slam_toolbox::srv::SerializePoseGraph>("/slam_toolbox/serialize_map");

  RCLCPP_INFO(this->get_logger(), "Map saver node started");

  RCLCPP_INFO(this->get_logger(), "Map path: %s", map_path_.c_str());

  if (save_on_start_) {
    timer_ = this->create_wall_timer(2s, std::bind(&MapSaverNode::save_map, this));
  }
}

void MapSaverNode::save_map()
{
  timer_->cancel();

  if (!save_map_client_->wait_for_service(5s)) {
    RCLCPP_ERROR(this->get_logger(), "Service /slam_toolbox/save_map not available");
    return;
  }

  if (!serialize_client_->wait_for_service(5s)) {
    RCLCPP_ERROR(this->get_logger(), "Service /slam_toolbox/serialize_map not available");
    return;
  }

  auto save_request = std::make_shared<slam_toolbox::srv::SaveMap::Request>();

  save_request->name.data = map_path_;

  RCLCPP_INFO(this->get_logger(), "Saving occupancy map...");

  auto save_future = save_map_client_->async_send_request(save_request);

  if (
    rclcpp::spin_until_future_complete(
      this->get_node_base_interface(),
      save_future,
      10s
    ) != rclcpp::FutureReturnCode::SUCCESS
  ) {
    RCLCPP_ERROR(this->get_logger(), "Failed to call save_map service");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Occupancy map saved");

  auto serialize_request =std::make_shared<slam_toolbox::srv::SerializePoseGraph::Request>();

  serialize_request->filename = map_path_;

  RCLCPP_INFO(this->get_logger(), "Saving pose graph...");

  auto serialize_future = serialize_client_->async_send_request(serialize_request);

  if (
    rclcpp::spin_until_future_complete(
      this->get_node_base_interface(),
      serialize_future,
      10s
    ) != rclcpp::FutureReturnCode::SUCCESS
  ) {
    RCLCPP_ERROR(this->get_logger(), "Failed to call serialize_map service");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Pose graph saved");

  if (shutdown_after_save_) {
    RCLCPP_INFO(this->get_logger(), "Map saver finished; shutting down");
    rclcpp::shutdown();
  }
}
