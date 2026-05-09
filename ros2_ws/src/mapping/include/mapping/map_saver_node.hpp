#pragma once

#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "slam_toolbox/srv/save_map.hpp"
#include "slam_toolbox/srv/serialize_pose_graph.hpp"

class MapSaverNode : public rclcpp::Node
{
public:
  MapSaverNode();

private:
  void save_map();

  std::string map_path_;
  bool save_on_start_;

  rclcpp::Client<slam_toolbox::srv::SaveMap>::SharedPtr save_map_client_;
  rclcpp::Client<slam_toolbox::srv::SerializePoseGraph>::SharedPtr serialize_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};