#pragma once

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "vision_msgs/msg/detection3_d_array.hpp"
#include "vision_msgs/msg/detection3_d.hpp"

struct ManagedObject
{
  int id = 0;

  std::string class_name = "unknown";

  geometry_msgs::msg::Pose pose;

  double size_x = 0.10;
  double size_y = 0.10;
  double size_z = 0.10;

  bool selected = false;
  bool collected = false;
};

class ObjectManagerNode : public rclcpp::Node
{
public:
  ObjectManagerNode();

private:
  void localizedObjectsCallback(
    const vision_msgs::msg::Detection3DArray::SharedPtr msg);

  int findNearestObject(
    const vision_msgs::msg::Detection3D & detection) const;

  std::string getClassName(
    const vision_msgs::msg::Detection3D & detection) const;

  void updateObject(
    ManagedObject & object,
    const vision_msgs::msg::Detection3D & detection);

  void addObject(
    const vision_msgs::msg::Detection3D & detection);

  void publishObjects();
  void publishPoseArray();
  void publishMarkers();
  void publishObstacleCloud();

  std::vector<ManagedObject> objects_;

  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr localized_objects_sub_;

  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr objects_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_cloud_pub_;

  double merge_distance_;
  double obstacle_point_spacing_;

  std::string map_frame_;

  int next_id_;
};