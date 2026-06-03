#include "object_manager/object_manager_node.hpp"

#include <algorithm>
#include <cmath>

#include "sensor_msgs/point_cloud2_iterator.hpp"

ObjectManagerNode::ObjectManagerNode()
: Node("object_manager_node"), next_id_(0)
{
  this->declare_parameter<double>("merge_distance", 0.20);
  this->declare_parameter<double>("obstacle_point_spacing", 0.04);
  this->declare_parameter<std::string>("map_frame", "map");

  merge_distance_ =
    this->get_parameter("merge_distance").as_double();

  obstacle_point_spacing_ =
    this->get_parameter("obstacle_point_spacing").as_double();

  map_frame_ =
    this->get_parameter("map_frame").as_string();

  localized_objects_sub_ =
    this->create_subscription<vision_msgs::msg::Detection3DArray>(
      "/localized_objects",
      10,
      std::bind(
        &ObjectManagerNode::localizedObjectsCallback,
        this,
        std::placeholders::_1
      )
    );

  update_object_pose_service_ =
    this->create_service<interfaces::srv::UpdateObjectPose>(
      "/object_manager/update_object_pose",
      std::bind(
        &ObjectManagerNode::updateObjectPoseCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );

  objects_pub_ =
    this->create_publisher<vision_msgs::msg::Detection3DArray>(
      "/objects",
      10
    );

  pose_array_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/object_poses",
      10
    );

  markers_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/object_markers",
      10
    );

  obstacle_cloud_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/object_obstacles",
      10
    );

  RCLCPP_INFO(this->get_logger(), "Object manager started");
}

void ObjectManagerNode::localizedObjectsCallback(
  const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  for (const auto & detection : msg->detections) {
    const int index = findNearestObject(detection);

    if (index >= 0) {
      updateObject(objects_[index], detection);
    } else if (isLimitedObjectClass(getClassName(detection))) {
      const int class_index = findActiveObjectByClass(getClassName(detection));
      if (class_index >= 0) {
        updateObject(objects_[class_index], detection);
      } else {
        addObject(detection);
      }
    } else {
      addObject(detection);
    }
  }

  publishObjects();
  publishPoseArray();
  publishMarkers();
  publishObstacleCloud();
}

void ObjectManagerNode::updateObjectPoseCallback(
  const std::shared_ptr<interfaces::srv::UpdateObjectPose::Request> request,
  std::shared_ptr<interfaces::srv::UpdateObjectPose::Response> response)
{
  ManagedObject * object = nullptr;

  if (request->object_id >= 0) {
    for (auto & candidate : objects_) {
      if (candidate.id == request->object_id) {
        object = &candidate;
        break;
      }
    }

    if (object == nullptr && static_cast<size_t>(request->object_id) < objects_.size()) {
      object = &objects_[request->object_id];
    }
  }

  if (object == nullptr && !request->object_name.empty()) {
    const std::string request_class_name =
      canonicalObjectClass(request->object_name);
    for (auto & candidate : objects_) {
      if (
        canonicalObjectClass(candidate.class_name) == request_class_name &&
        !candidate.collected)
      {
        object = &candidate;
        break;
      }
    }
  }

  if (object == nullptr) {
    response->success = false;
    response->message = "Objekt nicht gefunden";
    return;
  }

  object->pose = request->pose;
  object->selected = false;
  object->collected = false;

  publishObjects();
  publishPoseArray();
  publishMarkers();
  publishObstacleCloud();

  response->success = true;
  response->message = "Objektpose aktualisiert";
}

int ObjectManagerNode::findNearestObject(
  const vision_msgs::msg::Detection3D & detection) const
{
  const std::string class_name = getClassName(detection);

  const double x = detection.bbox.center.position.x;
  const double y = detection.bbox.center.position.y;

  for (size_t i = 0; i < objects_.size(); ++i) {
    if (objects_[i].collected) {
      continue;
    }

    if (canonicalObjectClass(objects_[i].class_name) != class_name) {
      continue;
    }

    const double dx = objects_[i].pose.position.x - x;
    const double dy = objects_[i].pose.position.y - y;

    const double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < merge_distance_) {
      return static_cast<int>(i);
    }
  }

  return -1;
}

int ObjectManagerNode::findActiveObjectByClass(const std::string & class_name) const
{
  const std::string canonical_class_name = canonicalObjectClass(class_name);

  for (size_t i = 0; i < objects_.size(); ++i) {
    if (objects_[i].collected) {
      continue;
    }

    if (canonicalObjectClass(objects_[i].class_name) == canonical_class_name) {
      return static_cast<int>(i);
    }
  }

  return -1;
}

std::string ObjectManagerNode::getClassName(
  const vision_msgs::msg::Detection3D & detection) const
{
  if (detection.results.empty()) {
    return "unknown";
  }

  return canonicalObjectClass(detection.results[0].hypothesis.class_id);
}

std::string ObjectManagerNode::canonicalObjectClass(const std::string & class_name) const
{
  if (class_name == "dose" || class_name == "can") {
    return "mate";
  }

  if (
    class_name == "rubikcube" ||
    class_name == "rubiks_cube" ||
    class_name == "rubikscube" ||
    class_name == "wurfel")
  {
    return "rubixcube";
  }

  return class_name;
}

bool ObjectManagerNode::isLimitedObjectClass(const std::string & class_name) const
{
  const std::string canonical_class_name = canonicalObjectClass(class_name);
  return (
    canonical_class_name == "ball" ||
    canonical_class_name == "mate" ||
    canonical_class_name == "rubixcube"
  );
}

void ObjectManagerNode::updateObject(
  ManagedObject & object,
  const vision_msgs::msg::Detection3D & detection)
{
  object.pose.position = detection.bbox.center.position;
  object.pose.orientation = detection.bbox.center.orientation;

  object.size_x = detection.bbox.size.x;
  object.size_y = detection.bbox.size.y;
  object.size_z = detection.bbox.size.z;

  object.class_name = getClassName(detection);
}

void ObjectManagerNode::addObject(
  const vision_msgs::msg::Detection3D & detection)
{
  ManagedObject object;

  object.id = next_id_++;
  object.class_name = getClassName(detection);

  object.pose.position = detection.bbox.center.position;
  object.pose.orientation = detection.bbox.center.orientation;

  object.size_x = detection.bbox.size.x;
  object.size_y = detection.bbox.size.y;
  object.size_z = detection.bbox.size.z;

  object.selected = false;
  object.collected = false;

  objects_.push_back(object);

  RCLCPP_INFO(
    this->get_logger(),
    "New object: id=%d class=%s x=%.2f y=%.2f",
    object.id,
    object.class_name.c_str(),
    object.pose.position.x,
    object.pose.position.y
  );
}

void ObjectManagerNode::publishObjects()
{
  vision_msgs::msg::Detection3DArray msg;

  msg.header.stamp = this->now();
  msg.header.frame_id = map_frame_;

  for (const auto & object : objects_) {
    if (object.collected) {
      continue;
    }

    vision_msgs::msg::Detection3D detection;

    detection.header = msg.header;

    detection.bbox.center.position = object.pose.position;
    detection.bbox.center.orientation = object.pose.orientation;

    detection.bbox.size.x = object.size_x;
    detection.bbox.size.y = object.size_y;
    detection.bbox.size.z = object.size_z;

    vision_msgs::msg::ObjectHypothesisWithPose result;
    result.hypothesis.class_id = object.class_name;
    result.hypothesis.score = 1.0;

    detection.results.push_back(result);

    msg.detections.push_back(detection);
  }

  objects_pub_->publish(msg);
}

void ObjectManagerNode::publishPoseArray()
{
  geometry_msgs::msg::PoseArray msg;

  msg.header.stamp = this->now();
  msg.header.frame_id = map_frame_;

  for (const auto & object : objects_) {
    if (!object.collected) {
      msg.poses.push_back(object.pose);
    }
  }

  pose_array_pub_->publish(msg);
}

void ObjectManagerNode::publishMarkers()
{
  visualization_msgs::msg::MarkerArray marker_array;

  for (const auto & object : objects_) {
    visualization_msgs::msg::Marker marker;

    marker.header.stamp = this->now();
    marker.header.frame_id = map_frame_;

    marker.ns = "objects";
    marker.id = object.id;

    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = object.collected ?
      visualization_msgs::msg::Marker::DELETE :
      visualization_msgs::msg::Marker::ADD;

    marker.pose = object.pose;

    marker.scale.x = object.size_x;
    marker.scale.y = object.size_y;
    marker.scale.z = object.size_z;

    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.0;
    marker.color.a = 0.8;

    marker_array.markers.push_back(marker);

    visualization_msgs::msg::Marker text;

    text.header = marker.header;
    text.ns = "object_labels";
    text.id = object.id + 1000;

    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = marker.action;

    text.pose = object.pose;
    text.pose.position.z += object.size_z + 0.08;

    text.scale.z = 0.08;

    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
    text.color.a = 1.0;

    text.text =
      std::to_string(object.id) + ": " + object.class_name;

    marker_array.markers.push_back(text);
  }

  markers_pub_->publish(marker_array);
}

void ObjectManagerNode::publishObstacleCloud()
{
  std::vector<std::array<float, 3>> points;

  const double spacing = std::max(0.01, obstacle_point_spacing_);

  for (const auto & object : objects_) {
    if (object.collected || object.selected) {
      continue;
    }

    const double half_x = object.size_x / 2.0;
    const double half_y = object.size_y / 2.0;

    for (double dx = -half_x; dx <= half_x; dx += spacing) {
      for (double dy = -half_y; dy <= half_y; dy += spacing) {
        points.push_back({
          static_cast<float>(object.pose.position.x + dx),
          static_cast<float>(object.pose.position.y + dy),
          static_cast<float>(object.size_z / 2.0)
        });
      }
    }
  }

  sensor_msgs::msg::PointCloud2 cloud;

  cloud.header.stamp = this->now();
  cloud.header.frame_id = map_frame_;

  cloud.height = 1;
  cloud.width = points.size();

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  for (const auto & point : points) {
    *iter_x = point[0];
    *iter_y = point[1];
    *iter_z = point[2];

    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  obstacle_cloud_pub_->publish(cloud);
}
