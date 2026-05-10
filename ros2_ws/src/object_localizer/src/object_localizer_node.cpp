#include "object_localizer/object_localizer_node.hpp"

#include <cmath>

#include "cv_bridge/cv_bridge.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

ObjectLocalizerNode::ObjectLocalizerNode()
: Node("object_localizer_node")
{
  this->declare_parameter<std::string>("target_frame", "map");
  this->declare_parameter<std::string>("camera_frame", "camera_link");
  this->declare_parameter<double>("ground_z", 0.0);

  this->declare_parameter<double>("objects.default.size_x", 0.10);
  this->declare_parameter<double>("objects.default.size_y", 0.10);
  this->declare_parameter<double>("objects.default.size_z", 0.10);
  this->declare_parameter<double>("objects.default.offset_x", 0.0);
  this->declare_parameter<double>("objects.default.offset_y", 0.0);

  this->declare_parameter<double>("objects.rubixcube.size_x", 0.057);
  this->declare_parameter<double>("objects.rubixcube.size_y", 0.057);
  this->declare_parameter<double>("objects.rubixcube.size_z", 0.057);
  this->declare_parameter<double>("objects.rubixcube.offset_x", 0.0);
  this->declare_parameter<double>("objects.rubixcube.offset_y", 0.0);

  this->declare_parameter<double>("objects.mate.size_x", 0.066);
  this->declare_parameter<double>("objects.mate.size_y", 0.066);
  this->declare_parameter<double>("objects.mate.size_z", 0.115);
  this->declare_parameter<double>("objects.mate.offset_x", 0.0);
  this->declare_parameter<double>("objects.mate.offset_y", 0.0);

  this->declare_parameter<double>("objects.ball.size_x", 0.19);
  this->declare_parameter<double>("objects.ball.size_y", 0.19);
  this->declare_parameter<double>("objects.ball.size_z", 0.19);
  this->declare_parameter<double>("objects.ball.offset_x", 0.0);
  this->declare_parameter<double>("objects.ball.offset_y", 0.0);

  this->declare_parameter<double>("objects.wurfel.size_x", 0.057);
  this->declare_parameter<double>("objects.wurfel.size_y", 0.057);
  this->declare_parameter<double>("objects.wurfel.size_z", 0.057);
  this->declare_parameter<double>("objects.wurfel.offset_x", 0.0);
  this->declare_parameter<double>("objects.wurfel.offset_y", 0.0);

  this->declare_parameter<double>("objects.fhgr_logo.size_x", 0.057);
  this->declare_parameter<double>("objects.fhgr_logo.size_y", 0.057);
  this->declare_parameter<double>("objects.fhgr_logo.size_z", 0.057);
  this->declare_parameter<double>("objects.fhgr_logo.offset_x", 0.0);
  this->declare_parameter<double>("objects.fhgr_logo.offset_y", 0.0);

  target_frame_ = this->get_parameter("target_frame").as_string();
  camera_frame_ = this->get_parameter("camera_frame").as_string();
  ground_z_ = this->get_parameter("ground_z").as_double();

  loadObjectConfigs();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image_raw",
    10,
    std::bind(&ObjectLocalizerNode::imageCallback, this, std::placeholders::_1)
  );

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/camera_info",
    10,
    std::bind(&ObjectLocalizerNode::cameraInfoCallback, this, std::placeholders::_1)
  );

  detections_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
    "/room_vision/detections",
    10,
    std::bind(&ObjectLocalizerNode::detectionsCallback, this, std::placeholders::_1)
  );

  localized_objects_pub_ =
    this->create_publisher<vision_msgs::msg::Detection3DArray>(
      "/localized_objects",
      10
    );

  pose_array_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/localized_object_poses",
      10
    );

  debug_image_pub_ =
    this->create_publisher<sensor_msgs::msg::Image>(
      "/object_localizer/debug_image",
      10
    );

  RCLCPP_INFO(this->get_logger(), "Object localizer started");
}

void ObjectLocalizerNode::loadObjectConfigs()
{
  const std::vector<std::string> names = {
    "default",
    "rubixcube",
    "mate",
    "ball",
    "wurfel",
    "fhgr_logo"
  };

  for (const auto & name : names) {
    ObjectConfig config;

    config.size_x =
      this->get_parameter("objects." + name + ".size_x").as_double();
    config.size_y =
      this->get_parameter("objects." + name + ".size_y").as_double();
    config.size_z =
      this->get_parameter("objects." + name + ".size_z").as_double();
    config.offset_x =
      this->get_parameter("objects." + name + ".offset_x").as_double();
    config.offset_y =
      this->get_parameter("objects." + name + ".offset_y").as_double();

    object_configs_[name] = config;
  }
}

void ObjectLocalizerNode::imageCallback(
  const sensor_msgs::msg::Image::SharedPtr msg)
{
  last_image_ = msg;
}

void ObjectLocalizerNode::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  last_camera_info_ = msg;
}

void ObjectLocalizerNode::detectionsCallback(
  const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  if (!last_image_ || !last_camera_info_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Waiting for image and camera_info"
    );
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;

  try {
    cv_ptr = cv_bridge::toCvCopy(last_image_, "bgr8");
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
    return;
  }

  cv::Mat debug_image = cv_ptr->image.clone();

  vision_msgs::msg::Detection3DArray localized_msg;
  localized_msg.header.stamp = this->now();
  localized_msg.header.frame_id = target_frame_;

  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header = localized_msg.header;

  for (const auto & detection : msg->detections) {
    const std::string class_name = getClassName(detection);
    const ObjectConfig object_config = getObjectConfig(class_name);

    cv::Point2d contact_pixel;

    if (!ground_contact_localizer_.findGroundContactPoint(
        cv_ptr->image,
        detection,
        contact_pixel))
    {
      continue;
    }

    geometry_msgs::msg::PoseStamped pose_target;

    if (!projectPixelToGround(contact_pixel, pose_target)) {
      continue;
    }

    pose_target.pose.position.x += object_config.offset_x;
    pose_target.pose.position.y += object_config.offset_y;
    pose_target.pose.position.z = object_config.size_z / 2.0;

    vision_msgs::msg::Detection3D det3d;
    det3d.header = pose_target.header;

    det3d.bbox.center.position = pose_target.pose.position;
    det3d.bbox.center.orientation = pose_target.pose.orientation;

    det3d.bbox.size.x = object_config.size_x;
    det3d.bbox.size.y = object_config.size_y;
    det3d.bbox.size.z = object_config.size_z;

    det3d.results = detection.results;

    localized_msg.detections.push_back(det3d);
    pose_array.poses.push_back(pose_target.pose);

    cv::circle(debug_image, contact_pixel, 5, cv::Scalar(0, 0, 255), -1);

    cv::putText(
      debug_image,
      class_name,
      cv::Point(
        static_cast<int>(contact_pixel.x) + 8,
        static_cast<int>(contact_pixel.y) - 8
      ),
      cv::FONT_HERSHEY_SIMPLEX,
      0.5,
      cv::Scalar(0, 0, 255),
      1
    );
  }

  localized_objects_pub_->publish(localized_msg);
  pose_array_pub_->publish(pose_array);

  auto debug_msg =
    cv_bridge::CvImage(last_image_->header, "bgr8", debug_image).toImageMsg();

  debug_image_pub_->publish(*debug_msg);
}

bool ObjectLocalizerNode::projectPixelToGround(
  const cv::Point2d & pixel,
  geometry_msgs::msg::PoseStamped & pose_target)
{
  if (!last_camera_info_ || !last_image_) {
    return false;
  }

  const double fx = last_camera_info_->k[0];
  const double fy = last_camera_info_->k[4];
  const double cx = last_camera_info_->k[2];
  const double cy = last_camera_info_->k[5];

  const double x = (pixel.x - cx) / fx;
  const double y = (pixel.y - cy) / fy;

  geometry_msgs::msg::PoseStamped ray_origin_camera;
  ray_origin_camera.header.stamp = last_image_->header.stamp;
  ray_origin_camera.header.frame_id = camera_frame_;
  ray_origin_camera.pose.position.x = 0.0;
  ray_origin_camera.pose.position.y = 0.0;
  ray_origin_camera.pose.position.z = 0.0;
  ray_origin_camera.pose.orientation.w = 1.0;

  geometry_msgs::msg::PoseStamped ray_point_camera;
  ray_point_camera.header = ray_origin_camera.header;
  ray_point_camera.pose.position.x = x;
  ray_point_camera.pose.position.y = y;
  ray_point_camera.pose.position.z = 1.0;
  ray_point_camera.pose.orientation.w = 1.0;

  geometry_msgs::msg::PoseStamped origin_target;
  geometry_msgs::msg::PoseStamped point_target;

  try {
    origin_target = tf_buffer_->transform(
      ray_origin_camera,
      target_frame_,
      tf2::durationFromSec(0.2)
    );

    point_target = tf_buffer_->transform(
      ray_point_camera,
      target_frame_,
      tf2::durationFromSec(0.2)
    );
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
    return false;
  }

  const double ox = origin_target.pose.position.x;
  const double oy = origin_target.pose.position.y;
  const double oz = origin_target.pose.position.z;

  const double px = point_target.pose.position.x;
  const double py = point_target.pose.position.y;
  const double pz = point_target.pose.position.z;

  const double dx = px - ox;
  const double dy = py - oy;
  const double dz = pz - oz;

  if (std::abs(dz) < 1e-6) {
    return false;
  }

  const double t = (ground_z_ - oz) / dz;

  if (t <= 0.0) {
    return false;
  }

  pose_target.header.stamp = this->now();
  pose_target.header.frame_id = target_frame_;

  pose_target.pose.position.x = ox + t * dx;
  pose_target.pose.position.y = oy + t * dy;
  pose_target.pose.position.z = ground_z_;
  pose_target.pose.orientation.w = 1.0;

  return true;
}

std::string ObjectLocalizerNode::getClassName(
  const vision_msgs::msg::Detection2D & detection) const
{
  if (detection.results.empty()) {
    return "unknown";
  }

  return detection.results[0].hypothesis.class_id;
}

ObjectConfig ObjectLocalizerNode::getObjectConfig(
  const std::string & class_name) const
{
  auto it = object_configs_.find(class_name);

  if (it != object_configs_.end()) {
    return it->second;
  }

  return object_configs_.at("default");
}