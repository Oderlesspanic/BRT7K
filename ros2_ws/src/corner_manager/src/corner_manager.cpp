#include "corner_manager/corner_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

CornerManager::CornerManager(const CornerManagerConfig & config)
: config_(config),
  next_id_(1)
{
}

void CornerManager::process_corner(
  const interfaces::msg::ValidColorCorner & corner,
  const rclcpp::Time & stamp)
{
  auto managed = valid_to_managed(corner, stamp);
  merge_or_add(managed);
}

void CornerManager::process_corners(
  const std::vector<interfaces::msg::ValidColorCorner> & corners,
  const rclcpp::Time & stamp)
{
  for (const auto & corner : corners) {
    process_corner(corner, stamp);
  }
}

void CornerManager::remove_expired(const rclcpp::Time & now)
{
  if (!config_.forget_after_time) {
    return;
  }

  corners_.erase(
    std::remove_if(
      corners_.begin(),
      corners_.end(),
      [&](const auto & corner) {
        const rclcpp::Time last_seen(corner.last_seen);
        return (now - last_seen).seconds() > config_.forget_after_seconds;
      }),
    corners_.end());
}

void CornerManager::enforce_max_corners()
{
  if (corners_.size() <= config_.max_corners) {
    return;
  }

  std::sort(
    corners_.begin(),
    corners_.end(),
    [](const auto & a, const auto & b) {
      return rclcpp::Time(a.last_seen) > rclcpp::Time(b.last_seen);
    });

  corners_.resize(config_.max_corners);
}

const std::vector<interfaces::msg::ManagedCorner> &
CornerManager::get_corners() const
{
  return corners_;
}

interfaces::msg::ManagedCorner CornerManager::valid_to_managed(
  const interfaces::msg::ValidColorCorner & corner,
  const rclcpp::Time & stamp)
{
  interfaces::msg::ManagedCorner managed;

  managed.corner_uid = "";
  managed.corner_id = corner.corner_id;
  managed.wall_id = corner.wall_id;

  managed.center_map = corner.center_map;
  managed.corners_map = corner.corners_map;

  managed.map_width = corner.map_width;
  managed.map_height = corner.map_height;
  managed.map_area = corner.map_area;

  managed.mean_h = corner.mean_h;
  managed.mean_s = corner.mean_s;
  managed.mean_v = corner.mean_v;

  managed.median_h = corner.median_h;
  managed.median_s = corner.median_s;
  managed.median_v = corner.median_v;

  managed.std_h = corner.std_h;
  managed.std_s = corner.std_s;
  managed.std_v = corner.std_v;

  managed.validation_score = corner.validation_score;

  managed.seen_count = 1;
  managed.first_seen = stamp;
  managed.last_seen = stamp;

  return managed;
}

void CornerManager::merge_or_add(
  const interfaces::msg::ManagedCorner & incoming)
{
  const auto match_index = find_best_match(incoming);

  if (match_index.has_value()) {
    update_existing(corners_[match_index.value()], incoming);
    return;
  }

  auto added = incoming;
  added.corner_uid = make_corner_uid();

  corners_.push_back(added);
}

std::optional<std::size_t> CornerManager::find_best_match(
  const interfaces::msg::ManagedCorner & incoming) const
{
  double best_score = std::numeric_limits<double>::max();
  std::optional<std::size_t> best_index;

  for (std::size_t i = 0; i < corners_.size(); ++i) {
    if (!is_candidate_match(corners_[i], incoming)) {
      continue;
    }

    const double score = compute_match_score(corners_[i], incoming);

    if (score < best_score) {
      best_score = score;
      best_index = i;
    }
  }

  return best_index;
}

bool CornerManager::is_candidate_match(
  const interfaces::msg::ManagedCorner & existing,
  const interfaces::msg::ManagedCorner & incoming) const
{
  if (existing.corner_id != incoming.corner_id) {
    return false;
  }

  if (existing.wall_id != incoming.wall_id) {
    return false;
  }

  if (center_distance(existing, incoming) > config_.merge_distance) {
    return false;
  }

  if (config_.use_hsv_for_merge) {
    const bool hue_reliable =
      hue_is_reliable(existing) && hue_is_reliable(incoming);

    if (hue_reliable) {
      if (hue_distance(existing.median_h, incoming.median_h) >
          config_.max_hue_distance)
      {
        return false;
      }
    }

    if (std::abs(existing.median_s - incoming.median_s) >
        config_.max_s_difference)
    {
      return false;
    }

    if (std::abs(existing.median_v - incoming.median_v) >
        config_.max_v_difference)
    {
      return false;
    }
  }

  return true;
}

double CornerManager::compute_match_score(
  const interfaces::msg::ManagedCorner & existing,
  const interfaces::msg::ManagedCorner & incoming) const
{
  const double dist_score =
    center_distance(existing, incoming) /
    std::max(config_.merge_distance, 1e-9);

  double hue_score = 0.0;

  if (config_.use_hsv_for_merge &&
      hue_is_reliable(existing) &&
      hue_is_reliable(incoming))
  {
    hue_score =
      hue_distance(existing.median_h, incoming.median_h) /
      std::max(config_.max_hue_distance, 1e-9);
  }

  const double s_score =
    std::abs(existing.median_s - incoming.median_s) /
    std::max(config_.max_s_difference, 1e-9);

  const double v_score =
    std::abs(existing.median_v - incoming.median_v) /
    std::max(config_.max_v_difference, 1e-9);

  return 2.0 * dist_score + 1.5 * hue_score + 0.5 * s_score + 0.5 * v_score;
}

double CornerManager::center_distance(
  const interfaces::msg::ManagedCorner & a,
  const interfaces::msg::ManagedCorner & b) const
{
  const double dx = a.center_map.x - b.center_map.x;
  const double dy = a.center_map.y - b.center_map.y;
  const double dz = a.center_map.z - b.center_map.z;

  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double CornerManager::hue_distance(double h1, double h2) const
{
  const double d = std::abs(h1 - h2);
  return std::min(d, 180.0 - d);
}

bool CornerManager::hue_is_reliable(
  const interfaces::msg::ManagedCorner & corner) const
{
  return corner.median_s >= config_.min_s_for_hue_match;
}

std::string CornerManager::make_corner_uid()
{
  std::ostringstream oss;
  oss << "managed_corner_" << next_id_++;
  return oss.str();
}

void CornerManager::update_existing(
  interfaces::msg::ManagedCorner & existing,
  const interfaces::msg::ManagedCorner & incoming)
{
  const double old_count = static_cast<double>(existing.seen_count);
  const double new_count = old_count + 1.0;

  existing.center_map.x =
    (existing.center_map.x * old_count + incoming.center_map.x) / new_count;
  existing.center_map.y =
    (existing.center_map.y * old_count + incoming.center_map.y) / new_count;
  existing.center_map.z =
    (existing.center_map.z * old_count + incoming.center_map.z) / new_count;

  existing.corners_map = incoming.corners_map;

  existing.map_width =
    (existing.map_width * old_count + incoming.map_width) / new_count;
  existing.map_height =
    (existing.map_height * old_count + incoming.map_height) / new_count;
  existing.map_area =
    (existing.map_area * old_count + incoming.map_area) / new_count;

  existing.mean_h = incoming.mean_h;
  existing.mean_s = incoming.mean_s;
  existing.mean_v = incoming.mean_v;

  existing.median_h = incoming.median_h;
  existing.median_s = incoming.median_s;
  existing.median_v = incoming.median_v;

  existing.std_h = incoming.std_h;
  existing.std_s = incoming.std_s;
  existing.std_v = incoming.std_v;

  existing.validation_score =
    std::max(existing.validation_score, incoming.validation_score);

  existing.seen_count += 1;
  existing.last_seen = incoming.last_seen;
}