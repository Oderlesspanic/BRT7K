#include "edge_color/semantic_color_edge_map_manager.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>

using json = nlohmann::json;

SemanticColorEdgeMapManager::SemanticColorEdgeMapManager()
: merge_distance_(0.20),
  max_reacquire_distance_factor_(3.0),
  save_to_file_(true),
  save_path_("/tmp/color_edge_map.json"),
  max_objects_(100),
  forget_after_time_(false),
  forget_after_seconds_(300.0),
  print_updates_(true),
  use_hue_for_merge_(true),
  max_hue_distance_(12.0),
  min_s_for_hue_match_(40.0),
  use_map_area_for_merge_(true),
  max_area_relative_difference_(0.40),
  next_id_(1)
{
}

void SemanticColorEdgeMapManager::configure(
  double merge_distance,
  double max_reacquire_distance_factor,
  bool save_to_file,
  const std::string & save_path,
  std::size_t max_objects,
  bool forget_after_time,
  double forget_after_seconds,
  bool print_updates,
  bool use_hue_for_merge,
  double max_hue_distance,
  double min_s_for_hue_match,
  bool use_map_area_for_merge,
  double max_area_relative_difference)
{
  merge_distance_ = merge_distance;
  max_reacquire_distance_factor_ = max_reacquire_distance_factor;

  save_to_file_ = save_to_file;
  save_path_ = save_path;
  max_objects_ = max_objects;
  forget_after_time_ = forget_after_time;
  forget_after_seconds_ = forget_after_seconds;
  print_updates_ = print_updates;

  use_hue_for_merge_ = use_hue_for_merge;
  max_hue_distance_ = max_hue_distance;
  min_s_for_hue_match_ = min_s_for_hue_match;

  use_map_area_for_merge_ = use_map_area_for_merge;
  max_area_relative_difference_ = max_area_relative_difference;
}

void SemanticColorEdgeMapManager::process_patch(
  const interfaces::msg::MappedColorPatch & patch,
  const rclcpp::Time & stamp)
{
  const ManagedSurface incoming = patch_to_surface(patch, stamp);
  merge_or_add(incoming);
}

void SemanticColorEdgeMapManager::process_patches(
  const std::vector<interfaces::msg::MappedColorPatch> & patches,
  const rclcpp::Time & stamp)
{
  for (const auto & patch : patches) {
    process_patch(patch, stamp);
  }
}

void SemanticColorEdgeMapManager::remove_expired(const rclcpp::Time & now)
{
  if (!forget_after_time_) {
    return;
  }

  const auto old_size = surfaces_.size();

  surfaces_.erase(
    std::remove_if(
      surfaces_.begin(),
      surfaces_.end(),
      [&](const ManagedSurface & surface) {
        const double age = (now - surface.last_seen).seconds();
        return age > forget_after_seconds_;
      }),
    surfaces_.end());

  if (print_updates_ && surfaces_.size() != old_size) {
    std::cout << "[semantic_color_edge_map_manager] expired removed: "
              << (old_size - surfaces_.size()) << std::endl;
  }
}

void SemanticColorEdgeMapManager::enforce_max_objects()
{
  if (surfaces_.size() <= max_objects_) {
    return;
  }

  sort_by_last_seen();

  const auto remove_count = surfaces_.size() - max_objects_;
  surfaces_.erase(surfaces_.begin(), surfaces_.begin() + static_cast<long>(remove_count));

  if (print_updates_) {
    std::cout << "[semantic_color_edge_map_manager] max_objects enforced, removed: "
              << remove_count << std::endl;
  }
}

bool SemanticColorEdgeMapManager::save_to_json() const
{
  if (!save_to_file_) {
    return true;
  }

  std::ofstream file(save_path_);
  if (!file.is_open()) {
    return false;
  }

  file << to_json().dump(2);
  return true;
}

bool SemanticColorEdgeMapManager::load_from_json()
{
  if (!save_to_file_) {
    return true;
  }

  std::ifstream file(save_path_);
  if (!file.is_open()) {
    return false;
  }

  json root;
  file >> root;

  if (!root.contains("surfaces") || !root["surfaces"].is_array()) {
    return false;
  }

  surfaces_.clear();

  std::uint64_t max_id = 0;

  for (const auto & entry : root["surfaces"]) {
    ManagedSurface surface = surface_from_json(entry);
    surfaces_.push_back(surface);

    try {
      const auto pos = surface.id.find_last_of('_');
      if (pos != std::string::npos) {
        const auto number = static_cast<std::uint64_t>(std::stoull(surface.id.substr(pos + 1)));
        max_id = std::max(max_id, number);
      }
    } catch (...) {
    }
  }

  next_id_ = max_id + 1;
  return true;
}

json SemanticColorEdgeMapManager::to_json() const
{
  json root;
  root["surfaces"] = json::array();

  for (const auto & surface : surfaces_) {
    root["surfaces"].push_back(surface_to_json(surface));
  }

  return root;
}

const std::vector<SemanticColorEdgeMapManager::ManagedSurface> &
SemanticColorEdgeMapManager::get_surfaces() const
{
  return surfaces_;
}

SemanticColorEdgeMapManager::ManagedSurface
SemanticColorEdgeMapManager::patch_to_surface(
  const interfaces::msg::MappedColorPatch & patch,
  const rclcpp::Time & stamp) const
{
  ManagedSurface surface;

  surface.id = "";

  surface.center_map = patch.center_map;
  surface.corners_map = patch.corners_map;

  surface.pixel_area = patch.pixel_area;
  surface.map_area = compute_polygon_area(patch.corners_map);

  surface.mean_h = patch.mean_h;
  surface.mean_s = patch.mean_s;
  surface.mean_v = patch.mean_v;

  surface.median_h = patch.median_h;
  surface.median_s = patch.median_s;
  surface.median_v = patch.median_v;

  surface.std_h = patch.std_h;
  surface.std_s = patch.std_s;
  surface.std_v = patch.std_v;

  surface.seen_count = 1;
  surface.first_seen = stamp;
  surface.last_seen = stamp;

  return surface;
}

void SemanticColorEdgeMapManager::merge_or_add(const ManagedSurface & incoming)
{
  std::size_t best_index = surfaces_.size();
  double best_score = std::numeric_limits<double>::max();

  for (std::size_t i = 0; i < surfaces_.size(); ++i) {
    if (!is_candidate_match(surfaces_[i], incoming)) {
      continue;
    }

    const double score = compute_match_score(surfaces_[i], incoming);
    if (score < best_score) {
      best_score = score;
      best_index = i;
    }
  }

  if (best_index < surfaces_.size()) {
    auto & existing = surfaces_[best_index];

    const double h_diff_before = hue_distance(existing.median_h, incoming.median_h);
    const double dist_before = center_distance(existing, incoming);

    existing.center_map = incoming.center_map;
    existing.corners_map = incoming.corners_map;

    existing.pixel_area = incoming.pixel_area;
    existing.map_area = incoming.map_area;

    existing.mean_h = incoming.mean_h;
    existing.mean_s = incoming.mean_s;
    existing.mean_v = incoming.mean_v;

    existing.median_h = incoming.median_h;
    existing.median_s = incoming.median_s;
    existing.median_v = incoming.median_v;

    existing.std_h = incoming.std_h;
    existing.std_s = incoming.std_s;
    existing.std_v = incoming.std_v;

    existing.last_seen = incoming.last_seen;
    existing.seen_count += 1;

    if (print_updates_) {
      std::cout
        << "[semantic_color_edge_map_manager] updated "
        << existing.id
        << " seen_count=" << existing.seen_count
        << " hue_diff=" << h_diff_before
        << " center_dist=" << dist_before
        << std::endl;
    }
  } else {
    ManagedSurface added = incoming;
    added.id = make_surface_id();
    surfaces_.push_back(added);

    if (print_updates_) {
      std::cout
        << "[semantic_color_edge_map_manager] added "
        << added.id
        << std::endl;
    }
  }
}

double SemanticColorEdgeMapManager::center_distance(
  const ManagedSurface & a,
  const ManagedSurface & b) const
{
  const double dx = a.center_map.x - b.center_map.x;
  const double dy = a.center_map.y - b.center_map.y;
  const double dz = a.center_map.z - b.center_map.z;

  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double SemanticColorEdgeMapManager::hue_distance(double h1, double h2) const
{
  const double d = std::abs(h1 - h2);
  return std::min(d, 180.0 - d);
}

double SemanticColorEdgeMapManager::relative_area_difference(double a, double b) const
{
  const double denom = std::max(std::abs(a), std::abs(b));
  if (denom < 1e-9) {
    return 0.0;
  }

  return std::abs(a - b) / denom;
}

double SemanticColorEdgeMapManager::compute_polygon_area(
  const std::vector<geometry_msgs::msg::Point> & corners) const
{
  if (corners.size() < 3) {
    return 0.0;
  }

  double area = 0.0;
  const std::size_t n = corners.size();

  for (std::size_t i = 0; i < n; ++i) {
    const auto & p1 = corners[i];
    const auto & p2 = corners[(i + 1) % n];
    area += p1.x * p2.y - p2.x * p1.y;
  }

  return 0.5 * std::abs(area);
}

bool SemanticColorEdgeMapManager::hue_is_reliable(const ManagedSurface & surface) const
{
  return surface.median_s >= min_s_for_hue_match_;
}

bool SemanticColorEdgeMapManager::is_candidate_match(
  const ManagedSurface & existing,
  const ManagedSurface & incoming) const
{
  const double dist = center_distance(existing, incoming);
  const double max_reacquire_distance = merge_distance_ * max_reacquire_distance_factor_;

  if (dist > max_reacquire_distance) {
    return false;
  }

  if (use_map_area_for_merge_) {
    const double area_diff = relative_area_difference(existing.map_area, incoming.map_area);
    if (area_diff > max_area_relative_difference_) {
      return false;
    }
  }

  if (use_hue_for_merge_) {
    const bool existing_reliable = hue_is_reliable(existing);
    const bool incoming_reliable = hue_is_reliable(incoming);

    if (existing_reliable && incoming_reliable) {
      const double h_diff = hue_distance(existing.median_h, incoming.median_h);
      if (h_diff > max_hue_distance_) {
        return false;
      }
    }
  }

  return true;
}

double SemanticColorEdgeMapManager::compute_match_score(
  const ManagedSurface & existing,
  const ManagedSurface & incoming) const
{
  const double dist = center_distance(existing, incoming);
  const double normalized_dist =
    (merge_distance_ > 1e-9) ? (dist / merge_distance_) : dist;

  double area_term = 0.0;
  if (use_map_area_for_merge_) {
    area_term = relative_area_difference(existing.map_area, incoming.map_area);
  }

  double hue_term = 0.0;
  if (use_hue_for_merge_) {
    const bool existing_reliable = hue_is_reliable(existing);
    const bool incoming_reliable = hue_is_reliable(incoming);

    if (existing_reliable && incoming_reliable) {
      hue_term = hue_distance(existing.median_h, incoming.median_h) /
        std::max(max_hue_distance_, 1e-9);
    }
  }

  return 3.0 * hue_term + 1.5 * area_term + 1.0 * normalized_dist;
}

std::string SemanticColorEdgeMapManager::make_surface_id()
{
  std::ostringstream oss;
  oss << "surface_" << next_id_++;
  return oss.str();
}

void SemanticColorEdgeMapManager::sort_by_last_seen()
{
  std::sort(
    surfaces_.begin(),
    surfaces_.end(),
    [](const ManagedSurface & a, const ManagedSurface & b) {
      return a.last_seen < b.last_seen;
    });
}

json SemanticColorEdgeMapManager::point_to_json(const geometry_msgs::msg::Point & p) const
{
  return {
    {"x", p.x},
    {"y", p.y},
    {"z", p.z}
  };
}

geometry_msgs::msg::Point SemanticColorEdgeMapManager::point_from_json(const json & j) const
{
  geometry_msgs::msg::Point p;
  p.x = j.at("x").get<double>();
  p.y = j.at("y").get<double>();
  p.z = j.at("z").get<double>();
  return p;
}

json SemanticColorEdgeMapManager::surface_to_json(const ManagedSurface & surface) const
{
  json j;
  j["id"] = surface.id;
  j["center_map"] = point_to_json(surface.center_map);

  j["corners_map"] = json::array();
  for (const auto & p : surface.corners_map) {
    j["corners_map"].push_back(point_to_json(p));
  }

  j["pixel_area"] = surface.pixel_area;
  j["map_area"] = surface.map_area;

  j["mean_h"] = surface.mean_h;
  j["mean_s"] = surface.mean_s;
  j["mean_v"] = surface.mean_v;

  j["median_h"] = surface.median_h;
  j["median_s"] = surface.median_s;
  j["median_v"] = surface.median_v;

  j["std_h"] = surface.std_h;
  j["std_s"] = surface.std_s;
  j["std_v"] = surface.std_v;

  j["seen_count"] = surface.seen_count;
  j["first_seen_ns"] = surface.first_seen.nanoseconds();
  j["last_seen_ns"] = surface.last_seen.nanoseconds();

  return j;
}

SemanticColorEdgeMapManager::ManagedSurface
SemanticColorEdgeMapManager::surface_from_json(const json & j) const
{
  ManagedSurface surface;

  surface.id = j.at("id").get<std::string>();
  surface.center_map = point_from_json(j.at("center_map"));

  for (const auto & p : j.at("corners_map")) {
    surface.corners_map.push_back(point_from_json(p));
  }

  surface.pixel_area = j.at("pixel_area").get<double>();
  surface.map_area = j.at("map_area").get<double>();

  surface.mean_h = j.at("mean_h").get<double>();
  surface.mean_s = j.at("mean_s").get<double>();
  surface.mean_v = j.at("mean_v").get<double>();

  surface.median_h = j.at("median_h").get<double>();
  surface.median_s = j.at("median_s").get<double>();
  surface.median_v = j.at("median_v").get<double>();

  surface.std_h = j.at("std_h").get<double>();
  surface.std_s = j.at("std_s").get<double>();
  surface.std_v = j.at("std_v").get<double>();

  surface.seen_count = j.at("seen_count").get<std::uint32_t>();
  surface.first_seen = rclcpp::Time(j.at("first_seen_ns").get<std::int64_t>());
  surface.last_seen = rclcpp::Time(j.at("last_seen_ns").get<std::int64_t>());

  return surface;
}