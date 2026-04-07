#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "interfaces/msg/mapped_color_patch.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"

class SemanticColorEdgeMapManager
{
public:
  struct ManagedSurface
  {
    std::string id;

    geometry_msgs::msg::Point center_map;
    std::vector<geometry_msgs::msg::Point> corners_map;

    double pixel_area;
    double map_area;

    double mean_h;
    double mean_s;
    double mean_v;

    double median_h;
    double median_s;
    double median_v;

    double std_h;
    double std_s;
    double std_v;

    std::uint32_t seen_count;

    rclcpp::Time first_seen;
    rclcpp::Time last_seen;
  };

  SemanticColorEdgeMapManager();

  void configure(
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
    double max_area_relative_difference);

  void process_patch(
    const interfaces::msg::MappedColorPatch & patch,
    const rclcpp::Time & stamp);

  void process_patches(
    const std::vector<interfaces::msg::MappedColorPatch> & patches,
    const rclcpp::Time & stamp);

  void remove_expired(const rclcpp::Time & now);
  void enforce_max_objects();

  bool save_to_json() const;
  bool load_from_json();

  nlohmann::json to_json() const;

  const std::vector<ManagedSurface> & get_surfaces() const;

private:
  ManagedSurface patch_to_surface(
    const interfaces::msg::MappedColorPatch & patch,
    const rclcpp::Time & stamp) const;

  void merge_or_add(const ManagedSurface & incoming);

  double center_distance(
    const ManagedSurface & a,
    const ManagedSurface & b) const;

  double hue_distance(double h1, double h2) const;
  double relative_area_difference(double a, double b) const;
  double compute_polygon_area(const std::vector<geometry_msgs::msg::Point> & corners) const;

  bool hue_is_reliable(const ManagedSurface & surface) const;
  bool is_candidate_match(
    const ManagedSurface & existing,
    const ManagedSurface & incoming) const;

  double compute_match_score(
    const ManagedSurface & existing,
    const ManagedSurface & incoming) const;

  std::string make_surface_id();
  void sort_by_last_seen();

  nlohmann::json point_to_json(const geometry_msgs::msg::Point & p) const;
  geometry_msgs::msg::Point point_from_json(const nlohmann::json & j) const;

  nlohmann::json surface_to_json(const ManagedSurface & surface) const;
  ManagedSurface surface_from_json(const nlohmann::json & j) const;

  double merge_distance_;
  double max_reacquire_distance_factor_;

  bool save_to_file_;
  std::string save_path_;
  std::size_t max_objects_;
  bool forget_after_time_;
  double forget_after_seconds_;
  bool print_updates_;

  bool use_hue_for_merge_;
  double max_hue_distance_;
  double min_s_for_hue_match_;

  bool use_map_area_for_merge_;
  double max_area_relative_difference_;

  std::uint64_t next_id_;
  std::vector<ManagedSurface> surfaces_;
};
