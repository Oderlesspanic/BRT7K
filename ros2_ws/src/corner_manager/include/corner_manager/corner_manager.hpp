#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "interfaces/msg/managed_corner.hpp"
#include "interfaces/msg/valid_color_corner.hpp"
#include "rclcpp/time.hpp"

struct CornerManagerConfig
{
  double merge_distance = 0.20;

  bool use_hsv_for_merge = true;
  double max_hue_distance = 12.0;
  double max_s_difference = 80.0;
  double max_v_difference = 100.0;
  double min_s_for_hue_match = 40.0;

  std::size_t max_corners = 20;

  bool forget_after_time = false;
  double forget_after_seconds = 300.0;
};

class CornerManager
{
public:
  explicit CornerManager(const CornerManagerConfig & config);

  void process_corner(
    const interfaces::msg::ValidColorCorner & corner,
    const rclcpp::Time & stamp);

  void process_corners(
    const std::vector<interfaces::msg::ValidColorCorner> & corners,
    const rclcpp::Time & stamp);

  void remove_expired(const rclcpp::Time & now);
  void enforce_max_corners();

  const std::vector<interfaces::msg::ManagedCorner> & get_corners() const;

private:
  interfaces::msg::ManagedCorner valid_to_managed(
    const interfaces::msg::ValidColorCorner & corner,
    const rclcpp::Time & stamp);

  void merge_or_add(const interfaces::msg::ManagedCorner & incoming);

  std::optional<std::size_t> find_best_match(
    const interfaces::msg::ManagedCorner & incoming) const;

  bool is_candidate_match(
    const interfaces::msg::ManagedCorner & existing,
    const interfaces::msg::ManagedCorner & incoming) const;

  double compute_match_score(
    const interfaces::msg::ManagedCorner & existing,
    const interfaces::msg::ManagedCorner & incoming) const;

  double center_distance(
    const interfaces::msg::ManagedCorner & a,
    const interfaces::msg::ManagedCorner & b) const;

  double hue_distance(double h1, double h2) const;

  bool hue_is_reliable(
    const interfaces::msg::ManagedCorner & corner) const;

  std::string make_corner_uid();

  void update_existing(
    interfaces::msg::ManagedCorner & existing,
    const interfaces::msg::ManagedCorner & incoming);

  CornerManagerConfig config_;
  std::uint64_t next_id_;
  std::vector<interfaces::msg::ManagedCorner> corners_;
};