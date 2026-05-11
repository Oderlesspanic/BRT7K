#include "edge_color/color_corner_validator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

ColorCornerValidator::ColorCornerValidator(
  const ColorCornerValidatorConfig & config)
: config_(config)
{
}

std::optional<interfaces::msg::ValidColorCorner>
ColorCornerValidator::validate(
  const interfaces::msg::ProjectedColorPatch & patch) const
{
  if (!is_on_valid_wall(patch)) {
    return std::nullopt;
  }

  if (!has_valid_a3_size(patch)) {
    return std::nullopt;
  }

  if (!has_valid_color(patch)) {
    return std::nullopt;
  }

  const auto corner_id = determine_corner_id(patch);

  if (!corner_id.has_value()) {
    return std::nullopt;
  }

  interfaces::msg::ValidColorCorner corner;

  corner.corner_id = corner_id.value();
  corner.wall_id = patch.wall_id;

  corner.center_map = patch.center_map;
  corner.corners_map = patch.corners_map;

  corner.map_width = patch.map_width;
  corner.map_height = patch.map_height;
  corner.map_area = patch.map_area;

  corner.mean_h = patch.mean_h;
  corner.mean_s = patch.mean_s;
  corner.mean_v = patch.mean_v;

  corner.median_h = patch.median_h;
  corner.median_s = patch.median_s;
  corner.median_v = patch.median_v;

  corner.std_h = patch.std_h;
  corner.std_s = patch.std_s;
  corner.std_v = patch.std_v;

  corner.validation_score =
    validation_score(patch, corner_id.value());

  return corner;
}

bool ColorCornerValidator::is_on_valid_wall(
  const interfaces::msg::ProjectedColorPatch & patch) const
{
  const double dist = wall_distance(patch);
  return dist <= config_.wall_distance_tolerance;
}

bool ColorCornerValidator::has_valid_a3_size(
  const interfaces::msg::ProjectedColorPatch & patch) const
{
  const double width_error =
    relative_error(patch.map_width, config_.a3_width);

  const double height_error =
    relative_error(patch.map_height, config_.a3_height);

  const bool normal_orientation =
    width_error <= config_.a3_size_tolerance &&
    height_error <= config_.a3_size_tolerance;

  const double width_error_rotated =
    relative_error(patch.map_width, config_.a3_height);

  const double height_error_rotated =
    relative_error(patch.map_height, config_.a3_width);

  const bool rotated_orientation =
    width_error_rotated <= config_.a3_size_tolerance &&
    height_error_rotated <= config_.a3_size_tolerance;

  return normal_orientation || rotated_orientation;
}

bool ColorCornerValidator::has_valid_color(
  const interfaces::msg::ProjectedColorPatch & patch) const
{
  const bool saturated_enough =
    patch.median_s >= config_.min_s_for_color;

  const bool bright_enough =
    patch.median_v >= config_.min_v_for_color;

  return saturated_enough && bright_enough;
}

std::optional<std::string> ColorCornerValidator::determine_corner_id(
  const interfaces::msg::ProjectedColorPatch & patch) const
{
  struct Candidate
  {
    std::string id;
    double x;
    double y;
  };

  const std::vector<Candidate> candidates = {
    {"corner_x_min_y_min", config_.arena_x_min, config_.arena_y_min},
    {"corner_x_min_y_max", config_.arena_x_min, config_.arena_y_max},
    {"corner_x_max_y_min", config_.arena_x_max, config_.arena_y_min},
    {"corner_x_max_y_max", config_.arena_x_max, config_.arena_y_max}
  };

  double best_distance = std::numeric_limits<double>::max();
  std::string best_id;

  for (const auto & candidate : candidates) {
    const double d = distance_2d(
      patch.center_map.x,
      patch.center_map.y,
      candidate.x,
      candidate.y
    );

    if (d < best_distance) {
      best_distance = d;
      best_id = candidate.id;
    }
  }

  if (best_distance > config_.corner_distance_tolerance) {
    return std::nullopt;
  }

  return best_id;
}

double ColorCornerValidator::relative_error(
  double measured,
  double expected) const
{
  if (std::abs(expected) < 1e-9) {
    return 1.0;
  }

  return std::abs(measured - expected) / expected;
}

double ColorCornerValidator::distance_2d(
  double x1,
  double y1,
  double x2,
  double y2) const
{
  const double dx = x1 - x2;
  const double dy = y1 - y2;

  return std::sqrt(dx * dx + dy * dy);
}

double ColorCornerValidator::wall_distance(
  const interfaces::msg::ProjectedColorPatch & patch) const
{
  if (patch.wall_id == "x_min") {
    return std::abs(patch.center_map.x - config_.arena_x_min);
  }

  if (patch.wall_id == "x_max") {
    return std::abs(patch.center_map.x - config_.arena_x_max);
  }

  if (patch.wall_id == "y_min") {
    return std::abs(patch.center_map.y - config_.arena_y_min);
  }

  if (patch.wall_id == "y_max") {
    return std::abs(patch.center_map.y - config_.arena_y_max);
  }

  return std::numeric_limits<double>::max();
}

double ColorCornerValidator::validation_score(
  const interfaces::msg::ProjectedColorPatch & patch,
  const std::string & corner_id) const
{
  double corner_x = 0.0;
  double corner_y = 0.0;

  if (corner_id == "corner_x_min_y_min") {
    corner_x = config_.arena_x_min;
    corner_y = config_.arena_y_min;
  } else if (corner_id == "corner_x_min_y_max") {
    corner_x = config_.arena_x_min;
    corner_y = config_.arena_y_max;
  } else if (corner_id == "corner_x_max_y_min") {
    corner_x = config_.arena_x_max;
    corner_y = config_.arena_y_min;
  } else if (corner_id == "corner_x_max_y_max") {
    corner_x = config_.arena_x_max;
    corner_y = config_.arena_y_max;
  }

  const double d_corner = distance_2d(
    patch.center_map.x,
    patch.center_map.y,
    corner_x,
    corner_y
  );

  const double corner_score =
    std::clamp(
      1.0 - d_corner / std::max(config_.corner_distance_tolerance, 1e-9),
      0.0,
      1.0
    );

  const double wall_score =
    std::clamp(
      1.0 - wall_distance(patch) / std::max(config_.wall_distance_tolerance, 1e-9),
      0.0,
      1.0
    );

  const double width_score =
    std::clamp(
      1.0 - relative_error(patch.map_width, config_.a3_width),
      0.0,
      1.0
    );

  const double height_score =
    std::clamp(
      1.0 - relative_error(patch.map_height, config_.a3_height),
      0.0,
      1.0
    );

  return 0.35 * corner_score +
         0.25 * wall_score +
         0.20 * width_score +
         0.20 * height_score;
}