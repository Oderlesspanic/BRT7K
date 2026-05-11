#pragma once

#include <optional>
#include <string>
#include <vector>

#include "interfaces/msg/projected_color_patch.hpp"
#include "interfaces/msg/valid_color_corner.hpp"

struct ColorCornerValidatorConfig
{
    double arena_x_min = -1.5;
    double arena_x_max = 1.5;
    double arena_y_min = -1.0;
    double arena_y_max = 1.0;

    double wall_distance_tolerance = 0.08;
    double corner_distance_tolerance = 0.35;

    double a3_width = 0.297;
    double a3_height = 0.420;
    double a3_size_tolerance = 0.35;

    double min_s_for_color = 40.0;
    double min_v_for_color = 50.0;
};

class ColorCornerValidator
{
public:
  explicit ColorCornerValidator(const ColorCornerValidatorConfig & config);

  std::optional<interfaces::msg::ValidColorCorner> validate(
    const interfaces::msg::ProjectedColorPatch & patch) const;

private:
  bool is_on_valid_wall(
    const interfaces::msg::ProjectedColorPatch & patch) const;

  bool has_valid_a3_size(
    const interfaces::msg::ProjectedColorPatch & patch) const;

  bool has_valid_color(
    const interfaces::msg::ProjectedColorPatch & patch) const;

  std::optional<std::string> determine_corner_id(
    const interfaces::msg::ProjectedColorPatch & patch) const;


  double relative_error(double measured, double expected) const;

  double distance_2d(
    double x1,
    double y1,
    double x2,
    double y2) const;

  double wall_distance(
    const interfaces::msg::ProjectedColorPatch & patch) const;

  double validation_score(
    const interfaces::msg::ProjectedColorPatch & patch,
    const std::string & corner_id) const;

  ColorCornerValidatorConfig config_;
};