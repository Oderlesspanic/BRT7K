#include "edge_color/color_corner_validator_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColorCornerValidatorNode>());
  rclcpp::shutdown();
  return 0;
}