#include "edge_color/color_wall_projector_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColorWallProjectorNode>());
  rclcpp::shutdown();
  return 0;
}