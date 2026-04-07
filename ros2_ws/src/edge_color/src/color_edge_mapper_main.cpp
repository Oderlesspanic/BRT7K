#include "rclcpp/rclcpp.hpp"
#include "edge_color/color_edge_mapper_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColorEdgeMapperNode>());
  rclcpp::shutdown();
  return 0;
}