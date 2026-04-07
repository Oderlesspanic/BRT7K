
#include "rclcpp/rclcpp.hpp"
#include "edge_color/color_edge_detection_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ColorEdgeDetectionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}