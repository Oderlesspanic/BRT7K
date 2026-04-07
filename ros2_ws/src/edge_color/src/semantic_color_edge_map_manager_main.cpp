#include "edge_color/semantic_color_edge_map_manager_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SemanticColorEdgeMapManagerNode>());
  rclcpp::shutdown();
  return 0;
}