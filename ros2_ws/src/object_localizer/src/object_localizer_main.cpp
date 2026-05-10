#include "object_localizer/object_localizer_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectLocalizerNode>());
  rclcpp::shutdown();
  return 0;
}