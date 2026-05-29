#include "object_task_executor/object_task_executor_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectTaskExecutorNode>());
  rclcpp::shutdown();
  return 0;
}
