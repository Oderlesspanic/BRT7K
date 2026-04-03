#include "rclcpp/rclcpp.hpp"
#include "mmc5603/mmc5603_node.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MMC5603Node>());
    rclcpp::shutdown();
    return 0;
}