#include "interface_test.hpp"

using Interface_Test = interface_test::Interface_Test;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Interface_Test>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}