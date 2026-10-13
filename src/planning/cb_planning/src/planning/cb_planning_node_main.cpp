#include "cb_planning_node.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    const auto wait_period = std::chrono::milliseconds(2000);
    rclcpp::sleep_for(wait_period);
    auto node = std::make_shared<cb_planning::CrabWalking_Planning>(rclcpp::NodeOptions());

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}