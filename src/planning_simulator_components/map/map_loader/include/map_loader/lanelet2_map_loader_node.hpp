#ifndef MAP_LOADER__LANELET2_MAP_LOADER_NODE_HPP_
#define MAP_LOADER__LANELET2_MAP_LOADER_NODE_HPP_
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <rclcpp/rclcpp.hpp>
class Lanelet2MapLoaderNode : public rclcpp::Node
{
public:
  explicit Lanelet2MapLoaderNode(const rclcpp::NodeOptions & options);
private:
  rclcpp::Publisher<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr pub_map_bin_;
};
#endif
