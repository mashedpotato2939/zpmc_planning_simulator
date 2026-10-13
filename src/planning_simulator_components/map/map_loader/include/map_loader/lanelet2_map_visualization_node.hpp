#ifndef MAP_LOADER__LANELET2_MAP_VISUALIZATION_NODE_HPP_
#define MAP_LOADER__LANELET2_MAP_VISUALIZATION_NODE_HPP_
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
class Lanelet2MapVisualizationNode : public rclcpp::Node
{
public:
  explicit Lanelet2MapVisualizationNode(const rclcpp::NodeOptions & options);
private:
  void onMapBin(autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg);
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr sub_map_bin_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
};
#endif
