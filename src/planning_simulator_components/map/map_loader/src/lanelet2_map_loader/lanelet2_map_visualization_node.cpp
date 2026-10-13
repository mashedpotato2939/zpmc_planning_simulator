#include "map_loader/lanelet2_map_visualization_node.hpp"
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <functional>

Lanelet2MapVisualizationNode::Lanelet2MapVisualizationNode(const rclcpp::NodeOptions & options)
: Node("lanelet2_map_visualization", options)
{
  pub_marker_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "output/lanelet2_map_marker", rclcpp::QoS(1).reliable().transient_local());
  sub_map_bin_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "input/lanelet2_map", rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&Lanelet2MapVisualizationNode::onMapBin, this, std::placeholders::_1));
}

void Lanelet2MapVisualizationNode::onMapBin(
  autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, map);
  visualization_msgs::msg::MarkerArray markers;
  int id = 0;
  for (const auto & ll : map->laneletLayer) {
    const auto add_line = [&markers, &id, this](const auto & line, const char * ns,
      float r, float g, float b, float width) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = now();
      marker.ns = ns;
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = width;
      marker.color.r = r; marker.color.g = g; marker.color.b = b; marker.color.a = 1.0;
      for (const auto & p : line) {
        geometry_msgs::msg::Point point;
        point.x = p.x(); point.y = p.y(); point.z = p.z();
        marker.points.push_back(point);
      }
      markers.markers.push_back(std::move(marker));
    };
    add_line(ll.leftBound(), "left_boundary", 0.2f, 0.8f, 1.0f, 0.12f);
    add_line(ll.rightBound(), "right_boundary", 0.2f, 0.8f, 1.0f, 0.12f);
    add_line(ll.centerline(), "centerline", 1.0f, 0.8f, 0.1f, 0.08f);
  }
  pub_marker_->publish(markers);
}
RCLCPP_COMPONENTS_REGISTER_NODE(Lanelet2MapVisualizationNode)
