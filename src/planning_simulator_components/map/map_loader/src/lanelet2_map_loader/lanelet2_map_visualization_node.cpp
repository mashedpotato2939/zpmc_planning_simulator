#include "map_loader/lanelet2_map_visualization_node.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

namespace
{
using Marker = visualization_msgs::msg::Marker;
using Point = geometry_msgs::msg::Point;

Point toPoint(const lanelet::ConstPoint3d & point)
{
  Point result;
  result.x = point.x();
  result.y = point.y();
  result.z = point.z();
  return result;
}

Point interpolate(const Point & start, const Point & end, double ratio)
{
  Point result;
  result.x = start.x + ratio * (end.x - start.x);
  result.y = start.y + ratio * (end.y - start.y);
  result.z = start.z + ratio * (end.z - start.z);
  return result;
}

Point pointAtDistance(
  const std::vector<Point> & points, const std::vector<double> & distances, double distance)
{
  const auto it = std::lower_bound(distances.begin(), distances.end(), distance);
  if (it == distances.begin()) return points.front();
  if (it == distances.end()) return points.back();
  const auto index = static_cast<size_t>(std::distance(distances.begin(), it));
  const double length = distances[index] - distances[index - 1];
  if (length == 0.0) return points[index];
  return interpolate(points[index - 1], points[index], (distance - distances[index - 1]) / length);
}

template <typename LineT>
void addLine(
  visualization_msgs::msg::MarkerArray & markers, const LineT & line, const char * ns,
  int & id, const rclcpp::Time & stamp, double width, float r, float g, float b, float alpha)
{
  if (line.size() < 2) return;
  Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id++;
  marker.type = Marker::LINE_STRIP;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = width;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = alpha;
  for (const auto & point : line) marker.points.push_back(toPoint(point));
  markers.markers.push_back(std::move(marker));
}
}  // namespace

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
  const auto stamp = now();
  for (const auto & ll : map->laneletLayer) {
    const auto center = ll.centerline();
    if (center.size() < 2) continue;

    // Match the ZPMC tile map's colors, widths and marker namespaces.
    addLine(markers, center, "centerline", id, stamp, 0.15, 0.2f, 0.4f, 0.9f, 0.8f);
    addLine(markers, ll.leftBound(), "left_boundary", id, stamp,
      0.08, 0.6f, 0.6f, 0.6f, 0.5f);
    addLine(markers, ll.rightBound(), "right_boundary", id, stamp,
      0.08, 0.6f, 0.6f, 0.6f, 0.5f);

    std::vector<Point> points;
    std::vector<double> distances;
    points.reserve(center.size());
    distances.reserve(center.size());
    for (const auto & point : center) {
      const auto current = toPoint(point);
      if (points.empty()) {
        distances.push_back(0.0);
      } else {
        distances.push_back(
          distances.back() + std::hypot(current.x - points.back().x, current.y - points.back().y));
      }
      points.push_back(current);
    }

    // The tiled map draws a roughly 1 m arrow every 5 m along each centerline.
    for (double distance = 0.0; distance + 1.0 <= distances.back(); distance += 5.0) {
      Marker arrow;
      arrow.header.frame_id = "map";
      arrow.header.stamp = stamp;
      arrow.ns = "lane_direction";
      arrow.id = id++;
      arrow.type = Marker::ARROW;
      arrow.action = Marker::ADD;
      arrow.pose.orientation.w = 1.0;
      arrow.scale.x = 0.5;
      arrow.scale.y = 1.0;
      arrow.scale.z = 1.0;
      arrow.color.r = 0.2f;
      arrow.color.g = 0.4f;
      arrow.color.b = 0.9f;
      arrow.color.a = 0.9f;
      arrow.points.push_back(pointAtDistance(points, distances, distance));
      arrow.points.push_back(pointAtDistance(points, distances, distance + 1.0));
      markers.markers.push_back(std::move(arrow));
    }
  }
  pub_marker_->publish(markers);
}
RCLCPP_COMPONENTS_REGISTER_NODE(Lanelet2MapVisualizationNode)
