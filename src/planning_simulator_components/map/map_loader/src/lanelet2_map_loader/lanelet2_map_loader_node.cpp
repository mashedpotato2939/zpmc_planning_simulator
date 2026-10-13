#include "map_loader/lanelet2_map_loader_node.hpp"

#include <autoware_lanelet2_extension/io/autoware_osm_parser.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <pugixml.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <limits>

namespace
{
// OSM nodes in this project contain local_x/local_y and share the PCD's map frame.
class LocalProjector : public lanelet::Projector
{
public:
  LocalProjector() : Projector(lanelet::Origin(lanelet::GPSPoint{})) {}
  lanelet::BasicPoint3d forward(const lanelet::GPSPoint & gps) const override
  { return {0.0, 0.0, gps.ele}; }
  lanelet::GPSPoint reverse(const lanelet::BasicPoint3d & point) const override
  { return {0.0, 0.0, point.z()}; }
};

size_t restoreLocalCoordinates(const std::string & file, const lanelet::LaneletMapPtr & map)
{
  pugi::xml_document document;
  if (!document.load_file(file.c_str())) {
    throw std::runtime_error("Could not parse Lanelet2 OSM: " + file);
  }
  size_t restored = 0;
  for (const auto & node : document.child("osm").children("node")) {
    const auto id = static_cast<lanelet::Id>(node.attribute("id").as_llong());
    if (!map->pointLayer.exists(id)) continue;
    bool has_x = false, has_y = false;
    double x = 0.0, y = 0.0, z = 0.0;
    for (const auto & tag : node.children("tag")) {
      const std::string key = tag.attribute("k").as_string();
      if (key == "local_x") { x = tag.attribute("v").as_double(); has_x = true; }
      if (key == "local_y") { y = tag.attribute("v").as_double(); has_y = true; }
      if (key == "local_z") z = tag.attribute("v").as_double();
    }
    if (has_x && has_y) {
      auto point = map->pointLayer.get(id);
      point.x() = x;
      point.y() = y;
      point.z() = z;
      ++restored;
    }
  }
  for (const auto & lane : map->laneletLayer) lane.resetCache();
  return restored;
}
}

Lanelet2MapLoaderNode::Lanelet2MapLoaderNode(const rclcpp::NodeOptions & options)
: Node("lanelet2_map_loader", options)
{
  const auto file = declare_parameter<std::string>("lanelet2_map_path", "");
  const auto resolution = declare_parameter<double>("center_line_resolution", 1.0);
  if (file.empty()) throw std::runtime_error("lanelet2_map_path is empty");
  lanelet::ErrorMessages errors;
  lanelet::LaneletMapPtr map = lanelet::load(file, LocalProjector{}, &errors);
  for (const auto & error : errors) RCLCPP_WARN_STREAM(get_logger(), error);
  if (!map || map->laneletLayer.empty()) throw std::runtime_error("Lanelet2 map has no lanelets: " + file);
  const auto restored = restoreLocalCoordinates(file, map);
  if (restored == 0) throw std::runtime_error("Lanelet2 map has no local_x/local_y coordinates: " + file);
  RCLCPP_INFO(get_logger(), "Restored local coordinates for %zu OSM nodes", restored);
  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -min_x;
  double min_y = min_x;
  double max_y = max_x;
  for (const auto & lane : map->laneletLayer) {
    for (const auto & point : lane.centerline()) {
      min_x = std::min(min_x, point.x()); max_x = std::max(max_x, point.x());
      min_y = std::min(min_y, point.y()); max_y = std::max(max_y, point.y());
    }
  }
  RCLCPP_INFO(get_logger(), "Lanelet2 local bounds: x=[%.1f, %.1f], y=[%.1f, %.1f]",
    min_x, max_x, min_y, max_y);
  // Preserve explicit OSM centerlines when present; fill only missing ones.
  lanelet::utils::overwriteLaneletsCenterline(map, resolution, false);
  autoware_map_msgs::msg::LaneletMapBin msg;
  msg.header.frame_id = "map";
  msg.header.stamp = now();
  lanelet::io_handlers::AutowareOsmParser::parseVersions(
    file, &msg.version_map_format, &msg.version_map);
  lanelet::utils::conversion::toBinMsg(map, &msg);
  pub_map_bin_ = create_publisher<autoware_map_msgs::msg::LaneletMapBin>(
    "output/lanelet2_map", rclcpp::QoS(1).reliable().transient_local());
  pub_map_bin_->publish(msg);
  RCLCPP_INFO(get_logger(), "Published %zu Lanelet2 lanelets from %s", map->laneletLayer.size(), file.c_str());
}
RCLCPP_COMPONENTS_REGISTER_NODE(Lanelet2MapLoaderNode)
