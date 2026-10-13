// Copyright 2024 ZPMC
// Licensed under the Apache License, Version 2.0

#include "zpmc_map_loader/zpmc_map_loader_node.hpp"

#include <hdmap/lane.h>
#include <hdmap/lane_graph.h>

namespace zpmc_map_loader
{

ZpmcMapLoaderNode::ZpmcMapLoaderNode(const rclcpp::NodeOptions & options)
: Node("zpmc_map_loader", options)
{
  tile_directory_ = declare_parameter<std::string>("tile_directory", "");
  tile_size_ = static_cast<float>(declare_parameter<double>("tile_size", 300.0));
  frame_id_ = declare_parameter<std::string>("frame_id", "map");

  if (tile_directory_.empty()) {
    RCLCPP_ERROR(get_logger(), "tile_directory parameter is required");
    throw std::runtime_error("tile_directory parameter is required");
  }

  // Load map — set large load_radius to load ALL tiles at startup (port-scale)
  hd_map_ = std::make_unique<hdmap::HdMap>();
  if (!hd_map_->load(tile_directory_, tile_size_)) {
    RCLCPP_ERROR(get_logger(), "Failed to load HD Map from: %s", tile_directory_.c_str());
    throw std::runtime_error("Failed to load HD Map from: " + tile_directory_);
  }

  // Trigger loading of all tiles by updating position at map center
  // Use a large radius to ensure all tiles are loaded
  hd_map_->updatePosition(0.0, 0.0);

  RCLCPP_INFO(get_logger(), "HD Map loaded from: %s (tile_size=%.0f)", tile_directory_.c_str(),
    tile_size_);

  // Publishers
  rclcpp::QoS map_qos(1);
  map_qos.reliable();
  map_qos.transient_local();

  map_info_pub_ = create_publisher<zpmc_map_msgs::msg::ZpmcMapInfo>(
    "/map/zpmc_map_info", map_qos);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/map/vector_map_marker_humble", map_qos);

  publishMapInfo();
  publishVisualization();

  RCLCPP_INFO(get_logger(), "Published map info and visualization markers");
}

void ZpmcMapLoaderNode::publishMapInfo()
{
  zpmc_map_msgs::msg::ZpmcMapInfo msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = frame_id_;
  msg.tile_directory = tile_directory_;
  msg.tile_size = tile_size_;
  msg.frame_id = frame_id_;

  // Extract metadata from map header if available
  const auto * header = hd_map_->tileManager().mapHeader();
  if (header) {
    msg.origin_lat = header->origin_lat();
    msg.origin_lon = header->origin_lon();
    msg.origin_alt = header->origin_alt();
    msg.tile_count = header->tiles() ? header->tiles()->size() : 0;
  }

  // Count lanes from the lane graph snapshot
  const auto & graph = hd_map_->laneGraph();
  const auto * snapshot = graph.getSnapshot();
  msg.lane_count = snapshot ? static_cast<uint32_t>(snapshot->lanes.size()) : 0;

  map_info_pub_->publish(msg);
}

void ZpmcMapLoaderNode::publishVisualization()
{
  auto markers = createLaneMarkers();
  if (!markers.markers.empty()) {
    marker_pub_->publish(markers);
    RCLCPP_INFO(get_logger(), "Published %zu visualization markers", markers.markers.size());
  }
}

visualization_msgs::msg::MarkerArray ZpmcMapLoaderNode::createLaneMarkers()
{
  visualization_msgs::msg::MarkerArray marker_array;
  int marker_id = 0;

  const auto & graph = hd_map_->laneGraph();
  const auto * snapshot = graph.getSnapshot();
  if (!snapshot) return marker_array;

  // Iterate all lanes in the snapshot
  for (const auto & [lane_id, lane] : snapshot->lanes) {
    if (!lane.valid()) continue;

    auto cl = lane.centerline();
    auto lb = lane.leftBoundary();
    auto rb = lane.rightBoundary();

    // Centerline marker
    if (cl.size() > 1) {
      visualization_msgs::msg::Marker centerline_marker;
      centerline_marker.header.frame_id = frame_id_;
      centerline_marker.header.stamp = this->now();
      centerline_marker.ns = "centerline";
      centerline_marker.id = marker_id++;
      centerline_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      centerline_marker.action = visualization_msgs::msg::Marker::ADD;
      centerline_marker.scale.x = 0.15;
      centerline_marker.pose.orientation.w = 1.0;

      // Color by lane type
      if (lane.isReverse()) {
        centerline_marker.color.r = 0.9f;
        centerline_marker.color.g = 0.2f;
        centerline_marker.color.b = 0.2f;
      } else {
        centerline_marker.color.r = 0.2f;
        centerline_marker.color.g = 0.4f;
        centerline_marker.color.b = 0.9f;
      }
      centerline_marker.color.a = 0.8f;

      // Subsample for visualization (every 1m = ~7 points at 0.15m resolution)
      for (uint32_t j = 0; j < cl.size(); j += 7) {
        geometry_msgs::msg::Point pt;
        pt.x = cl[j].x;
        pt.y = cl[j].y;
        pt.z = cl[j].z;
        centerline_marker.points.push_back(pt);
      }
      // Always include last point
      if (cl.size() > 0) {
        geometry_msgs::msg::Point pt;
        pt.x = cl[cl.size() - 1].x;
        pt.y = cl[cl.size() - 1].y;
        pt.z = cl[cl.size() - 1].z;
        centerline_marker.points.push_back(pt);
      }

      marker_array.markers.push_back(centerline_marker);
    }

    // Left boundary marker
    if (lb.size() > 1) {
      visualization_msgs::msg::Marker left_marker;
      left_marker.header.frame_id = frame_id_;
      left_marker.header.stamp = this->now();
      left_marker.ns = "left_boundary";
      left_marker.id = marker_id++;
      left_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      left_marker.action = visualization_msgs::msg::Marker::ADD;
      left_marker.scale.x = 0.08;
      left_marker.pose.orientation.w = 1.0;
      left_marker.color.r = 0.6f;
      left_marker.color.g = 0.6f;
      left_marker.color.b = 0.6f;
      left_marker.color.a = 0.5f;

      for (uint32_t j = 0; j < lb.size(); j += 7) {
        geometry_msgs::msg::Point pt;
        pt.x = lb[j].x;
        pt.y = lb[j].y;
        pt.z = lb[j].z;
        left_marker.points.push_back(pt);
      }
      if (lb.size() > 0) {
        geometry_msgs::msg::Point pt;
        pt.x = lb[lb.size() - 1].x;
        pt.y = lb[lb.size() - 1].y;
        pt.z = lb[lb.size() - 1].z;
        left_marker.points.push_back(pt);
      }

      marker_array.markers.push_back(left_marker);
    }

    // Right boundary marker
    if (rb.size() > 1) {
      visualization_msgs::msg::Marker right_marker;
      right_marker.header.frame_id = frame_id_;
      right_marker.header.stamp = this->now();
      right_marker.ns = "right_boundary";
      right_marker.id = marker_id++;
      right_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      right_marker.action = visualization_msgs::msg::Marker::ADD;
      right_marker.scale.x = 0.08;
      right_marker.pose.orientation.w = 1.0;
      right_marker.color.r = 0.6f;
      right_marker.color.g = 0.6f;
      right_marker.color.b = 0.6f;
      right_marker.color.a = 0.5f;

      for (uint32_t j = 0; j < rb.size(); j += 7) {
        geometry_msgs::msg::Point pt;
        pt.x = rb[j].x;
        pt.y = rb[j].y;
        pt.z = rb[j].z;
        right_marker.points.push_back(pt);
      }
      if (rb.size() > 0) {
        geometry_msgs::msg::Point pt;
        pt.x = rb[rb.size() - 1].x;
        pt.y = rb[rb.size() - 1].y;
        pt.z = rb[rb.size() - 1].z;
        right_marker.points.push_back(pt);
      }

      marker_array.markers.push_back(right_marker);
    }

    // Lane direction arrows along centerline (~5 m spacing)
    if (cl.size() > 8) {
      const uint32_t step = 35;  // ~5 m at 0.15 m resolution
      for (uint32_t j = 0; j + 7 < cl.size(); j += step) {
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = frame_id_;
        arrow.header.stamp = this->now();
        arrow.ns = "lane_direction";
        arrow.id = marker_id++;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.scale.x = 0.5;  // shaft diameter
        arrow.scale.y = 1.00;  // head diameter
        arrow.scale.z = 1.00;  // head length
        arrow.pose.orientation.w = 1.0;
        if (lane.isReverse()) {
          arrow.color.r = 0.9f;
          arrow.color.g = 0.2f;
          arrow.color.b = 0.2f;
        } else {
          arrow.color.r = 0.2f;
          arrow.color.g = 0.4f;
          arrow.color.b = 0.9f;
        }
        arrow.color.a = 0.9f;

        geometry_msgs::msg::Point p0, p1;
        p0.x = cl[j].x;       p0.y = cl[j].y;       p0.z = cl[j].z;
        p1.x = cl[j + 7].x;   p1.y = cl[j + 7].y;   p1.z = cl[j + 7].z;
        arrow.points.push_back(p0);
        arrow.points.push_back(p1);

        marker_array.markers.push_back(arrow);
      }
    }
  }

  return marker_array;
}

}  // namespace zpmc_map_loader

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(zpmc_map_loader::ZpmcMapLoaderNode)
