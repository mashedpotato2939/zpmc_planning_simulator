// Copyright 2024 ZPMC
// Licensed under the Apache License, Version 2.0

#ifndef ZPMC_MAP_LOADER__ZPMC_MAP_CLIENT_HPP_
#define ZPMC_MAP_LOADER__ZPMC_MAP_CLIENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <hdmap/hd_map.h>
#include <zpmc_map_msgs/msg/zpmc_map_info.hpp>

#include <functional>
#include <memory>
#include <string>

namespace zpmc_map_loader
{

/// Reusable client for downstream nodes that need ZPMC HD Map access.
/// Subscribes to /map/zpmc_map_info, loads HdMap from the published tile path.
class ZpmcMapClient
{
public:
  using MapReadyCallback = std::function<void(hdmap::HdMap &)>;

  /// Create client and subscribe to map info topic.
  /// @param node  The host ROS2 node (for subscription creation)
  /// @param on_map_ready  Callback invoked once when the map is loaded
  /// @param topic  Topic name (default: /map/zpmc_map_info)
  ZpmcMapClient(
    rclcpp::Node & node,
    MapReadyCallback on_map_ready,
    const std::string & topic = "/map/zpmc_map_info")
  : on_map_ready_(on_map_ready),
    logger_(node.get_logger().get_child("zpmc_map_client"))
  {
    rclcpp::QoS qos(1);
    qos.reliable();
    qos.transient_local();

    sub_ = node.create_subscription<zpmc_map_msgs::msg::ZpmcMapInfo>(
      topic, qos,
      [this](const zpmc_map_msgs::msg::ZpmcMapInfo::SharedPtr msg) {
        onMapInfo(msg);
      });
  }

  bool isMapLoaded() const { return loaded_; }

  hdmap::HdMap & map()
  {
    if (!hd_map_) {
      throw std::runtime_error("ZpmcMapClient: map not loaded yet");
    }
    return *hd_map_;
  }

  const hdmap::HdMap & map() const
  {
    if (!hd_map_) {
      throw std::runtime_error("ZpmcMapClient: map not loaded yet");
    }
    return *hd_map_;
  }

  /// Get tile directory path (available after map is loaded)
  const std::string & tileDirectory() const { return tile_directory_; }

private:
  void onMapInfo(const zpmc_map_msgs::msg::ZpmcMapInfo::SharedPtr msg)
  {
    if (loaded_) {
      return;  // Already loaded, ignore subsequent messages
    }

    if (msg->tile_directory.empty()) {
      RCLCPP_WARN(logger_, "Received ZpmcMapInfo with empty tile_directory");
      return;
    }

    tile_directory_ = msg->tile_directory;
    float tile_size = msg->tile_size > 0.0f ? msg->tile_size : 300.0f;

    RCLCPP_INFO(logger_, "Loading HD Map from: %s (tile_size=%.0f)",
      tile_directory_.c_str(), tile_size);

    hd_map_ = std::make_unique<hdmap::HdMap>();
    if (!hd_map_->load(tile_directory_, tile_size)) {
      RCLCPP_ERROR(logger_, "Failed to load HD Map from: %s", tile_directory_.c_str());
      hd_map_.reset();
      return;
    }

    // Load all tiles (port-scale: position doesn't matter, radius covers everything)
    hd_map_->updatePosition(0.0, 0.0);

    loaded_ = true;
    RCLCPP_INFO(logger_, "HD Map loaded successfully (%u tiles, %u lanes)",
      msg->tile_count, msg->lane_count);

    if (on_map_ready_) {
      on_map_ready_(*hd_map_);
    }
  }

  rclcpp::Subscription<zpmc_map_msgs::msg::ZpmcMapInfo>::SharedPtr sub_;
  std::unique_ptr<hdmap::HdMap> hd_map_;
  MapReadyCallback on_map_ready_;
  rclcpp::Logger logger_;
  bool loaded_{false};
  std::string tile_directory_;
};

}  // namespace zpmc_map_loader

#endif  // ZPMC_MAP_LOADER__ZPMC_MAP_CLIENT_HPP_
