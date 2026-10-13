// Copyright 2024 ZPMC
// Licensed under the Apache License, Version 2.0

#ifndef ZPMC_MAP_LOADER__ZPMC_MAP_LOADER_NODE_HPP_
#define ZPMC_MAP_LOADER__ZPMC_MAP_LOADER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <hdmap/hd_map.h>

#include <zpmc_map_msgs/msg/zpmc_map_info.hpp>

#include <memory>
#include <string>

namespace zpmc_map_loader
{

class ZpmcMapLoaderNode : public rclcpp::Node
{
public:
  explicit ZpmcMapLoaderNode(const rclcpp::NodeOptions & options);

private:
  void publishMapInfo();
  void publishVisualization();

  visualization_msgs::msg::MarkerArray createLaneMarkers();

  std::unique_ptr<hdmap::HdMap> hd_map_;

  rclcpp::Publisher<zpmc_map_msgs::msg::ZpmcMapInfo>::SharedPtr map_info_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::string tile_directory_;
  float tile_size_;
  std::string frame_id_;
};

}  // namespace zpmc_map_loader

#endif  // ZPMC_MAP_LOADER__ZPMC_MAP_LOADER_NODE_HPP_
