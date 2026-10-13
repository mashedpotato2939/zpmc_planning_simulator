// Copyright 2026 ZPMC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "igv_velocity_planner/velocity_shaper/diagnostics.hpp"

#include "igv_velocity_planner/velocity_shaper/geometry_utils.hpp"

#include <visualization_msgs/msg/marker.hpp>

#include <cstdio>
#include <string>

namespace igv_velocity_planner::velocity_shaper
{

namespace
{
using visualization_msgs::msg::Marker;

constexpr int32_t kMarkersPerStop = 10;
constexpr int32_t kWallIdOffset = 0;
constexpr int32_t kSphereIdOffset = 1;
constexpr int32_t kConnectorIdOffset = 2;
constexpr int32_t kTextIdOffset = 3;

std::string makeHardStopText(const HardStopMarkerData & data)
{
  char distance_line[64];
  std::snprintf(
    distance_line, sizeof(distance_line), "stop distance=%.2fm", data.stop_distance);

  std::string text = "STOP\n";
  text += hardStopReasonText(data.type);
  text += "\n";
  text += distance_line;

  if (data.anchor) {
    const double gap = distance2D(data.stop_pose.position, *data.anchor);
    char gap_line[64];
    std::snprintf(gap_line, sizeof(gap_line), "obstacle gap=%.2fm", gap);
    text += "\n";
    text += gap_line;
  }

  return text;
}
}  // namespace

bool isHardStopType(const ConstraintType type)
{
  switch (type) {
    case ConstraintType::kOccupancyStop:
    case ConstraintType::kStaticObstacleStop:
    case ConstraintType::kDynamicStop:
      return true;
    default:
      return false;
  }
}

const char * hardStopReasonText(const ConstraintType type)
{
  switch (type) {
    case ConstraintType::kOccupancyStop:
      return "OccupancyStop";
    case ConstraintType::kStaticObstacleStop:
      return "StaticObstacleStop";
    case ConstraintType::kDynamicStop:
      return "DynamicStop";
    default:
      return "ObstacleStop";
  }
}

const char * hardStopVelocityFactorSequence(const ConstraintType type)
{
  switch (type) {
    case ConstraintType::kOccupancyStop:
      return "fms_velocity_shaper.occupancy_grid";
    case ConstraintType::kStaticObstacleStop:
      return "fms_velocity_shaper.static_obstacle_stop";
    case ConstraintType::kDynamicStop:
      return "fms_velocity_shaper.dynamic_obstacle_stop";
    default:
      return "fms_velocity_shaper.stop";
  }
}

void appendHardStopMarkers(
  visualization_msgs::msg::MarkerArray & markers, const std_msgs::msg::Header & header,
  const HardStopMarkerData & data, const int32_t marker_group_id)
{
  const int32_t base_id = marker_group_id * kMarkersPerStop;

  Marker wall;
  wall.header = header;
  wall.ns = "fms_velocity_shaper/stop_wall";
  wall.id = base_id + kWallIdOffset;
  wall.type = Marker::CUBE;
  wall.action = Marker::ADD;
  wall.pose = data.stop_pose;
  wall.pose.position.z += 1.0;
  wall.scale.x = 0.1;
  wall.scale.y = 5.0;
  wall.scale.z = 2.0;
  wall.color.r = 1.0f;
  wall.color.g = 0.0f;
  wall.color.b = 0.0f;
  wall.color.a = 0.5f;
  markers.markers.push_back(wall);

  Marker sphere;
  sphere.header = header;
  sphere.ns = "fms_velocity_shaper/stop_point";
  sphere.id = base_id + kSphereIdOffset;
  sphere.type = Marker::SPHERE;
  sphere.action = Marker::ADD;
  sphere.pose = data.stop_pose;
  sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.6;
  sphere.color.r = 1.0f;
  sphere.color.g = 0.1f;
  sphere.color.b = 0.1f;
  sphere.color.a = 0.9f;
  markers.markers.push_back(sphere);

  if (data.anchor) {
    Marker line;
    line.header = header;
    line.ns = "fms_velocity_shaper/stop_to_obj";
    line.id = base_id + kConnectorIdOffset;
    line.type = Marker::LINE_STRIP;
    line.action = Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.1;
    line.color.r = 1.0f;
    line.color.g = 1.0f;
    line.color.b = 0.0f;
    line.color.a = 0.9f;
    line.points.push_back(data.stop_pose.position);
    line.points.push_back(*data.anchor);
    markers.markers.push_back(line);
  }

  Marker text;
  text.header = header;
  text.ns = "fms_velocity_shaper/stop_text";
  text.id = base_id + kTextIdOffset;
  text.type = Marker::TEXT_VIEW_FACING;
  text.action = Marker::ADD;
  text.pose = data.stop_pose;
  text.pose.position.z += 2.5;
  text.scale.z = 0.45;
  text.color.r = 1.0f;
  text.color.g = 1.0f;
  text.color.b = 1.0f;
  text.color.a = 1.0f;
  text.text = makeHardStopText(data);
  markers.markers.push_back(text);
}

}  // namespace igv_velocity_planner::velocity_shaper
