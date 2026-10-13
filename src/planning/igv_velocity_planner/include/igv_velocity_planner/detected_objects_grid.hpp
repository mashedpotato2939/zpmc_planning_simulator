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

#ifndef FMS_TRAJECTORY_BRIDGE__DETECTED_OBJECTS_GRID_HPP_
#define FMS_TRAJECTORY_BRIDGE__DETECTED_OBJECTS_GRID_HPP_

#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <cstdint>
#include <string>
#include <vector>

// Pure (rclcpp-free) helpers that rasterize DetectedObjects into a
// nav_msgs/OccupancyGrid. The companion node (detected_objects_grid_node) owns
// the subscriptions, TF, the recent-detections hold buffer, and the publish
// timer; everything here is deterministic and unit-testable in isolation.
//
// The consumer — fms_velocity_shaper's applyOccupancyGridStopCheck — samples
// only the path *centerline* cell per point and does not account for ego width.
// Callers must therefore pass an `inflation` of at least the ego half-width so a
// laterally-offset obstacle still lands under the centerline.
namespace igv_velocity_planner::detected_objects_grid
{

// Build a square, axis-aligned (identity orientation) grid centered on
// (ego_x, ego_y): side = length_m, `resolution` m/cell, origin at the
// lower-left corner, all cells free (0). `frame` becomes header.frame_id.
nav_msgs::msg::OccupancyGrid makeEgoCenteredGrid(
  double ego_x, double ego_y, const std::string & frame, double resolution, double length_m);

// Mark every cell whose center lies inside the rotated rectangle centered at
// `pose` with half-extents (0.5*dim_x + inflation, 0.5*dim_y + inflation),
// rotated by the pose yaw. Writes max(existing, value). No-op on a degenerate
// grid. `dim_x`/`dim_y` are the object's full length/width.
void markBoundingBox(
  nav_msgs::msg::OccupancyGrid & grid, const geometry_msgs::msg::Pose & pose,
  double dim_x, double dim_y, double inflation, int8_t value);

// Mark every cell whose center lies within radius (0.5*dim_x + inflation) of
// `pose` position. Writes max(existing, value).
void markCylinder(
  nav_msgs::msg::OccupancyGrid & grid, const geometry_msgs::msg::Pose & pose,
  double dim_x, double inflation, int8_t value);

// Rasterize a single object's footprint at `pose_in_grid` (already in the grid
// frame). Dispatches on object.shape.type: BOUNDING_BOX -> markBoundingBox,
// CYLINDER -> markCylinder, POLYGON -> conservative bounding-box from
// shape.dimensions (footprint polygons are rare for the close-range detections
// this path targets; the AABB fallback is safe — it never under-marks).
void markObjectFootprint(
  nav_msgs::msg::OccupancyGrid & grid,
  const autoware_perception_msgs::msg::DetectedObject & object,
  const geometry_msgs::msg::Pose & pose_in_grid, double inflation, int8_t value);

// Index of the first entry in the ascending-sorted `stamps_s` that is still
// within `hold_s` of `now_s` (i.e. stamps_s[i] >= now_s - hold_s). Entries
// before it have expired and should be dropped from the hold buffer. Returns
// stamps_s.size() when every entry has expired.
size_t firstRetainedIndex(const std::vector<double> & stamps_s, double now_s, double hold_s);

}  // namespace igv_velocity_planner::detected_objects_grid

#endif  // FMS_TRAJECTORY_BRIDGE__DETECTED_OBJECTS_GRID_HPP_
