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

#include "igv_velocity_planner/detected_objects_grid.hpp"

#include <algorithm>
#include <cmath>

namespace igv_velocity_planner::detected_objects_grid
{

namespace
{

double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

// World (grid-frame) coordinates of cell-center (cx, cy). The grid is
// axis-aligned with identity orientation, so this is a pure translation.
inline void cellCenter(
  const nav_msgs::msg::OccupancyGrid & grid, int64_t cx, int64_t cy, double & wx, double & wy)
{
  wx = grid.info.origin.position.x + (static_cast<double>(cx) + 0.5) * grid.info.resolution;
  wy = grid.info.origin.position.y + (static_cast<double>(cy) + 0.5) * grid.info.resolution;
}

inline void setCellMax(
  nav_msgs::msg::OccupancyGrid & grid, int64_t cx, int64_t cy, int8_t value)
{
  const size_t idx = static_cast<size_t>(cy) * grid.info.width + static_cast<size_t>(cx);
  if (value > grid.data[idx]) grid.data[idx] = value;
}

// Iterate the cells whose centers fall in the world AABB [wx-r, wx+r] x
// [wy-r, wy+r], clamped to the grid, calling `mark(cx, cy, world_x, world_y)`
// for each. Centralizes the bounds math the box/cylinder fills share.
template <typename F>
void forEachCellInRadius(
  nav_msgs::msg::OccupancyGrid & grid, double wx, double wy, double r, F && mark)
{
  const double res = grid.info.resolution;
  if (res <= 0.0) return;
  const auto w = static_cast<int64_t>(grid.info.width);
  const auto h = static_cast<int64_t>(grid.info.height);
  const double ox = grid.info.origin.position.x;
  const double oy = grid.info.origin.position.y;

  // Cell index range covering [wx-r, wx+r]; cell centers are tested by callers.
  int64_t cx_min = static_cast<int64_t>(std::floor((wx - r - ox) / res));
  int64_t cx_max = static_cast<int64_t>(std::floor((wx + r - ox) / res));
  int64_t cy_min = static_cast<int64_t>(std::floor((wy - r - oy) / res));
  int64_t cy_max = static_cast<int64_t>(std::floor((wy + r - oy) / res));
  cx_min = std::max<int64_t>(cx_min, 0);
  cy_min = std::max<int64_t>(cy_min, 0);
  cx_max = std::min<int64_t>(cx_max, w - 1);
  cy_max = std::min<int64_t>(cy_max, h - 1);

  for (int64_t cy = cy_min; cy <= cy_max; ++cy) {
    for (int64_t cx = cx_min; cx <= cx_max; ++cx) {
      double cwx = 0.0;
      double cwy = 0.0;
      cellCenter(grid, cx, cy, cwx, cwy);
      mark(cx, cy, cwx, cwy);
    }
  }
}

}  // namespace

nav_msgs::msg::OccupancyGrid makeEgoCenteredGrid(
  double ego_x, double ego_y, const std::string & frame, double resolution, double length_m)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = frame;
  grid.info.resolution = resolution;
  const auto cells =
    (resolution > 0.0) ? static_cast<uint32_t>(std::llround(length_m / resolution)) : 0u;
  grid.info.width = cells;
  grid.info.height = cells;
  grid.info.origin.position.x = ego_x - 0.5 * length_m;
  grid.info.origin.position.y = ego_y - 0.5 * length_m;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<size_t>(cells) * cells, 0);
  return grid;
}

void markBoundingBox(
  nav_msgs::msg::OccupancyGrid & grid, const geometry_msgs::msg::Pose & pose,
  double dim_x, double dim_y, double inflation, int8_t value)
{
  if (grid.data.empty() || grid.info.resolution <= 0.0) return;
  const double hx = 0.5 * std::max(0.0, dim_x) + inflation;
  const double hy = 0.5 * std::max(0.0, dim_y) + inflation;
  if (hx <= 0.0 || hy <= 0.0) return;

  const double px = pose.position.x;
  const double py = pose.position.y;
  const double yaw = yawFromQuat(pose.orientation);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  // Circumscribing radius of the rectangle bounds the cell search window.
  const double r = std::hypot(hx, hy);

  forEachCellInRadius(grid, px, py, r, [&](int64_t cx, int64_t cy, double wx, double wy) {
    // Transform the cell center into the object frame and test the rectangle.
    const double dx = wx - px;
    const double dy = wy - py;
    const double rx = c * dx + s * dy;
    const double ry = -s * dx + c * dy;
    if (std::abs(rx) <= hx && std::abs(ry) <= hy) setCellMax(grid, cx, cy, value);
  });
}

void markCylinder(
  nav_msgs::msg::OccupancyGrid & grid, const geometry_msgs::msg::Pose & pose,
  double dim_x, double inflation, int8_t value)
{
  if (grid.data.empty() || grid.info.resolution <= 0.0) return;
  const double radius = 0.5 * std::max(0.0, dim_x) + inflation;
  if (radius <= 0.0) return;
  const double px = pose.position.x;
  const double py = pose.position.y;
  const double r2 = radius * radius;

  forEachCellInRadius(grid, px, py, radius, [&](int64_t cx, int64_t cy, double wx, double wy) {
    const double dx = wx - px;
    const double dy = wy - py;
    if (dx * dx + dy * dy <= r2) setCellMax(grid, cx, cy, value);
  });
}

void markObjectFootprint(
  nav_msgs::msg::OccupancyGrid & grid,
  const autoware_perception_msgs::msg::DetectedObject & object,
  const geometry_msgs::msg::Pose & pose_in_grid, double inflation, int8_t value)
{
  using autoware_perception_msgs::msg::Shape;
  switch (object.shape.type) {
    case Shape::CYLINDER:
      markCylinder(grid, pose_in_grid, object.shape.dimensions.x, inflation, value);
      break;
    case Shape::BOUNDING_BOX:
    case Shape::POLYGON:  // conservative AABB from dimensions — never under-marks
    default:
      markBoundingBox(
        grid, pose_in_grid, object.shape.dimensions.x, object.shape.dimensions.y, inflation, value);
      break;
  }
}

size_t firstRetainedIndex(const std::vector<double> & stamps_s, double now_s, double hold_s)
{
  const double cutoff = now_s - hold_s;
  size_t i = 0;
  while (i < stamps_s.size() && stamps_s[i] < cutoff) ++i;
  return i;
}

}  // namespace igv_velocity_planner::detected_objects_grid
