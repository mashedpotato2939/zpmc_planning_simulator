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

#ifndef FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__DISTANCE_MAP_HPP_
#define FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__DISTANCE_MAP_HPP_

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace igv_velocity_planner::velocity_shaper
{

// ROI-bounded Euclidean Distance Transform over an OccupancyGrid. Distances
// are in METRES to the nearest cell whose value is >= occupancy_threshold.
// Built once per grid message (the EDT pass is O(ROI cells), exact, via the
// Felzenszwalb & Huttenlocher two-pass lower-envelope algorithm), then queried
// many times per shaping cycle. Pure — depends only on OccupancyGrid.
//
// Cells below the threshold (including unknown == -1) are treated as FREE, so
// an all-free / all-unknown grid yields "no obstacle anywhere" (every query
// returns the saturation distance), which is the benign no-op behavior the
// occupancy pathway needs when the topic is sparse or never published.
class DistanceMap
{
public:
  // Build the EDT over the rectangular ROI [cx0,cx1) x [cy0,cy1) in grid cells.
  // The ROI is clamped to the grid bounds; an empty ROI yields valid() == false.
  DistanceMap(
    const nav_msgs::msg::OccupancyGrid & grid, int8_t occupancy_threshold,
    uint32_t cx0, uint32_t cy0, uint32_t cx1, uint32_t cy1);

  bool valid() const { return valid_; }
  double resolution() const { return res_; }
  uint32_t roiWidth() const { return roi_w_; }
  uint32_t roiHeight() const { return roi_h_; }
  uint32_t roiCellX0() const { return cx0_; }
  uint32_t roiCellY0() const { return cy0_; }
  double saturationDistance() const { return saturation_m_; }

  double distanceAtCell(uint32_t roi_x, uint32_t roi_y) const
  {
    if (!valid_ || roi_x >= roi_w_ || roi_y >= roi_h_) return saturation_m_;
    return dist_[static_cast<size_t>(roi_y) * roi_w_ + roi_x];
  }

  // Metric distance-to-nearest-obstacle at a point in GRID-LOCAL coordinates
  // (already origin-subtracted and de-rotated by the grid yaw — the same
  // transform the legacy occupancy check applied). Bilinear over the four
  // surrounding cell-center distances. Returns the saturation distance when
  // the point falls outside the ROI (treated as clear).
  double queryMetric(double gx, double gy) const;

private:
  bool valid_{false};
  double res_{0.0};
  uint32_t roi_w_{0};
  uint32_t roi_h_{0};
  uint32_t cx0_{0};
  uint32_t cy0_{0};
  double saturation_m_{0.0};         // distance reported when no obstacle is in range
  std::vector<float> dist_;          // ROI-local, row-major, metres at cell index
};

}  // namespace igv_velocity_planner::velocity_shaper

#endif  // FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__DISTANCE_MAP_HPP_
