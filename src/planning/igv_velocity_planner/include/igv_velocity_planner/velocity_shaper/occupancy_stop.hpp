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

#ifndef FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__OCCUPANCY_STOP_HPP_
#define FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__OCCUPANCY_STOP_HPP_

#include "igv_velocity_planner/velocity_shaper/constraint_generator.hpp"
#include "igv_velocity_planner/velocity_shaper/distance_map.hpp"
#include "igv_velocity_planner/velocity_shaper/params.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace igv_velocity_planner::velocity_shaper
{

// Free-space occupancy generator. Builds a Euclidean distance map over
// `/perception/occupancy_grid_map/map` and samples a corridor (the path
// inflated by the ego half-width) against it:
//   * emits a hard kOccupancyStop (v=0) at the first sample whose corridor
//     distance-to-nearest-obstacle drops below ego_half_width + required_clearance;
//   * optionally emits graded kSlowDown constraints over the band of distances
//     just above that threshold (closer -> slower).
//
// The trajectory points must be supplied in the GRID frame via
// setGridFramePoints() before generate() (the node performs the TF). When the
// grid is absent, empty, or the grid-frame points were not supplied (TF
// failure), the generator is a benign no-op — the real vehicle may never
// publish the topic.
class OccupancyStopGenerator : public IConstraintGenerator
{
public:
  OccupancyStopGenerator(const Params::OccupancyGridCheck & params, const bool & enable)
  : params_(params), enable_(enable)
  {
  }

  const char * name() const override { return "occupancy_stop"; }
  bool enabled() const override { return enable_; }

  // Non-owning: the node owns the buffer (refreshed each cycle).
  void setGridFramePoints(const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> * pts)
  {
    grid_frame_points_ = pts;
  }

  const DistanceMap * distanceMap() const { return edt_ && edt_->valid() ? edt_.get() : nullptr; }
  bool ensureDistanceMap(const nav_msgs::msg::OccupancyGrid & grid);
  bool hasCachedDistanceMapFor(const nav_msgs::msg::OccupancyGrid & grid) const;

  void generate(const TickInput & in, std::vector<VelocityConstraint> & out) override;

private:
  const Params::OccupancyGridCheck & params_;
  const bool & enable_;
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> * grid_frame_points_{nullptr};

  // EDT cached across cycles; rebuilt when the current grid message or EDT parameters change.
  std::unique_ptr<DistanceMap> edt_;
  int64_t cached_stamp_ns_{-1};
  const nav_msgs::msg::OccupancyGrid * cached_grid_{nullptr};
  uint32_t cached_width_{0};
  uint32_t cached_height_{0};
  double cached_resolution_{0.0};
  size_t cached_data_size_{0};
  int8_t cached_occupancy_threshold_{0};
};

}  // namespace igv_velocity_planner::velocity_shaper

#endif  // FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__OCCUPANCY_STOP_HPP_
