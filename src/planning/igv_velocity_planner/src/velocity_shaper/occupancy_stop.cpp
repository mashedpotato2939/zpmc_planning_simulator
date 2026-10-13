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

#include "igv_velocity_planner/velocity_shaper/occupancy_stop.hpp"

#include "igv_velocity_planner/velocity_shaper/geometry_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace igv_velocity_planner::velocity_shaper
{

namespace
{
int64_t stampToNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}
}  // namespace

bool OccupancyStopGenerator::hasCachedDistanceMapFor(const nav_msgs::msg::OccupancyGrid & grid) const
{
  return distanceMap() != nullptr && cached_grid_ == &grid &&
         cached_stamp_ns_ == stampToNanoseconds(grid.header.stamp) &&
         cached_width_ == grid.info.width && cached_height_ == grid.info.height &&
         cached_resolution_ == grid.info.resolution && cached_data_size_ == grid.data.size() &&
         cached_occupancy_threshold_ == params_.occupancy_threshold;
}

bool OccupancyStopGenerator::ensureDistanceMap(const nav_msgs::msg::OccupancyGrid & grid)
{
  if (!enable_ || grid.data.empty()) return false;
  if (grid.info.resolution <= 0.0 || grid.info.width == 0 || grid.info.height == 0) return false;

  const size_t expected_size =
    static_cast<size_t>(grid.info.width) * static_cast<size_t>(grid.info.height);
  if (grid.data.size() != expected_size) return false;

  const int64_t stamp_ns = stampToNanoseconds(grid.header.stamp);
  const bool cache_matches = hasCachedDistanceMapFor(grid);
  if (!cache_matches) {
    edt_ = std::make_unique<DistanceMap>(
      grid, params_.occupancy_threshold, 0u, 0u, grid.info.width, grid.info.height);
    cached_stamp_ns_ = stamp_ns;
    cached_grid_ = &grid;
    cached_width_ = grid.info.width;
    cached_height_ = grid.info.height;
    cached_resolution_ = grid.info.resolution;
    cached_data_size_ = grid.data.size();
    cached_occupancy_threshold_ = params_.occupancy_threshold;
  }

  return distanceMap() != nullptr;
}

void OccupancyStopGenerator::generate(const TickInput & in, std::vector<VelocityConstraint> & out)
{
  if (!enable_ || in.grid == nullptr || in.grid->data.empty()) return;
  const auto * gpts = grid_frame_points_;
  if (gpts == nullptr || gpts->size() < 2 || gpts->size() != in.points.size()) return;

  const auto & grid = *in.grid;
  if (!ensureDistanceMap(grid)) return;

  // Grid-local transform (origin-subtract + de-rotate by the grid yaw) — same
  // convention as the legacy occupancy check and DistanceMap::queryMetric.
  const double ox = grid.info.origin.position.x;
  const double oy = grid.info.origin.position.y;
  const double grid_yaw = yawFromQuat(grid.info.origin.orientation);
  const double cy = std::cos(-grid_yaw);
  const double sy = std::sin(-grid_yaw);
  const auto to_local = [&](double wx, double wy, double & lx, double & ly) {
    const double dx = wx - ox;
    const double dy = wy - oy;
    lx = cy * dx - sy * dy;
    ly = sy * dx + cy * dy;
  };

  const size_t n = gpts->size();
  std::vector<double> lx(n), ly(n);
  for (size_t i = 0; i < n; ++i) {
    to_local((*gpts)[i].pose.position.x, (*gpts)[i].pose.position.y, lx[i], ly[i]);
  }

  const double required = in.ego_half_width + params_.required_clearance;
  const bool slow_down_enabled = params_.enable_slow_down && params_.slow_band > 0.0;
  const double slow_outer = slow_down_enabled ? required + params_.slow_band : required;
  const double hw = std::max(0.0, in.ego_half_width);

  double s_accum = 0.0;
  for (size_t i = 0; i < n; ++i) {
    double s_prev = s_accum;
    if (i > 0) {
      s_accum += distance2D(in.points[i].pose.position, in.points[i - 1].pose.position);
    }
    if (s_accum < in.ego_s) continue;
    if (s_accum > in.ego_s + params_.max_distance) break;

    // Local tangent → perpendicular, to offset across the corridor half-width.
    double tx = 0.0, ty = 0.0;
    if (i + 1 < n) {
      tx = lx[i + 1] - lx[i];
      ty = ly[i + 1] - ly[i];
    } else {
      tx = lx[i] - lx[i - 1];
      ty = ly[i] - ly[i - 1];
    }
    const double tn = std::hypot(tx, ty);
    double px = 0.0, py = 0.0;
    if (tn > 1e-9) {
      px = -ty / tn;
      py = tx / tn;
    }

    double d_min = std::numeric_limits<double>::infinity();
    for (const double off : {-hw, 0.0, hw}) {
      d_min = std::min(d_min, edt_->queryMetric(lx[i] + off * px, ly[i] + off * py));
    }

    if (d_min < required) {
      VelocityConstraint c;
      c.start_s = std::max(in.ego_s, s_accum - params_.stop_distance_buffer);
      c.end_s = std::numeric_limits<double>::infinity();
      c.target_velocity = 0.0;
      c.type = ConstraintType::kOccupancyStop;
      c.priority = priorityOf(ConstraintType::kOccupancyStop);
      c.source = "occupancy_stop";
      c.anchor = in.points[i].pose.position;
      out.push_back(std::move(c));
      return;  // first hard intrusion wins; nothing beyond a stop matters
    }

    if (slow_down_enabled && d_min < slow_outer && i > 0) {
      const double ratio = std::clamp((d_min - required) / params_.slow_band, 0.0, 1.0);
      VelocityConstraint c;
      c.start_s = std::max(in.ego_s, s_prev);
      c.end_s = s_accum;  // tiles the corridor segment-by-segment
      c.target_velocity = params_.slow_min_target_velocity +
        ratio * (params_.slow_max_target_velocity - params_.slow_min_target_velocity);
      c.type = ConstraintType::kSlowDown;
      c.priority = priorityOf(ConstraintType::kSlowDown);
      c.source = "occupancy_slow_down";
      c.anchor = in.points[i].pose.position;
      out.push_back(std::move(c));
    }
  }
}

}  // namespace igv_velocity_planner::velocity_shaper
