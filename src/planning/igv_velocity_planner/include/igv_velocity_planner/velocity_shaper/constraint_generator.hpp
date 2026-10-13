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

#ifndef FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__CONSTRAINT_GENERATOR_HPP_
#define FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__CONSTRAINT_GENERATOR_HPP_

#include "igv_velocity_planner/velocity_shaper/constraint.hpp"

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>

#include <optional>
#include <vector>

namespace igv_velocity_planner::velocity_shaper
{

// One immutable snapshot of the world for a single shaping cycle. Every
// generator reads the same tick, so all constraints are generated against a
// consistent view (the time-consistency property the legacy per-pathway
// caches lacked). `points` are in the path/trajectory frame. Pointers are
// nullable — a generator must no-op when its input is absent.
struct TickInput
{
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points;
  const autoware_perception_msgs::msg::PredictedObjects * objects{nullptr};
  const nav_msgs::msg::OccupancyGrid * grid{nullptr};
  const nav_msgs::msg::Odometry * odometry{nullptr};
  double ego_front_clearance{0.0};
  double ego_half_width{0.0};
  rclcpp::Time stamp;
  // Arc length of the current ego pose on `points`; obstacle stops must be
  // forward of this even when the path keeps a short back-buffer behind ego.
  double ego_s{0.0};
  // Rear bumper offset in the ego/base_link frame. When present with odometry,
  // dynamic stops can ignore objects fully behind the ego footprint.
  std::optional<double> ego_min_longitudinal_offset{std::nullopt};
};

// A constraint generator turns perception + path into zero or more
// VelocityConstraints. Generators are owned by the node (so per-UUID
// hysteresis state survives across ticks) and are pure with respect to the
// trajectory — they append to `out`, never mutate `points`.
class IConstraintGenerator
{
public:
  virtual ~IConstraintGenerator() = default;
  virtual const char * name() const = 0;
  virtual bool enabled() const = 0;
  virtual void generate(const TickInput & in, std::vector<VelocityConstraint> & out) = 0;
};

}  // namespace igv_velocity_planner::velocity_shaper

#endif  // FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__CONSTRAINT_GENERATOR_HPP_
