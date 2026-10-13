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

#ifndef FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__GENERATORS_HPP_
#define FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__GENERATORS_HPP_

#include "igv_velocity_planner/velocity_shaper/adaptive_cruise.hpp"
#include "igv_velocity_planner/velocity_shaper/constraint_generator.hpp"
#include "igv_velocity_planner/velocity_shaper/dynamic_obstacle_stop.hpp"
#include "igv_velocity_planner/velocity_shaper/params.hpp"
#include "igv_velocity_planner/velocity_shaper/slow_down.hpp"
#include "igv_velocity_planner/velocity_shaper/static_obstacle_stop.hpp"

#include <rclcpp/time.hpp>

#include <string>
#include <unordered_map>

namespace igv_velocity_planner::velocity_shaper
{

// Single-leader ACC cap (stateless).
class AdaptiveCruiseGenerator : public IConstraintGenerator
{
public:
  explicit AdaptiveCruiseGenerator(const Params::AdaptiveCruise & params) : params_(params) {}
  const char * name() const override { return "adaptive_cruise"; }
  bool enabled() const override { return params_.enable; }
  void generate(const TickInput & in, std::vector<VelocityConstraint> & out) override;

private:
  const Params::AdaptiveCruise & params_;
};

// Predicted-collision stop with per-UUID time hysteresis (stateful).
class DynamicStopGenerator : public IConstraintGenerator
{
public:
  DynamicStopGenerator(
    const Params::DynamicObstacleStop & params, const double & ego_front_clearance,
    const double & ego_half_width)
  : params_(params), ego_front_clearance_(ego_front_clearance), ego_half_width_(ego_half_width)
  {
  }
  const char * name() const override { return "dynamic_obstacle_stop"; }
  bool enabled() const override { return params_.enable; }
  void generate(const TickInput & in, std::vector<VelocityConstraint> & out) override;

private:
  struct State
  {
    rclcpp::Time first_collision_time;
    rclcpp::Time last_collision_time;
    bool committed{false};
    double last_stop_distance{0.0};
  };
  const Params::DynamicObstacleStop & params_;
  const double & ego_front_clearance_;
  const double & ego_half_width_;
  std::unordered_map<std::string, State> decisions_;
};

// Footprint-aware static-obstacle hard stop with per-UUID time hysteresis
// (stateful). Emits a v=0 constraint, backed off by the ego front clearance +
// stop_distance_buffer, when a near-static object's oriented footprint intrudes
// the ego corridor.
class StaticObstacleStopGenerator : public IConstraintGenerator
{
public:
  StaticObstacleStopGenerator(
    const Params::StaticObstacleStop & params, const double & ego_front_clearance,
    const double & ego_half_width)
  : params_(params), ego_front_clearance_(ego_front_clearance), ego_half_width_(ego_half_width)
  {
  }
  const char * name() const override { return "static_obstacle_stop"; }
  bool enabled() const override { return params_.enable; }
  void generate(const TickInput & in, std::vector<VelocityConstraint> & out) override;

private:
  struct State
  {
    rclcpp::Time first_intrusion_time;
    rclcpp::Time last_intrusion_time;
    bool committed{false};
    double last_stop_distance{0.0};
  };
  const Params::StaticObstacleStop & params_;
  const double & ego_front_clearance_;
  const double & ego_half_width_;
  std::unordered_map<std::string, State> decisions_;
};

// Outer-band object slow-down with per-UUID time hysteresis (stateful).
class SlowDownGenerator : public IConstraintGenerator
{
public:
  SlowDownGenerator(const Params::SlowDown & params, const double & ego_half_width)
  : params_(params), ego_half_width_(ego_half_width)
  {
  }
  const char * name() const override { return "slow_down"; }
  bool enabled() const override { return params_.enable; }
  void generate(const TickInput & in, std::vector<VelocityConstraint> & out) override;

private:
  struct State
  {
    rclcpp::Time first_detection_time;
    rclcpp::Time last_detection_time;
    bool committed{false};
  };
  const Params::SlowDown & params_;
  const double & ego_half_width_;
  std::unordered_map<std::string, State> decisions_;
};

}  // namespace igv_velocity_planner::velocity_shaper

#endif  // FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__GENERATORS_HPP_
