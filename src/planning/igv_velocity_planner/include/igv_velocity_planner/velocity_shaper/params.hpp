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

#ifndef FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__PARAMS_HPP_
#define FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__PARAMS_HPP_

#include <cstdint>

namespace igv_velocity_planner::velocity_shaper
{

// Aggregate parameter bundle for the velocity-shaper pathways. Public so unit
// tests can construct synthetic configs and the node body can declare a single
// member of this type.
struct Params
{
  struct TrajectoryPreprocessing
  {
    double resample_interval{0.5};  // [m] arc-length spacing before any velocity constraints run
  } trajectory_preprocessing;

  struct DynamicObstacleStop
  {
    bool enable{true};                                     // [debug] runtime toggle for this pathway
    double minimum_object_velocity{0.5};                   // [m/s] below this, object is "static"
    double stop_distance_buffer{0.5};                      // [m] gap before the predicted collision
    double time_horizon{5.0};                              // [s] how far ahead to look at the predicted path
    double hysteresis{1.0};                                // [m] tolerance once a stop is committed
    double extra_object_width{1.0};                        // [m] safety inflation around object footprint
    double add_stop_duration_buffer{0.5};                  // [s] sustained-collision time before committing
    double remove_stop_duration_buffer{1.0};               // [s] sustained-clear time before releasing
    double minimum_object_distance_from_ego_trajectory{1.0};  // [m] ignore objects further than this
  } dynamic_obstacle_stop;

  struct AdaptiveCruise
  {
    bool enable{true};
    double follow_distance_min{5.0};       // [m] minimum gap kept at low speed
    double follow_time_gap{1.5};           // [s] time gap term in the desired-gap law
    double velocity_margin{0.5};           // [m/s] subtracted from leader velocity for the cap
    double max_deceleration{1.5};          // [m/s^2] cap on commanded slowdown
    double leader_lateral_margin{1.0};     // [m] half-width of the leader-detection band
    double leader_heading_max_diff{0.7};   // [rad] max heading diff to count as a leader
    double leader_search_distance{60.0};   // [m] max forward distance to look for a leader
  } adaptive_cruise;

  struct PerceptionInputs
  {
    bool use_occupancy_grid{false};        // gate the occupancy-grid pathway (publisher-pending)
  } perception_inputs;

  // Occupancy free-space check. Builds a Euclidean distance map over the grid
  // and samples a corridor (path inflated by the ego half-width) against it.
  // A hard stop fires where the distance to the nearest occupied cell drops
  // below `ego_half_width + required_clearance`; optional graded slow-down
  // fires in the `slow_band` of distances just above that threshold. Consumed
  // from `/perception/occupancy_grid_map/map`; cells {-1 unknown, 0 free, 100
  // occupied} -- unknown is treated as FREE (the topic may never be published
  // on the real vehicle, in which case the whole pathway is a benign no-op).
  struct OccupancyGridCheck
  {
    int8_t  occupancy_threshold{50};        // grid value ≥ this is "occupied" (splits a 0/100 grid)
    double  required_clearance{0.6};        // [m] hard-stop when d_min < ego_half_width + this
    bool    enable_slow_down{false};        // [debug] gate graded slow-down; hard stop remains active
    double  slow_band{1.5};                 // [m] graded slow-down band above the clearance threshold
    double  slow_min_target_velocity{0.5};  // [m/s] target at the inner edge (closest → slowest)
    double  slow_max_target_velocity{4.0};  // [m/s] target at the outer edge (farthest → fastest)
    double  stop_distance_buffer{1.5};      // [m] back the stop off by this much from the intrusion
    double  max_distance{40.0};             // [m] limit forward look
    double  transform_timeout{0.05};        // [s] tf2 lookup timeout
  } occupancy_grid_check;

  // Object-driven "slow down to pass" pathway. Depresses velocity over a
  // localized window when an object sits in the outer lateral band.
  struct SlowDown
  {
    bool   enable{true};
    double inner_lateral_margin{0.5};   // [m] objects inside this are owned by static stop/occupancy
    double outer_lateral_margin{2.0};   // [m] objects beyond this are ignored
    double min_target_velocity{1.5};    // [m/s] target at inner edge (closest → slowest)
    double max_target_velocity{4.0};    // [m/s] target at outer edge (farthest → fastest)
    double forward_margin{5.0};         // [m] window starts this far before the object
    double backward_margin{3.0};        // [m] window ends this far after the object
    double max_distance{50.0};          // [m] don't look past this arc-length
    double add_duration_buffer{0.3};    // [s] sustained-detection before committing
    double remove_duration_buffer{0.5}; // [s] sustained-clear before releasing
  } slow_down;

  // Static-obstacle hard stop. Stops with a safety gap whenever a near-static
  // object's oriented footprint intrudes the ego corridor (ego_half_width +
  // lateral_clearance). Footprint-aware: samples the object's box corners/edges,
  // not just its center, so an elongated or rotated object that reaches the path
  // is caught even when its centroid projects outside the lane. Per-UUID time
  // hysteresis like dynamic_obstacle_stop.
  struct StaticObstacleStop
  {
    bool   enable{true};
    double maximum_object_velocity{0.5};      // [m/s] treat as static below this (moving → dynamic_obstacle_stop)
    double lateral_clearance{0.5};            // [m] footprint must come within ego_half_width + this to stop
    double stop_distance_buffer{2.0};         // [m] gap kept before the object's near face
    double max_distance{50.0};                // [m] don't look past this arc-length
    double add_stop_duration_buffer{0.5};     // [s] sustained-intrusion before committing
    double remove_stop_duration_buffer{1.0};  // [s] sustained-clear before releasing
  } static_obstacle_stop;
};

}  // namespace igv_velocity_planner::velocity_shaper

#endif  // FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__PARAMS_HPP_
