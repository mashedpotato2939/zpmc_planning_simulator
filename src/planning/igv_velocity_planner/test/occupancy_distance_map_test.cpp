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

#include "igv_velocity_planner/velocity_shaper/distance_map.hpp"
#include "igv_velocity_planner/velocity_shaper/geometry_utils.hpp"
#include "igv_velocity_planner/velocity_shaper/occupancy_debug.hpp"
#include "igv_velocity_planner/velocity_shaper/occupancy_stop.hpp"

#include <gtest/gtest.h>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/time.hpp>

#include <cmath>
#include <vector>

namespace
{
namespace vs = igv_velocity_planner::velocity_shaper;
using TrajectoryPoint = autoware_planning_msgs::msg::TrajectoryPoint;

nav_msgs::msg::OccupancyGrid makeFreeGrid(uint32_t cells, double resolution)
{
  nav_msgs::msg::OccupancyGrid g;
  g.info.resolution = resolution;
  g.info.width = cells;
  g.info.height = cells;
  g.info.origin.orientation.w = 1.0;
  g.data.assign(static_cast<size_t>(cells) * cells, 0);
  return g;
}

void setCell(nav_msgs::msg::OccupancyGrid & g, uint32_t cx, uint32_t cy, int8_t value)
{
  g.data[static_cast<size_t>(cy) * g.info.width + cx] = value;
}

std::vector<TrajectoryPoint> straightPoints(double length, double step, double vel)
{
  std::vector<TrajectoryPoint> pts;
  for (double x = 0.0; x <= length + 1e-9; x += step) {
    TrajectoryPoint p;
    p.pose.position.x = x;
    p.pose.orientation.w = 1.0;
    p.longitudinal_velocity_mps = static_cast<float>(vel);
    pts.push_back(p);
  }
  return pts;
}
}  // namespace

// EDT distances match the analytic Euclidean distance to the single obstacle.
TEST(DistanceMap, KnownGeometryEuclidean)
{
  auto g = makeFreeGrid(10, 1.0);
  setCell(g, 5, 5, 100);
  vs::DistanceMap dm(g, 50, 0u, 0u, 10u, 10u);
  ASSERT_TRUE(dm.valid());
  EXPECT_NEAR(dm.queryMetric(5.0, 5.0), 0.0, 1e-3);  // at the obstacle
  EXPECT_NEAR(dm.queryMetric(8.0, 5.0), 3.0, 1e-3);  // 3 cells away in x
  EXPECT_NEAR(dm.queryMetric(5.0, 9.0), 4.0, 1e-3);  // 4 cells away in y
}

// An all-free grid reports the saturation distance everywhere (no obstacle).
TEST(DistanceMap, AllFreeSaturates)
{
  auto g = makeFreeGrid(10, 1.0);
  vs::DistanceMap dm(g, 50, 0u, 0u, 10u, 10u);
  ASSERT_TRUE(dm.valid());
  EXPECT_GT(dm.queryMetric(5.0, 5.0), 5.0);
}

TEST(OccupancyDebug, DistanceMapGridScalesAndPreservesMetadata)
{
  auto g = makeFreeGrid(4, 1.0);
  g.header.frame_id = "map";
  g.header.stamp.sec = 12;
  g.info.origin.position.x = 3.0;
  setCell(g, 0, 0, 100);

  vs::DistanceMap dm(g, 50, 0u, 0u, 4u, 4u);
  auto debug_grid = vs::makeOccupancyDistanceMapGrid(g, dm, 2.0);

  ASSERT_TRUE(debug_grid.has_value());
  EXPECT_EQ(debug_grid->header.frame_id, "map");
  EXPECT_EQ(debug_grid->header.stamp.sec, 12);
  EXPECT_DOUBLE_EQ(debug_grid->info.origin.position.x, 3.0);
  EXPECT_EQ(debug_grid->info.width, 4u);
  EXPECT_EQ(debug_grid->info.height, 4u);
  ASSERT_EQ(debug_grid->data.size(), 16u);
  EXPECT_EQ(debug_grid->data[0], 0);   // obstacle cell: 0 m
  EXPECT_EQ(debug_grid->data[1], 50);  // 1 m of a 2 m display cap
  EXPECT_EQ(debug_grid->data[2], 100); // at/above cap
}

TEST(OccupancyDebug, CorridorMarkersIncludeCenterlineAndBoundaries)
{
  std_msgs::msg::Header header;
  header.frame_id = "map";
  auto grid = makeFreeGrid(10, 1.0);
  setCell(grid, 1, 0, 100);
  vs::DistanceMap dm(grid, 50, 0u, 0u, 10u, 10u);
  const auto pts = straightPoints(2.0, 1.0, 1.0);

  auto markers = vs::makeOccupancyCorridorMarkers(header, grid, dm, pts, 1.0, 10.0, 0.1);

  ASSERT_EQ(markers.markers.size(), 6u);
  EXPECT_EQ(markers.markers[0].action, visualization_msgs::msg::Marker::DELETEALL);

  const auto & center = markers.markers[1];
  const auto & left = markers.markers[2];
  const auto & right = markers.markers[3];
  const auto & min_point = markers.markers[4];
  const auto & text = markers.markers[5];
  EXPECT_EQ(center.ns, "fms_velocity_shaper/occupancy_corridor_center");
  EXPECT_EQ(left.ns, "fms_velocity_shaper/occupancy_corridor_boundary");
  EXPECT_EQ(right.ns, "fms_velocity_shaper/occupancy_corridor_boundary");
  EXPECT_EQ(min_point.ns, "fms_velocity_shaper/occupancy_corridor_min_point");
  EXPECT_EQ(min_point.type, visualization_msgs::msg::Marker::SPHERE);
  EXPECT_EQ(text.ns, "fms_velocity_shaper/occupancy_corridor_distance_text");
  EXPECT_EQ(text.type, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  EXPECT_EQ(text.text, "min d=0.00m");
  ASSERT_EQ(center.points.size(), 3u);
  ASSERT_EQ(left.points.size(), 3u);
  ASSERT_EQ(right.points.size(), 3u);
  EXPECT_NEAR(center.points.front().y, 0.0, 1e-9);
  EXPECT_NEAR(left.points.front().y, 1.0, 1e-9);
  EXPECT_NEAR(right.points.front().y, -1.0, 1e-9);
}

TEST(OccupancyDebug, ResampledSparsePathPublishesCorridorWithinMaxDistance)
{
  std_msgs::msg::Header header;
  header.frame_id = "map";
  auto grid = makeFreeGrid(80, 0.5);
  setCell(grid, 40, 0, 100);
  vs::DistanceMap dm(grid, 90, 0u, 0u, 80u, 80u);
  const auto sparse = straightPoints(24.0, 24.0, 5.0);
  const auto dense = vs::resampleTrajectoryPoints(sparse, 0.5);

  auto markers = vs::makeOccupancyCorridorMarkers(header, grid, dm, dense, 1.0, 10.0, 0.1);

  ASSERT_GE(markers.markers.size(), 4u);
  const auto & center = markers.markers[1];
  EXPECT_EQ(center.ns, "fms_velocity_shaper/occupancy_corridor_center");
  EXPECT_GT(center.points.size(), 2u);
  EXPECT_NEAR(center.points.front().x, 0.0, 1e-9);
  EXPECT_LE(center.points.back().x, 10.0 + 1e-6);
}

TEST(OccupancyDebug, SparsePathClipsCorridorAtMaxDistance)
{
  std_msgs::msg::Header header;
  header.frame_id = "map";
  auto grid = makeFreeGrid(80, 0.5);
  setCell(grid, 40, 0, 100);
  vs::DistanceMap dm(grid, 90, 0u, 0u, 80u, 80u);
  const auto pts = straightPoints(24.0, 24.0, 5.0);

  auto markers = vs::makeOccupancyCorridorMarkers(header, grid, dm, pts, 1.0, 10.0, 0.1);

  ASSERT_GE(markers.markers.size(), 4u);
  const auto & center = markers.markers[1];
  ASSERT_EQ(center.points.size(), 2u);
  EXPECT_NEAR(center.points.front().x, 0.0, 1e-9);
  EXPECT_NEAR(center.points.back().x, 10.0, 1e-9);
}

TEST(OccupancyDebug, InvalidInputsProduceNoDebugPayload)
{
  nav_msgs::msg::OccupancyGrid empty_grid;
  vs::DistanceMap invalid_dm(empty_grid, 50, 0u, 0u, 0u, 0u);
  EXPECT_FALSE(vs::makeOccupancyDistanceMapGrid(empty_grid, invalid_dm, 1.0).has_value());

  std_msgs::msg::Header header;
  auto markers = vs::makeOccupancyCorridorMarkers(header, empty_grid, invalid_dm, {}, 1.0, 10.0, 1.0);
  EXPECT_TRUE(markers.markers.empty());
}

TEST(OccupancyStop, EnsureDistanceMapBuildsCacheWithoutPathPoints)
{
  auto grid = makeFreeGrid(10, 1.0);
  grid.header.stamp.sec = 7;
  setCell(grid, 5, 5, 100);
  vs::Params::OccupancyGridCheck params;
  bool enable = true;

  vs::OccupancyStopGenerator gen(params, enable);

  EXPECT_TRUE(gen.ensureDistanceMap(grid));
  EXPECT_TRUE(gen.hasCachedDistanceMapFor(grid));
  ASSERT_NE(gen.distanceMap(), nullptr);
  EXPECT_NEAR(gen.distanceMap()->queryMetric(5.0, 5.0), 0.0, 1e-3);
}

// A cell blocking the path centerline produces a hard occupancy stop backed off
// by stop_distance_buffer (slow_band 0 → no graded constraints, clean assertion).
TEST(OccupancyStop, BlockedCorridorHardStop)
{
  auto pts = straightPoints(40.0, 0.5, 5.0);
  auto grid = makeFreeGrid(80, 0.5);  // 40 m × 40 m
  setCell(grid, 40, 0, 100);          // ~ (20.0, 0.0)

  vs::Params::OccupancyGridCheck params;
  params.required_clearance = 0.6;
  params.slow_band = 0.0;             // hard stop only
  params.stop_distance_buffer = 1.5;
  bool enable = true;

  vs::OccupancyStopGenerator gen(params, enable);
  gen.setGridFramePoints(&pts);
  vs::TickInput in{pts, nullptr, &grid, nullptr, 0.0, 1.0, rclcpp::Time()};
  std::vector<vs::VelocityConstraint> out;
  gen.generate(in, out);

  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].type, vs::ConstraintType::kOccupancyStop);
  EXPECT_NEAR(out[0].target_velocity, 0.0, 1e-6);
  EXPECT_GT(out[0].start_s, 15.0);
  EXPECT_LT(out[0].start_s, 19.0);
}

TEST(OccupancyStop, BackBufferCellBehindEgoDoesNotStop)
{
  auto pts = straightPoints(40.0, 0.5, 5.0);
  auto grid = makeFreeGrid(80, 0.5);
  setCell(grid, 16, 0, 100);  // ~ (8.0, 0.0), inside the retained back-buffer

  vs::Params::OccupancyGridCheck params;
  params.required_clearance = 0.2;
  params.slow_band = 0.0;
  params.stop_distance_buffer = 1.5;
  params.max_distance = 30.0;
  bool enable = true;

  vs::OccupancyStopGenerator gen(params, enable);
  gen.setGridFramePoints(&pts);
  vs::TickInput in{
    pts, nullptr, &grid, nullptr, 0.0, 0.0, rclcpp::Time(), /*ego_s=*/10.0};
  std::vector<vs::VelocityConstraint> out;
  gen.generate(in, out);

  EXPECT_TRUE(out.empty());
}

TEST(OccupancyStop, ForwardCellAfterEgoStillStops)
{
  auto pts = straightPoints(40.0, 0.5, 5.0);
  auto grid = makeFreeGrid(80, 0.5);
  setCell(grid, 24, 0, 100);  // ~ (12.0, 0.0), ahead of ego_s=10

  vs::Params::OccupancyGridCheck params;
  params.required_clearance = 0.2;
  params.slow_band = 0.0;
  params.stop_distance_buffer = 1.5;
  params.max_distance = 30.0;
  bool enable = true;

  vs::OccupancyStopGenerator gen(params, enable);
  gen.setGridFramePoints(&pts);
  vs::TickInput in{
    pts, nullptr, &grid, nullptr, 0.0, 0.0, rclcpp::Time(), /*ego_s=*/10.0};
  std::vector<vs::VelocityConstraint> out;
  gen.generate(in, out);

  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].type, vs::ConstraintType::kOccupancyStop);
  EXPECT_NEAR(out[0].target_velocity, 0.0, 1e-9);
  EXPECT_GE(out[0].start_s, 10.0);
  EXPECT_LT(out[0].start_s, 12.0);
}

// An obstacle just outside the hard-clearance band but inside the slow band
// produces graded slow-down constraints and no hard stop.
TEST(OccupancyStop, NearObstacleGradedSlowDown)
{
  auto pts = straightPoints(40.0, 0.5, 5.0);
  auto grid = makeFreeGrid(80, 0.5);
  setCell(grid, 40, 6, 100);          // ~ (20.0, 3.0): 3 m lateral

  vs::Params::OccupancyGridCheck params;
  params.required_clearance = 0.6;    // required = 1.0 (half width) + 0.6 = 1.6
  params.enable_slow_down = true;
  params.slow_band = 1.5;             // slow band up to 3.1 m
  params.slow_min_target_velocity = 0.5;
  params.slow_max_target_velocity = 4.0;
  bool enable = true;

  vs::OccupancyStopGenerator gen(params, enable);
  gen.setGridFramePoints(&pts);
  vs::TickInput in{pts, nullptr, &grid, nullptr, 0.0, 1.0, rclcpp::Time()};
  std::vector<vs::VelocityConstraint> out;
  gen.generate(in, out);

  ASSERT_FALSE(out.empty());
  bool any_stop = false;
  bool any_slow = false;
  for (const auto & c : out) {
    if (c.type == vs::ConstraintType::kOccupancyStop) any_stop = true;
    if (c.type == vs::ConstraintType::kSlowDown) {
      any_slow = true;
      EXPECT_GE(c.target_velocity, params.slow_min_target_velocity - 1e-6);
      EXPECT_LE(c.target_velocity, params.slow_max_target_velocity + 1e-6);
    }
  }
  EXPECT_FALSE(any_stop);
  EXPECT_TRUE(any_slow);
}

// With the occupancy slow-down toggle off, a near obstacle inside the slow band
// but outside the hard-stop band emits no constraint.
TEST(OccupancyStop, NearObstacleSlowDownDisabledNoConstraint)
{
  auto pts = straightPoints(40.0, 0.5, 5.0);
  auto grid = makeFreeGrid(80, 0.5);
  setCell(grid, 40, 6, 100);          // ~ (20.0, 3.0): 3 m lateral

  vs::Params::OccupancyGridCheck params;
  params.required_clearance = 0.6;    // required = 1.0 (half width) + 0.6 = 1.6
  params.enable_slow_down = false;
  params.slow_band = 1.5;             // would slow down if enabled
  bool enable = true;

  vs::OccupancyStopGenerator gen(params, enable);
  gen.setGridFramePoints(&pts);
  vs::TickInput in{pts, nullptr, &grid, nullptr, 0.0, 1.0, rclcpp::Time()};
  std::vector<vs::VelocityConstraint> out;
  gen.generate(in, out);

  EXPECT_TRUE(out.empty());
}

// All-free grid → no constraints (benign).
TEST(OccupancyStop, FreeGridNoConstraints)
{
  auto pts = straightPoints(40.0, 0.5, 5.0);
  auto grid = makeFreeGrid(80, 0.5);
  vs::Params::OccupancyGridCheck params;
  bool enable = true;
  vs::OccupancyStopGenerator gen(params, enable);
  gen.setGridFramePoints(&pts);
  vs::TickInput in{pts, nullptr, &grid, nullptr, 0.0, 1.0, rclcpp::Time()};
  std::vector<vs::VelocityConstraint> out;
  gen.generate(in, out);
  EXPECT_TRUE(out.empty());
}

// Disabled, or grid-frame points not supplied (TF failure) → benign no-op.
TEST(OccupancyStop, DisabledOrUnsuppliedIsNoop)
{
  auto pts = straightPoints(40.0, 0.5, 5.0);
  auto grid = makeFreeGrid(80, 0.5);
  setCell(grid, 40, 0, 100);
  vs::Params::OccupancyGridCheck params;

  {
    bool enable = false;
    vs::OccupancyStopGenerator gen(params, enable);
    gen.setGridFramePoints(&pts);
    vs::TickInput in{pts, nullptr, &grid, nullptr, 0.0, 1.0, rclcpp::Time()};
    std::vector<vs::VelocityConstraint> out;
    gen.generate(in, out);
    EXPECT_TRUE(out.empty());
  }
  {
    bool enable = true;
    vs::OccupancyStopGenerator gen(params, enable);  // grid-frame points never set
    vs::TickInput in{pts, nullptr, &grid, nullptr, 0.0, 1.0, rclcpp::Time()};
    std::vector<vs::VelocityConstraint> out;
    gen.generate(in, out);
    EXPECT_TRUE(out.empty());
  }
}
