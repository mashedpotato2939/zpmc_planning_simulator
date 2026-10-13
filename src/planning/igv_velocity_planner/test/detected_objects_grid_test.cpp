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
#include "igv_velocity_planner/velocity_shaper/geometry_utils.hpp"
#include "igv_velocity_planner/velocity_shaper/occupancy_stop.hpp"
#include "igv_velocity_planner/velocity_shaper/params.hpp"

#include <gtest/gtest.h>

#include <rclcpp/time.hpp>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace
{

namespace dog = igv_velocity_planner::detected_objects_grid;

using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::OccupancyGrid;

// Read the cell value at world (wx, wy). Assumes an identity-orientation grid
// (as produced by makeEgoCenteredGrid). Returns -128 when out of bounds.
int8_t cellAt(const OccupancyGrid & g, double wx, double wy)
{
  const double res = g.info.resolution;
  const int64_t cx =
    static_cast<int64_t>(std::floor((wx - g.info.origin.position.x) / res));
  const int64_t cy =
    static_cast<int64_t>(std::floor((wy - g.info.origin.position.y) / res));
  if (cx < 0 || cy < 0 || cx >= static_cast<int64_t>(g.info.width) ||
      cy >= static_cast<int64_t>(g.info.height)) {
    return -128;
  }
  return g.data[static_cast<size_t>(cy) * g.info.width + static_cast<size_t>(cx)];
}

geometry_msgs::msg::Pose poseAt(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.orientation.z = std::sin(0.5 * yaw);
  p.orientation.w = std::cos(0.5 * yaw);
  return p;
}

// Straight path along +x at y=0 (mirrors the velocity-shaper test helper).
std::vector<TrajectoryPoint> straightTrajectoryPoints(double length, double step, double velocity)
{
  std::vector<TrajectoryPoint> pts;
  for (double s = 0.0; s <= length + 1e-6; s += step) {
    TrajectoryPoint p;
    p.pose.position.x = s;
    p.pose.position.y = 0.0;
    p.pose.orientation.w = 1.0;
    p.longitudinal_velocity_mps = static_cast<float>(velocity);
    pts.push_back(p);
  }
  return pts;
}

}  // namespace

// ---------------------------------------------------------------------------
// makeEgoCenteredGrid
// ---------------------------------------------------------------------------

TEST(DetectedObjectsGrid, EgoCenteredGridGeometry)
{
  const auto g = dog::makeEgoCenteredGrid(10.0, -5.0, "map", 0.5, 40.0);
  EXPECT_EQ(g.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(g.info.resolution, 0.5);
  EXPECT_EQ(g.info.width, 80u);
  EXPECT_EQ(g.info.height, 80u);
  // Origin is the lower-left corner: ego - length/2.
  EXPECT_DOUBLE_EQ(g.info.origin.position.x, 10.0 - 20.0);
  EXPECT_DOUBLE_EQ(g.info.origin.position.y, -5.0 - 20.0);
  EXPECT_EQ(g.data.size(), 80u * 80u);
  // All free initially.
  for (const auto v : g.data) EXPECT_EQ(v, 0);
}

// ---------------------------------------------------------------------------
// markBoundingBox
// ---------------------------------------------------------------------------

TEST(DetectedObjectsGrid, BoundingBoxAxisAligned)
{
  auto g = dog::makeEgoCenteredGrid(0.0, 0.0, "map", 0.1, 20.0);
  // 2 m (x) by 1 m (y) box at the origin, no inflation.
  dog::markBoundingBox(g, poseAt(0.0, 0.0), 2.0, 1.0, 0.0, 100);

  // Inside the box.
  EXPECT_EQ(cellAt(g, 0.0, 0.0), 100);
  EXPECT_EQ(cellAt(g, 0.9, 0.4), 100);
  EXPECT_EQ(cellAt(g, -0.9, -0.4), 100);
  // Just outside the half-extents (hx=1.0, hy=0.5).
  EXPECT_EQ(cellAt(g, 1.2, 0.0), 0);
  EXPECT_EQ(cellAt(g, 0.0, 0.7), 0);
}

TEST(DetectedObjectsGrid, BoundingBoxRotated)
{
  auto g = dog::makeEgoCenteredGrid(0.0, 0.0, "map", 0.1, 20.0);
  // Long thin box (4 x 0.4) rotated 90 deg → extends along y, not x.
  dog::markBoundingBox(g, poseAt(0.0, 0.0, M_PI / 2.0), 4.0, 0.4, 0.0, 100);

  // Now occupied far along +y (the rotated length axis).
  EXPECT_EQ(cellAt(g, 0.0, 1.8), 100);
  EXPECT_EQ(cellAt(g, 0.0, -1.8), 100);
  // But not far along +x (the rotated width axis, half-width 0.2).
  EXPECT_EQ(cellAt(g, 1.0, 0.0), 0);
}

TEST(DetectedObjectsGrid, BoundingBoxInflationExpandsFootprint)
{
  // A point exactly 0.4 m beyond a box edge is free without inflation but
  // occupied once inflated by the ego half-width (0.5).
  const double edge_query = 1.0 + 0.4;  // hx (1.0) + 0.4

  auto raw = dog::makeEgoCenteredGrid(0.0, 0.0, "map", 0.1, 20.0);
  dog::markBoundingBox(raw, poseAt(0.0, 0.0), 2.0, 1.0, 0.0, 100);
  EXPECT_EQ(cellAt(raw, edge_query, 0.0), 0);

  auto inflated = dog::makeEgoCenteredGrid(0.0, 0.0, "map", 0.1, 20.0);
  dog::markBoundingBox(inflated, poseAt(0.0, 0.0), 2.0, 1.0, /*inflation=*/0.5, 100);
  EXPECT_EQ(cellAt(inflated, edge_query, 0.0), 100);
}

TEST(DetectedObjectsGrid, SetCellMaxNeverLowersValue)
{
  auto g = dog::makeEgoCenteredGrid(0.0, 0.0, "map", 0.1, 20.0);
  dog::markBoundingBox(g, poseAt(0.0, 0.0), 2.0, 1.0, 0.0, 100);
  // Overlapping lower-valued box must not lower the existing 100.
  dog::markBoundingBox(g, poseAt(0.0, 0.0), 2.0, 1.0, 0.0, 40);
  EXPECT_EQ(cellAt(g, 0.0, 0.0), 100);
}

// ---------------------------------------------------------------------------
// markCylinder
// ---------------------------------------------------------------------------

TEST(DetectedObjectsGrid, CylinderMarksDisk)
{
  auto g = dog::makeEgoCenteredGrid(0.0, 0.0, "map", 0.1, 20.0);
  // dim_x = 2.0 → radius 1.0, no inflation.
  dog::markCylinder(g, poseAt(0.0, 0.0), 2.0, 0.0, 100);

  EXPECT_EQ(cellAt(g, 0.0, 0.0), 100);
  EXPECT_EQ(cellAt(g, 0.9, 0.0), 100);
  EXPECT_EQ(cellAt(g, 0.0, 0.9), 100);
  // Corner of the bounding square is outside the disk.
  EXPECT_EQ(cellAt(g, 0.9, 0.9), 0);
  // Beyond the radius.
  EXPECT_EQ(cellAt(g, 1.2, 0.0), 0);
}

// ---------------------------------------------------------------------------
// markObjectFootprint dispatch
// ---------------------------------------------------------------------------

TEST(DetectedObjectsGrid, FootprintDispatchesOnShapeType)
{
  // CYLINDER → disk: corner of the bounding box is free.
  DetectedObject cyl;
  cyl.shape.type = Shape::CYLINDER;
  cyl.shape.dimensions.x = 2.0;
  auto gc = dog::makeEgoCenteredGrid(0.0, 0.0, "map", 0.1, 20.0);
  dog::markObjectFootprint(gc, cyl, poseAt(0.0, 0.0), 0.0, 100);
  EXPECT_EQ(cellAt(gc, 0.0, 0.0), 100);
  EXPECT_EQ(cellAt(gc, 0.9, 0.9), 0);

  // POLYGON falls back to a conservative AABB from dimensions → corner filled.
  DetectedObject poly;
  poly.shape.type = Shape::POLYGON;
  poly.shape.dimensions.x = 2.0;
  poly.shape.dimensions.y = 2.0;
  auto gp = dog::makeEgoCenteredGrid(0.0, 0.0, "map", 0.1, 20.0);
  dog::markObjectFootprint(gp, poly, poseAt(0.0, 0.0), 0.0, 100);
  EXPECT_EQ(cellAt(gp, 0.9, 0.9), 100);
}

// ---------------------------------------------------------------------------
// Producer -> consumer integration: a rasterized detection grid drives the
// velocity-shaper occupancy pathway (distance-map + corridor) to a hard stop.
// This is the off-by-default fallback path (enable_detection_grid).
// ---------------------------------------------------------------------------

TEST(DetectedObjectsGrid, RasterizedGridStopsLaterallyOffsetObstacle)
{
  namespace vs = igv_velocity_planner::velocity_shaper;
  auto pts = straightTrajectoryPoints(40.0, 0.5, 5.0);
  auto grid = dog::makeEgoCenteredGrid(20.0, 0.0, "map", 0.2, 80.0);
  DetectedObject obj;
  obj.shape.type = Shape::BOUNDING_BOX;
  obj.shape.dimensions.x = 0.6;
  obj.shape.dimensions.y = 0.6;
  dog::markObjectFootprint(grid, obj, poseAt(20.0, 0.4), /*inflation=*/0.5, 100);
  EXPECT_GE(cellAt(grid, 20.0, 0.0), 60);

  vs::Params::OccupancyGridCheck params;
  params.occupancy_threshold = 60;
  params.required_clearance = 0.6;
  params.slow_band = 0.0;  // hard stop only
  params.stop_distance_buffer = 1.5;
  params.max_distance = 40.0;
  bool enable = true;
  vs::OccupancyStopGenerator gen(params, enable);
  gen.setGridFramePoints(&pts);
  vs::TickInput in{pts, nullptr, &grid, nullptr, 0.0, 0.5, rclcpp::Time()};
  std::vector<vs::VelocityConstraint> out;
  gen.generate(in, out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].type, vs::ConstraintType::kOccupancyStop);
  EXPECT_NEAR(out[0].target_velocity, 0.0, 1e-9);
  EXPECT_GT(out[0].start_s, 14.0);   // backed off well before the obstacle
  EXPECT_LT(out[0].start_s, 20.0);
}

TEST(DetectedObjectsGrid, RawFootprintStillStopsViaCorridor)
{
  namespace vs = igv_velocity_planner::velocity_shaper;
  auto pts = straightTrajectoryPoints(40.0, 0.5, 5.0);
  auto grid = dog::makeEgoCenteredGrid(20.0, 0.0, "map", 0.2, 80.0);
  DetectedObject obj;
  obj.shape.type = Shape::BOUNDING_BOX;
  obj.shape.dimensions.x = 0.6;
  obj.shape.dimensions.y = 0.6;
  // No producer inflation: footprint stays ~0.4 m off the centerline. The new
  // distance-map + corridor consumer (ego_half_width) still reaches it — the
  // upgrade that removes the old need for producer-side inflation.
  dog::markObjectFootprint(grid, obj, poseAt(20.0, 0.4), /*inflation=*/0.0, 100);
  EXPECT_EQ(cellAt(grid, 20.0, 0.0), 0);

  vs::Params::OccupancyGridCheck params;
  params.occupancy_threshold = 60;
  params.required_clearance = 0.6;
  params.slow_band = 0.0;
  params.max_distance = 40.0;
  bool enable = true;
  vs::OccupancyStopGenerator gen(params, enable);
  gen.setGridFramePoints(&pts);
  vs::TickInput in{pts, nullptr, &grid, nullptr, 0.0, 0.5, rclcpp::Time()};
  std::vector<vs::VelocityConstraint> out;
  gen.generate(in, out);
  ASSERT_FALSE(out.empty());
  EXPECT_EQ(out.front().type, vs::ConstraintType::kOccupancyStop);
}

// ---------------------------------------------------------------------------
// firstRetainedIndex — binary temporal hold bookkeeping
// ---------------------------------------------------------------------------

TEST(DetectedObjectsGrid, FirstRetainedIndexKeepsWithinHold)
{
  // Stamps every 0.1 s, now = 1.0, hold = 0.5 → cutoff 0.5; keep >= 0.5.
  const std::vector<double> stamps{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
  EXPECT_EQ(dog::firstRetainedIndex(stamps, 1.0, 0.5), 4u);  // stamps[4] == 0.5 retained
}

TEST(DetectedObjectsGrid, FirstRetainedIndexOneFrameDropoutStillHeld)
{
  // A detection seen 0.2 s ago is still within a 0.5 s hold (survives a 1-2
  // frame dropout at 10 Hz).
  const std::vector<double> stamps{0.8};
  EXPECT_EQ(dog::firstRetainedIndex(stamps, 1.0, 0.5), 0u);  // retained
}

TEST(DetectedObjectsGrid, FirstRetainedIndexAllExpired)
{
  const std::vector<double> stamps{0.1, 0.2, 0.3};
  // now = 2.0, hold = 0.5 → everything older than 1.5 expired.
  EXPECT_EQ(dog::firstRetainedIndex(stamps, 2.0, 0.5), stamps.size());
}

TEST(DetectedObjectsGrid, FirstRetainedIndexEmpty)
{
  EXPECT_EQ(dog::firstRetainedIndex({}, 1.0, 0.5), 0u);
}
