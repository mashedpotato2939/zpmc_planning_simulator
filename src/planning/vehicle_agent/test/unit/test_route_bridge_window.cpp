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

#include "vehicle_agent/ros_bridge/route_bridge.hpp"

#include <gtest/gtest.h>

using vehicle_agent::RouteBridgeNode;
using FMSRouteWindow = igv_planning_msgs::msg::FMSRouteWindow;
using FMSPathSegment = igv_planning_msgs::msg::FMSPathSegment;
using SegmentWaypoint = igv_planning_msgs::msg::SegmentWaypoint;

namespace
{
SegmentWaypoint pt(double x, double y,
                   uint8_t type = SegmentWaypoint::POINT_NORMAL)
{
  SegmentWaypoint p;
  p.x = x;
  p.y = y;
  p.point_type = type;
  return p;
}

// Build a single-segment forward route with N points spaced 1m apart along x.
FMSRouteWindow makeLine(std::size_t n)
{
  FMSRouteWindow w;
  w.vehicle_id = "v";
  w.route_id = "r";
  w.status = FMSRouteWindow::STATUS_ACTIVE;
  FMSPathSegment seg;
  seg.orientation = FMSPathSegment::ORIENT_FORWARD;
  for (std::size_t i = 0; i < n; ++i) seg.points.push_back(pt(double(i), 0.0));
  w.segments.push_back(std::move(seg));
  return w;
}
}  // namespace

TEST(SliceAroundEgo, NoEgoReturnsFullRoute)
{
  auto full = makeLine(20);
  auto out = RouteBridgeNode::sliceAroundEgo(full, 0.0, 0.0, false, 10.0, 30.0);
  ASSERT_EQ(out.segments.size(), 1u);
  EXPECT_EQ(out.segments[0].points.size(), 20u);
}

TEST(SliceAroundEgo, EgoMidRouteCentersWindow)
{
  auto full = makeLine(100);    // x = 0..99
  auto out = RouteBridgeNode::sliceAroundEgo(
    full, /*ego_x=*/50.0, /*ego_y=*/0.0, true,
    /*behind_m=*/5.0, /*ahead_m=*/10.0);
  ASSERT_EQ(out.segments.size(), 1u);
  const auto & pts = out.segments[0].points;
  ASSERT_FALSE(pts.empty());
  // First point should be ~5 m behind ego (the walk stops AFTER the threshold
  // is exceeded, so the first point may be one step further back).
  EXPECT_LE(pts.front().x, 45.0);
  EXPECT_GE(pts.front().x, 44.0);
  // Last point should be ~10 m ahead.
  EXPECT_GE(pts.back().x, 60.0);
  EXPECT_LE(pts.back().x, 61.0);
}

TEST(SliceAroundEgo, EgoBeforeRouteStartIncludesStart)
{
  auto full = makeLine(10);    // x = 0..9
  auto out = RouteBridgeNode::sliceAroundEgo(
    full, /*ego_x=*/-3.0, /*ego_y=*/0.0, true, 5.0, 5.0);
  ASSERT_EQ(out.segments.size(), 1u);
  const auto & pts = out.segments[0].points;
  ASSERT_FALSE(pts.empty());
  EXPECT_DOUBLE_EQ(pts.front().x, 0.0);
}

TEST(SliceAroundEgo, EgoPastRouteEndGuardEnsuresTwoPoints)
{
  auto full = makeLine(10);
  auto out = RouteBridgeNode::sliceAroundEgo(
    full, /*ego_x=*/100.0, /*ego_y=*/0.0, true,
    /*behind_m=*/0.0, /*ahead_m=*/0.0);
  ASSERT_EQ(out.segments.size(), 1u);
  // ≥1-preceding-waypoint guard kicks in: nearest is point 9, win_start
  // gets bumped back to 8.
  ASSERT_EQ(out.segments[0].points.size(), 2u);
  EXPECT_DOUBLE_EQ(out.segments[0].points[0].x, 8.0);
  EXPECT_DOUBLE_EQ(out.segments[0].points[1].x, 9.0);
}

TEST(SliceAroundEgo, WindowCuttingLcZoneExtendsToEnclose)
{
  // Build a route where points 30..40 are an LC zone. Ego sits at 32, window
  // ahead/behind both small enough that the natural slice cuts both LC
  // markers — slicer must extend to include them.
  FMSRouteWindow w = makeLine(100);
  auto & pts = w.segments[0].points;
  pts[30].point_type = SegmentWaypoint::POINT_LC_START;
  pts[40].point_type = SegmentWaypoint::POINT_LC_END;

  auto out = RouteBridgeNode::sliceAroundEgo(
    w, /*ego_x=*/32.0, /*ego_y=*/0.0, true,
    /*behind_m=*/1.0, /*ahead_m=*/1.0);
  ASSERT_EQ(out.segments.size(), 1u);
  const auto & op = out.segments[0].points;
  ASSERT_FALSE(op.empty());
  EXPECT_DOUBLE_EQ(op.front().x, 30.0)
    << "win_start must extend back to LC_START";
  EXPECT_DOUBLE_EQ(op.back().x, 40.0)
    << "win_end must extend forward to LC_END";
}

TEST(SliceAroundEgo, OrientationFlipPreservesSegmentBoundary)
{
  // Build a route with a forward run then a reverse run sharing a boundary
  // point. The slicer must keep them as two FMSPathSegments with the boundary
  // point present in both.
  FMSRouteWindow w;
  w.status = FMSRouteWindow::STATUS_ACTIVE;
  FMSPathSegment fwd;
  fwd.orientation = FMSPathSegment::ORIENT_FORWARD;
  for (double x = 0.0; x <= 5.0; x += 1.0) fwd.points.push_back(pt(x, 0.0));
  FMSPathSegment rev;
  rev.orientation = FMSPathSegment::ORIENT_REVERSE;
  for (double x = 5.0; x <= 10.0; x += 1.0) rev.points.push_back(pt(x, 0.0));
  w.segments = {fwd, rev};

  // Ego near the boundary so the window spans both segments.
  auto out = RouteBridgeNode::sliceAroundEgo(
    w, /*ego_x=*/5.0, /*ego_y=*/0.0, true, 3.0, 3.0);
  ASSERT_EQ(out.segments.size(), 2u);
  EXPECT_EQ(out.segments[0].orientation, FMSPathSegment::ORIENT_FORWARD);
  EXPECT_EQ(out.segments[1].orientation, FMSPathSegment::ORIENT_REVERSE);
  EXPECT_DOUBLE_EQ(out.segments[0].points.back().x, 5.0);
  EXPECT_DOUBLE_EQ(out.segments[1].points.front().x, 5.0);
}

TEST(SliceAroundEgo, PreservesHeaderAndIdentity)
{
  auto full = makeLine(20);
  full.vehicle_id = "IGV2001";
  full.route_id = "R-id";
  full.job_id = "J-id";
  full.job_task_id = "T-id";
  full.header.frame_id = "map";
  full.status = FMSRouteWindow::STATUS_ACTIVE;

  auto out = RouteBridgeNode::sliceAroundEgo(full, 10.0, 0.0, true, 2.0, 2.0);
  EXPECT_EQ(out.vehicle_id, "IGV2001");
  EXPECT_EQ(out.route_id, "R-id");
  EXPECT_EQ(out.job_id, "J-id");
  EXPECT_EQ(out.job_task_id, "T-id");
  EXPECT_EQ(out.header.frame_id, "map");
  EXPECT_EQ(out.status, FMSRouteWindow::STATUS_ACTIVE);
}
