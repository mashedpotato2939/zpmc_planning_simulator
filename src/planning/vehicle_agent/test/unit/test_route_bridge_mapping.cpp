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
using IrMission = igv_planning_msgs::msg::IrMission;
using IrCommand = igv_planning_msgs::msg::IrCommand;
using IrWaypoint = igv_planning_msgs::msg::IrWaypoint;
using FMSPathSegment = igv_planning_msgs::msg::FMSPathSegment;
using FMSRouteWindow = igv_planning_msgs::msg::FMSRouteWindow;
using FMSStopPoint = igv_planning_msgs::msg::FMSStopPoint;
using SegmentWaypoint = igv_planning_msgs::msg::SegmentWaypoint;

namespace
{
IrWaypoint wp(double x, double y, const std::string & segid,
              double yaw = 0.0, double speed = 1.5,
              uint8_t point_type = SegmentWaypoint::POINT_NORMAL)
{
  IrWaypoint w;
  w.x = x;
  w.y = y;
  w.z = 0.0;
  w.yaw = yaw;
  w.target_speed = speed;
  w.segment_id = segid;
  w.point_type = point_type;
  return w;
}

rclcpp::Time anyStamp() { return rclcpp::Time(123456789, 0); }
}  // namespace

TEST(RouteBridgeMapping, SingleForwardSegment)
{
  IrMission m;
  m.id = "R-001";
  m.kind = IrMission::KIND_ROUTE;
  m.waypoints = {wp(10.0, 1.0, "seg0:fwd"),
                 wp(11.0, 1.0, "seg0:fwd"),
                 wp(12.0, 1.0, "seg0:fwd")};

  const auto out = RouteBridgeNode::translateMission(m, "IGV2001", anyStamp());
  EXPECT_EQ(out.vehicle_id, "IGV2001");
  EXPECT_EQ(out.route_id, "R-001");
  EXPECT_EQ(out.status, FMSRouteWindow::STATUS_ACTIVE);
  ASSERT_EQ(out.segments.size(), 1u);
  EXPECT_EQ(out.segments[0].orientation, FMSPathSegment::ORIENT_FORWARD);
  ASSERT_EQ(out.segments[0].points.size(), 3u);
  EXPECT_DOUBLE_EQ(out.segments[0].points[0].x, 10.0);
  EXPECT_DOUBLE_EQ(out.segments[0].points[2].x, 12.0);
  // Last point of last segment is the destination.
  EXPECT_EQ(out.segments[0].points.back().point_type,
            SegmentWaypoint::POINT_DESTINATION);
  EXPECT_EQ(out.segments[0].points.front().point_type,
            SegmentWaypoint::POINT_NORMAL);
}

TEST(RouteBridgeMapping, MixedForwardReverseSegments)
{
  IrMission m;
  m.id = "R-mixed";
  m.kind = IrMission::KIND_ROUTE;
  m.waypoints = {
    wp(0.0, 0.0, "seg0:fwd"),
    wp(1.0, 0.0, "seg0:fwd"),
    wp(2.0, 0.0, "seg0:fwd"),
    wp(2.5, 0.0, "seg1:rev"),
    wp(3.0, 0.0, "seg1:rev"),
  };

  const auto out = RouteBridgeNode::translateMission(m, "IGV2001", anyStamp());
  ASSERT_EQ(out.segments.size(), 2u);
  EXPECT_EQ(out.segments[0].orientation, FMSPathSegment::ORIENT_FORWARD);
  EXPECT_EQ(out.segments[1].orientation, FMSPathSegment::ORIENT_REVERSE);
  ASSERT_EQ(out.segments[0].points.size(), 3u);
  ASSERT_EQ(out.segments[1].points.size(), 2u);
  EXPECT_EQ(out.segments[1].points.back().point_type,
            SegmentWaypoint::POINT_DESTINATION);
  EXPECT_EQ(out.segments[0].points.back().point_type,
            SegmentWaypoint::POINT_NORMAL)
    << "only the final point of the FINAL segment should be DESTINATION";
}

TEST(RouteBridgeMapping, LocalPlanMarkersPropagate)
{
  // The Q-turn scenario: fms_path_converter only invokes its A* local replan
  // when it sees POINT_LOCAL_PLAN_START / POINT_LOCAL_PLAN_END on the
  // SegmentWaypoint. Confirm the bridge preserves them verbatim.
  IrMission m;
  m.id = "R-qturn";
  m.kind = IrMission::KIND_ROUTE;
  m.waypoints = {
    wp(14.0, -6.0, "seg0:fwd", 2.45, 5.0, SegmentWaypoint::POINT_NORMAL),
    wp(11.0, -3.0, "seg0:fwd", 2.45, 5.0, SegmentWaypoint::POINT_LOCAL_PLAN_START),
    wp(-1.6, -2.8, "seg0:fwd", -2.27, 5.0, SegmentWaypoint::POINT_LOCAL_PLAN_END),
    wp(-4.7, -6.4, "seg0:fwd", -2.26, 5.0, SegmentWaypoint::POINT_DESTINATION),
  };

  const auto out = RouteBridgeNode::translateMission(m, "IGV2001", anyStamp());
  ASSERT_EQ(out.segments.size(), 1u);
  const auto & pts = out.segments[0].points;
  ASSERT_EQ(pts.size(), 4u);
  EXPECT_EQ(pts[0].point_type, SegmentWaypoint::POINT_NORMAL);
  EXPECT_EQ(pts[1].point_type, SegmentWaypoint::POINT_LOCAL_PLAN_START);
  EXPECT_EQ(pts[2].point_type, SegmentWaypoint::POINT_LOCAL_PLAN_END);
  EXPECT_EQ(pts[3].point_type, SegmentWaypoint::POINT_DESTINATION);
}

TEST(RouteBridgeMapping, ExplicitDestinationSuppressesAutoTag)
{
  // When a waypoint already carries POINT_DESTINATION, the auto-promote
  // fallback must not double-mark the final point.
  IrMission m;
  m.id = "R-explicit-dest";
  m.kind = IrMission::KIND_ROUTE;
  m.waypoints = {
    wp(0.0, 0.0, "seg0:fwd", 0.0, 1.5, SegmentWaypoint::POINT_NORMAL),
    wp(1.0, 0.0, "seg0:fwd", 0.0, 1.5, SegmentWaypoint::POINT_DESTINATION),
    wp(2.0, 0.0, "seg0:fwd", 0.0, 0.0, SegmentWaypoint::POINT_POI),
  };

  const auto out = RouteBridgeNode::translateMission(m, "IGV2001", anyStamp());
  ASSERT_EQ(out.segments.size(), 1u);
  const auto & pts = out.segments[0].points;
  ASSERT_EQ(pts.size(), 3u);
  EXPECT_EQ(pts[1].point_type, SegmentWaypoint::POINT_DESTINATION);
  EXPECT_EQ(pts[2].point_type, SegmentWaypoint::POINT_POI)
    << "auto-promote must not overwrite producer-set point_type once any "
       "waypoint in the route already carries POINT_DESTINATION";
}

TEST(RouteBridgeMapping, MissingSegmentIdCollapsesToOneForward)
{
  IrMission m;
  m.id = "R-legacy";
  m.kind = IrMission::KIND_ROUTE;
  m.waypoints = {wp(0.0, 0.0, ""), wp(1.0, 0.0, ""), wp(2.0, 0.0, "")};

  const auto out = RouteBridgeNode::translateMission(m, "IGV2001", anyStamp());
  ASSERT_EQ(out.segments.size(), 1u);
  EXPECT_EQ(out.segments[0].orientation, FMSPathSegment::ORIENT_FORWARD);
  EXPECT_EQ(out.segments[0].points.size(), 3u);
}

TEST(RouteBridgeMapping, AbortMissionEmitsEmptyCancelled)
{
  IrMission m;
  m.id = "R-die";
  m.kind = IrMission::KIND_ABORT;
  m.waypoints = {wp(0.0, 0.0, "seg0:fwd")};   // should be ignored

  const auto out = RouteBridgeNode::translateMission(m, "IGV2001", anyStamp());
  EXPECT_EQ(out.status, FMSRouteWindow::STATUS_CANCELLED);
  EXPECT_TRUE(out.segments.empty());
}

TEST(RouteBridgeMapping, ResumeMissionAlsoCancelled)
{
  // KIND_RESUME on a mission is unusual (it's really a Command in IR), but
  // if it shows up the legacy converter has no resume semantics — treat it
  // as a route cancellation so the converter clears state.
  IrMission m;
  m.id = "R-resume";
  m.kind = IrMission::KIND_RESUME;
  const auto out = RouteBridgeNode::translateMission(m, "IGV2001", anyStamp());
  EXPECT_EQ(out.status, FMSRouteWindow::STATUS_CANCELLED);
}

TEST(RouteBridgeMapping, FieldsRoundTrip)
{
  IrMission m;
  m.id = "R-fields";
  m.kind = IrMission::KIND_ROUTE;
  m.waypoints = {wp(7.5, -2.25, "seg0:fwd", /*yaw=*/1.3, /*speed=*/2.75)};
  const auto out = RouteBridgeNode::translateMission(m, "v77", anyStamp());
  ASSERT_EQ(out.segments.size(), 1u);
  ASSERT_EQ(out.segments[0].points.size(), 1u);
  const auto & p = out.segments[0].points[0];
  EXPECT_DOUBLE_EQ(p.x, 7.5);
  EXPECT_DOUBLE_EQ(p.y, -2.25);
  EXPECT_DOUBLE_EQ(p.heading, 1.3);
  EXPECT_DOUBLE_EQ(p.speed_limit, 2.75);
  EXPECT_EQ(out.vehicle_id, "v77");
}

TEST(RouteBridgeMapping, PauseStopPointUsesEgoPose)
{
  IrCommand c;
  c.mission_id = "R-1";
  c.kind = IrCommand::KIND_PAUSE;
  const auto sp = RouteBridgeNode::translateCommand(
    c, "IGV2001", /*x=*/12.5, /*y=*/-3.0, /*have_ego=*/true, anyStamp());
  ASSERT_TRUE(sp.has_value());
  EXPECT_EQ(sp->vehicle_id, "IGV2001");
  EXPECT_DOUBLE_EQ(sp->x, 12.5);
  EXPECT_DOUBLE_EQ(sp->y, -3.0);
  EXPECT_FALSE(sp->clear);
}

TEST(RouteBridgeMapping, ResumeStopPointHasClearTrue)
{
  IrCommand c;
  c.kind = IrCommand::KIND_RESUME;
  const auto sp = RouteBridgeNode::translateCommand(
    c, "IGV2001", 0.0, 0.0, true, anyStamp());
  ASSERT_TRUE(sp.has_value());
  EXPECT_TRUE(sp->clear);
}

TEST(RouteBridgeMapping, EstopAndClearFaultDropPublish)
{
  for (uint8_t k : {IrCommand::KIND_ESTOP, IrCommand::KIND_CLEAR_FAULT}) {
    IrCommand c;
    c.kind = k;
    const auto sp = RouteBridgeNode::translateCommand(
      c, "IGV2001", 0.0, 0.0, true, anyStamp());
    EXPECT_FALSE(sp.has_value()) << "kind=" << static_cast<int>(k);
  }
}

TEST(RouteBridgeMapping, PauseWithoutEgoFallsBackToZero)
{
  IrCommand c;
  c.kind = IrCommand::KIND_PAUSE;
  const auto sp = RouteBridgeNode::translateCommand(
    c, "IGV2001", 12.5, -3.0, /*have_ego=*/false, anyStamp());
  ASSERT_TRUE(sp.has_value());
  EXPECT_DOUBLE_EQ(sp->x, 0.0);
  EXPECT_DOUBLE_EQ(sp->y, 0.0);
  EXPECT_FALSE(sp->clear);
}
