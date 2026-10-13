#include <gtest/gtest.h>
#include "igv_path_planner/fms_path_planning.hpp"
#include "igv_path_planner/route_segment_selector.hpp"
#include <cmath>
using igv_path_planner::planning::RouteSegmentSelector;
using igv_path_planner::planning::SubsegmentType;
using Segment = igv_planning_msgs::msg::FMSPathSegment;
using Window = igv_planning_msgs::msg::FMSRouteWindow;
Segment segment(int start, int end, bool reverse=false) {
  Segment s; s.orientation=reverse ? Segment::ORIENT_REVERSE : Segment::ORIENT_FORWARD;
  const int step=end>start ? 1 : -1;
  for(int x=start;;x+=step) {
    igv_planning_msgs::msg::SegmentWaypoint p; p.x=x;
    p.heading=step>0 ? 0.0 : M_PI; s.points.push_back(p);
    if(x==end) break;
  }
  return s;
}
geometry_msgs::msg::Pose pose(double x, double yaw = 0.0) {
  geometry_msgs::msg::Pose p; p.position.x=x;
  p.orientation.w=std::cos(yaw * 0.5);
  p.orientation.z=std::sin(yaw * 0.5);
  return p;
}
TEST(SegmentTracker, PartitionDoesNotAdvance) {
  RouteSegmentSelector t; Window w; w.segments={segment(0,20),segment(20,40)};
  ASSERT_TRUE(t.select(w,"a",pose(5),1,0).valid);
  w.route_id="a#p1";
  auto r=t.select(w,"a",pose(6),1,1); ASSERT_TRUE(r.valid); EXPECT_EQ(r.index,0u);
}
TEST(SegmentTracker, WindowDropChangesIndexNotExecution) {
  RouteSegmentSelector t; Window w; w.segments={segment(0,10),segment(10,30)};
  ASSERT_EQ(t.select(w,"a",pose(15),1,0).index,1u);
  w.segments={segment(12,30)};
  auto r=t.select(w,"a",pose(16),1,1); ASSERT_TRUE(r.valid); EXPECT_EQ(r.index,0u);
}
TEST(SegmentTracker, SameDirectionBoundary) {
  RouteSegmentSelector t; Window w; w.segments={segment(0,10),segment(10,20)};
  ASSERT_TRUE(t.select(w,"a",pose(5),1,0).valid);
  auto r=t.select(w,"a",pose(9.7),1,1); ASSERT_TRUE(r.valid); EXPECT_EQ(r.index,1u);
}
TEST(SegmentTracker, AmbiguousOverlapIsNotGuessed) {
  RouteSegmentSelector t; Window w; w.segments={segment(0,10),segment(10,0,true)};
  EXPECT_FALSE(t.select(w,"a",pose(5),0,0).valid);
}
TEST(SegmentTracker, ReverseRequiresContinuousStop) {
  RouteSegmentSelector t; Window w; w.segments={segment(0,10)};
  ASSERT_TRUE(t.select(w,"a",pose(5),1,0).valid);
  w.segments.push_back(segment(10,0,true));
  EXPECT_EQ(t.select(w,"a",pose(10),1,1).index,0u);
  EXPECT_EQ(t.select(w,"a",pose(10),0,2).index,0u);
  EXPECT_EQ(t.select(w,"a",pose(10),0.3,2.5).index,0u);
  EXPECT_EQ(t.select(w,"a",pose(10),0,3).index,0u);
  auto r=t.select(w,"a",pose(10),0,4.1); ASSERT_TRUE(r.valid); EXPECT_EQ(r.index,1u);
  r=t.select(w,"a",pose(9),0.5,5); ASSERT_TRUE(r.valid); EXPECT_EQ(r.index,1u);
}
TEST(SegmentTracker, NoJumpToFutureOverlappingSegment) {
  RouteSegmentSelector t; Window w; w.segments={segment(0,20)};
  ASSERT_TRUE(t.select(w,"a",pose(5),1,0).valid);
  w.segments.push_back(segment(20,0,true));
  auto r=t.select(w,"a",pose(6),1,1); ASSERT_TRUE(r.valid); EXPECT_EQ(r.index,0u);
}
TEST(SegmentTracker, MissingHistoryAndEmptySegmentsHold) {
  RouteSegmentSelector t; Window w; w.segments={segment(0,10)};
  ASSERT_TRUE(t.select(w,"a",pose(5),1,0).valid);
  w.segments={segment(20,30)};
  EXPECT_FALSE(t.select(w,"a",pose(25),1,1).valid);
  w.segments={Segment{}}; EXPECT_FALSE(t.select(w,"b",pose(0),0,2).valid);
}
TEST(SegmentTracker, BacktrackRejectedAndNewRouteResets) {
  RouteSegmentSelector t; Window w; w.segments={segment(0,20)};
  ASSERT_TRUE(t.select(w,"a",pose(10),1,0).valid);
  EXPECT_FALSE(t.select(w,"a",pose(5),1,1).valid);
  EXPECT_TRUE(t.select(w,"b",pose(5),1,2).valid);
}
TEST(SegmentTracker, LastWindowEndpointDoesNotAdvance) {
  RouteSegmentSelector t; Window w; w.segments={segment(0,10)};
  ASSERT_TRUE(t.select(w,"a",pose(8),1,0).valid);
  auto r=t.select(w,"a",pose(10),0,5); ASSERT_TRUE(r.valid); EXPECT_EQ(r.index,0u);
  w.segments={segment(5,20)};
  r=t.select(w,"a",pose(11),1,6); ASSERT_TRUE(r.valid); EXPECT_EQ(r.index,0u);
}

TEST(SegmentTerminalStop, OrdinarySlidingWindowEndKeepsMoving) {
  Window w; w.segments={segment(0,20)};
  EXPECT_EQ(igv_path_planner::planning::continuousMotionEndIndex(w,0),0u);
  EXPECT_FALSE(igv_path_planner::planning::requiresTerminalStop(w,0));
}

TEST(SegmentTerminalStop, SameDirectionSuccessorsFormOneMotionPath) {
  Window w; w.segments={segment(0,20),segment(20,40),segment(40,60)};
  EXPECT_EQ(igv_path_planner::planning::continuousMotionEndIndex(w,0),2u);
  EXPECT_FALSE(igv_path_planner::planning::requiresTerminalStop(w,0));
}

TEST(SegmentTerminalStop, DirectionChangeRequiresStop) {
  Window w; w.segments={segment(0,20),segment(20,0,true)};
  EXPECT_EQ(igv_path_planner::planning::continuousMotionEndIndex(w,0),0u);
  EXPECT_TRUE(igv_path_planner::planning::requiresTerminalStop(w,0));
}

TEST(SegmentTerminalStop, DestinationRequiresStopBeforeLaterData) {
  Window w; w.segments={segment(0,20),segment(20,40)};
  w.segments[0].points.back().point_type =
    igv_planning_msgs::msg::SegmentWaypoint::POINT_DESTINATION;
  EXPECT_EQ(igv_path_planner::planning::continuousMotionEndIndex(w,0),0u);
  EXPECT_TRUE(igv_path_planner::planning::requiresTerminalStop(w,0));
}

TEST(SegmentTracker, ConnectorToFirstPointAllowsDistantPathAhead) {
  RouteSegmentSelector t;
  t.parameters.max_distance = 3.0;
  t.parameters.connector_max_distance = 25.0;
  t.parameters.connector_max_yaw_error = M_PI / 4.0;
  Window w; w.segments={segment(20,40)};
  const auto r = t.select(w,"a",pose(0),1,0);
  ASSERT_TRUE(r.valid);
  EXPECT_EQ(r.index,0u);
}

TEST(SegmentTracker, ConnectorToFirstPointRejectsWrongHeading) {
  RouteSegmentSelector t;
  t.parameters.max_distance = 3.0;
  t.parameters.connector_max_distance = 25.0;
  t.parameters.connector_max_yaw_error = M_PI / 4.0;
  Window w; w.segments={segment(20,40)};
  EXPECT_FALSE(t.select(w,"a",pose(0, M_PI),1,0).valid);
}

TEST(SegmentBoundaryNormalization, PrependsPreviousEndpointAcrossGap) {
  Window w;
  w.segments = {segment(0, 10), segment(12, 20, true)};
  w.segments[0].points.back().point_type =
    igv_planning_msgs::msg::SegmentWaypoint::POINT_LOCAL_PLAN_END;
  w.segments[1].points.front().heading = -0.75;
  w.segments[1].points.front().speed_limit = 2.5;

  const auto inserted =
    igv_path_planner::planning::prependPreviousSegmentEndpoints(w, 0.5);

  ASSERT_EQ(inserted, 1u);
  ASSERT_EQ(w.segments[1].points.front().x, 10.0);
  EXPECT_DOUBLE_EQ(w.segments[1].points.front().heading, -0.75);
  EXPECT_DOUBLE_EQ(w.segments[1].points.front().speed_limit, 2.5);
  EXPECT_EQ(
    w.segments[1].points.front().point_type,
    igv_planning_msgs::msg::SegmentWaypoint::POINT_NORMAL);
}

TEST(SegmentBoundaryNormalization, LeavesConnectedSegmentsUnchanged) {
  Window w;
  w.segments = {segment(0, 10), segment(10, 20, true)};
  const auto original_size = w.segments[1].points.size();

  EXPECT_EQ(igv_path_planner::planning::prependPreviousSegmentEndpoints(w, 0.5), 0u);
  EXPECT_EQ(w.segments[1].points.size(), original_size);
}

TEST(SubsegmentSplit, CroppedLocalPlanTailUsesPreviousFrame) {
  auto s = segment(0, 4);
  s.points.back().point_type =
    igv_planning_msgs::msg::SegmentWaypoint::POINT_LOCAL_PLAN_END;
  const auto parts = igv_path_planner::planning::splitSubsegments(s);
  ASSERT_EQ(parts.size(), 1u);
  EXPECT_EQ(parts.front().type, SubsegmentType::PreviousFrame);
  EXPECT_EQ(parts.front().first, 0u);
  EXPECT_EQ(parts.front().last, s.points.size() - 1);
}

TEST(SubsegmentSplit, CroppedLocalPlanHeadIsPending) {
  auto s = segment(0, 4);
  s.points[2].point_type =
    igv_planning_msgs::msg::SegmentWaypoint::POINT_LOCAL_PLAN_START;
  const auto parts = igv_path_planner::planning::splitSubsegments(s);
  ASSERT_EQ(parts.size(), 2u);
  EXPECT_EQ(parts[0].type, SubsegmentType::Normal);
  EXPECT_EQ(parts[0].first, 0u);
  EXPECT_EQ(parts[0].last, 1u);
  EXPECT_EQ(parts[1].type, SubsegmentType::Pending);
  EXPECT_EQ(parts[1].first, 2u);
  EXPECT_EQ(parts[1].last, s.points.size() - 1);
}

TEST(SubsegmentSplit, StartedSpecialSegmentStillRequiresMatchingEnd) {
  auto s = segment(0, 2);
  s.points[0].point_type =
    igv_planning_msgs::msg::SegmentWaypoint::POINT_LOCAL_PLAN_START;
  s.points[2].point_type =
    igv_planning_msgs::msg::SegmentWaypoint::POINT_LC_END;
  EXPECT_THROW(
    igv_path_planner::planning::splitSubsegments(s),
    std::runtime_error);
}
