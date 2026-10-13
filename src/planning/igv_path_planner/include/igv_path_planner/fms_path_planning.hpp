// Copyright 2024 ZPMC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef IGV_PATH_PLANNER__FMS_PATH_PLANNING_HPP_
#define IGV_PATH_PLANNER__FMS_PATH_PLANNING_HPP_

#include "igv_planning_msgs/msg/fms_route_window.hpp"
#include "igv_path_planner/route_segment_selector.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace igv_path_planner::planning
{

enum class SubsegmentType
{
  Normal,
  LaneChange,
  LocalPlan,
  PreviousFrame,
  Pending,
};

struct Subsegment
{
  SubsegmentType type;
  size_t first;
  size_t last;
};

inline bool containsDestination(
  const igv_planning_msgs::msg::FMSPathSegment & segment)
{
  return std::any_of(
    segment.points.begin(), segment.points.end(),
    [](const auto & point) {
      return point.point_type ==
             igv_planning_msgs::msg::SegmentWaypoint::POINT_DESTINATION;
    });
}

/// Return the last segment that can be followed without stopping. Consecutive
/// segments with the same travel direction form one continuous motion path.
/// A destination always terminates that path, even if more data follows in the
/// sliding window.
inline size_t continuousMotionEndIndex(
  const igv_planning_msgs::msg::FMSRouteWindow & window, const size_t start_index)
{
  if (start_index >= window.segments.size()) {
    throw std::out_of_range("Route segment index is outside the route window");
  }

  size_t end_index = start_index;
  while (true) {
    if (containsDestination(window.segments[end_index])) {
      break;
    }
    if (end_index + 1 >= window.segments.size()) {
      break;
    }
    if (
      window.segments[end_index + 1].orientation !=
      window.segments[start_index].orientation)
    {
      break;
    }
    ++end_index;
  }
  return end_index;
}

/// A segment boundary is a stop boundary only for a real destination or a
/// forward/reverse transition. The ordinary edge of a sliding route window is
/// not a stop request.
inline bool requiresTerminalStop(
  const igv_planning_msgs::msg::FMSRouteWindow & window, const size_t start_index)
{
  const size_t end_index = continuousMotionEndIndex(window, start_index);
  if (containsDestination(window.segments[end_index])) {
    return true;
  }
  return end_index + 1 < window.segments.size() &&
         window.segments[end_index + 1].orientation !=
           window.segments[start_index].orientation;
}

inline size_t prependPreviousSegmentEndpoints(
  igv_planning_msgs::msg::FMSRouteWindow & window, const double connection_distance)
{
  size_t inserted = 0;
  for (size_t i = 1; i < window.segments.size(); ++i) {
    const auto & previous = window.segments[i - 1];
    auto & current = window.segments[i];
    if (previous.points.empty() || current.points.empty()) {
      continue;
    }

    const auto & previous_end = previous.points.back();
    const auto & current_start = current.points.front();
    if (std::hypot(previous_end.x - current_start.x, previous_end.y - current_start.y) <=
      connection_distance)
    {
      continue;
    }

    auto boundary = previous_end;
    boundary.heading = current_start.heading;
    boundary.speed_limit = current_start.speed_limit;
    boundary.point_type = igv_planning_msgs::msg::SegmentWaypoint::POINT_NORMAL;
    current.points.insert(current.points.begin(), boundary);
    ++inserted;
  }
  return inserted;
}

inline bool isLaneChangeStart(const uint8_t type)
{
  return type == igv_planning_msgs::msg::SegmentWaypoint::POINT_LC_START;
}

inline bool isLaneChangeEnd(const uint8_t type)
{
  return type == igv_planning_msgs::msg::SegmentWaypoint::POINT_LC_END;
}

inline bool isLocalPlanStart(const uint8_t type)
{
  return type == igv_planning_msgs::msg::SegmentWaypoint::POINT_LOCAL_PLAN_START;
}

inline bool isLocalPlanEnd(const uint8_t type)
{
  return type == igv_planning_msgs::msg::SegmentWaypoint::POINT_LOCAL_PLAN_END;
}

inline bool isNormalLikePoint(const uint8_t type)
{
  return type == igv_planning_msgs::msg::SegmentWaypoint::POINT_NORMAL ||
         type == igv_planning_msgs::msg::SegmentWaypoint::POINT_DESTINATION;
}

inline bool isEndMarker(const uint8_t type)
{
  return isLaneChangeEnd(type) || isLocalPlanEnd(type);
}

inline std::vector<Subsegment> splitSubsegments(
  const igv_planning_msgs::msg::FMSPathSegment & segment)
{
  std::vector<Subsegment> subsegments;
  size_t i = 0;

  while (i < segment.points.size()) {
    const auto type = segment.points[i].point_type;

    if (isLaneChangeStart(type) || isLocalPlanStart(type)) {
      const bool lane_change = isLaneChangeStart(type);
      const auto expected_end = lane_change
        ? igv_planning_msgs::msg::SegmentWaypoint::POINT_LC_END
        : igv_planning_msgs::msg::SegmentWaypoint::POINT_LOCAL_PLAN_END;

      size_t end = i + 1;
      while (end < segment.points.size() && segment.points[end].point_type != expected_end) {
        if (!isNormalLikePoint(segment.points[end].point_type)) {
          throw std::runtime_error(
            lane_change
            ? "Invalid lane-change subsegment marker order"
            : "Invalid local-plan subsegment marker order");
        }
        ++end;
      }
      if (end >= segment.points.size()) {
        subsegments.push_back({SubsegmentType::Pending, i, segment.points.size() - 1});
        break;
      }

      subsegments.push_back(
        {lane_change ? SubsegmentType::LaneChange : SubsegmentType::LocalPlan, i, end});
      i = end + 1;
      continue;
    }

    if (isEndMarker(type)) {
      if (subsegments.empty()) {
        subsegments.push_back({SubsegmentType::PreviousFrame, i, i});
        ++i;
        continue;
      }
      throw std::runtime_error("Invalid route segment: END marker appeared without START marker");
    }

    const size_t first = i;
    bool first_cropped_special_tail = false;
    while (i < segment.points.size()) {
      const auto normal_type = segment.points[i].point_type;
      if (isLaneChangeStart(normal_type) || isLocalPlanStart(normal_type)) {
        break;
      }
      if (isEndMarker(normal_type)) {
        first_cropped_special_tail = subsegments.empty();
        if (!first_cropped_special_tail) {
          throw std::runtime_error("Invalid route segment: END marker appeared without START marker");
        }
        ++i;
        break;
      }
      if (!isNormalLikePoint(normal_type)) {
        throw std::runtime_error("Invalid route segment marker inside NORMAL subsegment");
      }
      ++i;
    }
    if (i > first) {
      subsegments.push_back(
        {first_cropped_special_tail ? SubsegmentType::PreviousFrame : SubsegmentType::Normal,
         first, i - 1});
    }
  }

  return subsegments;
}

}  // namespace igv_path_planner::planning

#endif  // IGV_PATH_PLANNER__FMS_PATH_PLANNING_HPP_
