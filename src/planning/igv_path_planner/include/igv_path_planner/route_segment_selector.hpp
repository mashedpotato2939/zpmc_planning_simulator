#ifndef IGV_PATH_PLANNER__ROUTE_SEGMENT_SELECTOR_HPP_
#define IGV_PATH_PLANNER__ROUTE_SEGMENT_SELECTOR_HPP_

#include <igv_planning_msgs/msg/fms_route_window.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <iostream>

namespace igv_path_planner::planning
{
// Tracks execution, not the rolling #p revision. No stable segment IDs are
// available: association requires two distinct shared waypoints. If that
// evidence disappears, hold instead of guessing a geometrically nearby branch.
class RouteSegmentSelector
{
  using Segment = igv_planning_msgs::msg::FMSPathSegment;
  using Window = igv_planning_msgs::msg::FMSRouteWindow;
  struct Projection {
    double distance{std::numeric_limits<double>::infinity()};
    double yaw_error{0.0};
    double s{0.0};
    double length{std::numeric_limits<double>::infinity()};
  };
public:
  struct Parameters {
    double max_distance{5.0};
    double max_yaw_error{1.57};
    double connector_max_distance{50.0};
    double connector_max_yaw_error{0.78539816339};
    double ambiguity_margin{0.25};
    double endpoint_distance{0.5};
    double stopped_speed{0.1};
    double stopped_duration{1.0};
    double backtrack_tolerance{1.0};
    double progress_slack{2.0};
  };
  Parameters parameters;
  struct Result {
    bool valid{false};
    bool changed{false};
    size_t index{0};
    std::string reason;
  };

  void reset() { initialized_ = false; stopped_since_.reset(); }
  size_t current(const Window &) const { return index_; }

  Result select(const Window & window, const std::string & base,
    const geometry_msgs::msg::Pose & ego, double speed, double time)
  {
    if (base != base_) { reset(); base_ = base; }
    const auto fail = [this](const std::string & reason) {
      stopped_since_.reset();
      return Result{false, false, 0, reason};
    };
    if (!std::isfinite(speed) || !std::isfinite(time) ||
      !std::isfinite(ego.position.x) || !std::isfinite(ego.position.y)) {
      return fail("Invalid localization");
    }
    size_t selected = 0;
    if (!initialized_) {
      double best = std::numeric_limits<double>::infinity();
      double second = best;
      for (size_t i = 0; i < window.segments.size(); ++i) {
        const auto q = project(window.segments[i], ego);
        const auto c = projectConnectorToStart(window.segments[i], ego);
        const bool on_segment = acceptable(q);
        const bool on_connector = acceptableConnector(c);
        if (!on_segment && !on_connector) continue;
        const double score = on_segment ? q.distance + q.yaw_error : c.length + c.yaw_error;
        if (score < best) { second = best; best = score; selected = i; }
        else second = std::min(second, score);
      }
      if (!std::isfinite(best)) return fail("No segment matches vehicle pose");
      if (second - best <= parameters.ambiguity_margin) {
        return fail("Ambiguous initial segment; execution history required");
      }
    } else {
      size_t count = 0;
      for (size_t i = 0; i < window.segments.size(); ++i) {
        if (overlaps(reference_, window.segments[i])) { selected = i; ++count; }
      }

      if (count != 1) 
      {
        std::cout << "stamp=" << window.header.stamp.sec << "."
          << window.header.stamp.nanosec
          << " route_id=" << window.route_id
          << " segments=" << window.segments.size()
          << std::endl;
        for(int i=0; i<window.segments.size(); ++i)
        {
          std::cout << "window.segments[" << i << "].size = " << window.segments[i].points.size() << std::endl;
        }
        return fail("Current segment cannot be uniquely associated with window");
      }
    }
    const auto & seg = window.segments[selected];
    const auto q = project(seg, ego);
    const auto c = projectConnectorToStart(seg, ego);
    const bool on_segment = acceptable(q);
    const bool on_connector = acceptableConnector(c);
    if (!on_segment && !on_connector) {
      return fail("Vehicle is outside current segment matching limits");
    }
    if (initialized_ && on_segment) {
      // Compare both poses in the OLD geometry: window-local s changes as VA crops.
      const double delta = project(reference_, ego).s - project(reference_, last_pose_).s;
      const double displacement = std::hypot(
        ego.position.x - last_pose_.position.x, ego.position.y - last_pose_.position.y);
      if (delta < -parameters.backtrack_tolerance ||
        delta > displacement + parameters.progress_slack) {
        return fail("Discontinuous segment progress");
      }
    }
    bool switched = false;
    // Only a visible successor proves this endpoint is a segment boundary;
    // the last point of a sliding window is not a segment-completion signal.
    if (initialized_ && on_segment && selected + 1 < window.segments.size() &&
      q.length - q.s <= parameters.endpoint_distance &&
      distance(ego.position.x, ego.position.y, seg.points.back().x,
        seg.points.back().y) <= parameters.endpoint_distance)
    {
      const auto & next = window.segments[selected + 1];
      const bool connected = next.points.size() >= 2 &&
        distance(seg.points.back().x, seg.points.back().y,
          next.points.front().x, next.points.front().y) <= parameters.endpoint_distance;
      const bool reverse_change = next.orientation != seg.orientation;
      if (connected && acceptable(project(next, ego))) {
        if (!reverse_change) switched = true;
        else if (std::abs(speed) <= parameters.stopped_speed) {
          if (!stopped_since_ || time < *stopped_since_) stopped_since_ = time;
          switched = time - *stopped_since_ >= parameters.stopped_duration;
        } else stopped_since_.reset();
      } else stopped_since_.reset();
    } else stopped_since_.reset();
    if (switched) { ++selected; stopped_since_.reset(); }
    const bool changed = !initialized_ || switched || reference_ != window.segments[selected];
    reference_ = window.segments[selected];
    index_ = selected;
    last_pose_ = ego;
    initialized_ = true;
    return {true, changed, selected, {}};
  }
private:
  static double distance(double x, double y, double a, double b) {
    return std::hypot(x-a, y-b);
  }
  static double angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
  static double yawOfPose(const geometry_msgs::msg::Pose & pose) {
    const auto & q = pose.orientation;
    return std::atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z));
  }
  static Projection project(const Segment & seg, const geometry_msgs::msg::Pose & ego) {
    Projection out;
    if (seg.points.size() < 2) return out;
    const double yaw = yawOfPose(ego);
    double s = 0.0;
    for (size_t i=1; i<seg.points.size(); ++i) {
      const auto & a = seg.points[i-1]; const auto & b = seg.points[i];
      const double dx=b.x-a.x, dy=b.y-a.y, len=std::hypot(dx,dy);
      if (len < 1e-6) continue;
      const double t=std::clamp(((ego.position.x-a.x)*dx+(ego.position.y-a.y)*dy)/(len*len),0.0,1.0);
      const double d=distance(ego.position.x,ego.position.y,a.x+t*dx,a.y+t*dy);
      if (d < out.distance) {
        out.distance=d; out.s=s+t*len;
        const double heading=a.heading+t*angle(b.heading-a.heading)+
          (seg.orientation==Segment::ORIENT_REVERSE ? M_PI : 0.0);
        out.yaw_error=std::abs(angle(yaw-heading));
      }
      s+=len;
    }
    out.length=s;
    return out;
  }
  static Projection projectConnectorToStart(const Segment & seg, const geometry_msgs::msg::Pose & ego) {
    Projection out;
    if (seg.points.empty()) return out;
    const auto & first = seg.points.front();
    const double dx = first.x - ego.position.x;
    const double dy = first.y - ego.position.y;
    const double len = std::hypot(dx, dy);
    out.distance = 0.0;
    out.s = 0.0;
    out.length = len;
    if (len < 1e-6) {
      out.yaw_error = 0.0;
      return out;
    }
    const double travel_yaw = std::atan2(dy, dx);
    const double heading = travel_yaw +
      (seg.orientation == Segment::ORIENT_REVERSE ? M_PI : 0.0);
    out.yaw_error = std::abs(angle(yawOfPose(ego) - heading));
    return out;
  }
  bool acceptable(const Projection & q) const {
    return q.distance <= parameters.max_distance && q.yaw_error <= parameters.max_yaw_error;
  }
  bool acceptableConnector(const Projection & q) const {
    return std::isfinite(q.length) && std::isfinite(q.yaw_error) &&
      q.length <= parameters.connector_max_distance &&
      q.yaw_error <= parameters.connector_max_yaw_error;
  }
  static bool overlaps(const Segment & a, const Segment & b) {
    if (a.orientation != b.orientation) return false;
    std::optional<std::pair<double,double>> first;
    size_t next_index=0;
    for (const auto & p : a.points) {
      for (size_t j=next_index; j<b.points.size(); ++j) {
        const auto & q=b.points[j];
        if (distance(p.x,p.y,q.x,q.y)<0.05 && std::abs(angle(p.heading-q.heading))<0.1) {
          if (first && distance(p.x,p.y,first->first,first->second)>0.1) return true;
          first=std::make_pair(p.x,p.y); next_index=j+1; break;
        }
      }
    }
    return false;
  }
  bool initialized_{false};
  size_t index_{0};
  std::string base_;
  Segment reference_;
  geometry_msgs::msg::Pose last_pose_;
  std::optional<double> stopped_since_;
};
}  // namespace igv_path_planner::planning
#endif
