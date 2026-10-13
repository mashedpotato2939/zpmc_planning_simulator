#include "igv_path_planner/utils/path_planning_utils.hpp"

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace igv_path_planner::utils
{
namespace
{

double normalizeRadian(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double distance2d(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double segmentYaw(
  const autoware_planning_msgs::msg::PathPoint & front,
  const autoware_planning_msgs::msg::PathPoint & back)
{
  const double dx = back.pose.position.x - front.pose.position.x;
  const double dy = back.pose.position.y - front.pose.position.y;
  if (std::hypot(dx, dy) < 1e-6) {
    return tf2::getYaw(front.pose.orientation);
  }
  return std::atan2(dy, dx);
}

double squaredDistanceToSegment2d(
  const geometry_msgs::msg::Point & point,
  const geometry_msgs::msg::Point & front,
  const geometry_msgs::msg::Point & back)
{
  const double vx = back.x - front.x;
  const double vy = back.y - front.y;
  const double wx = point.x - front.x;
  const double wy = point.y - front.y;
  const double len_sq = vx * vx + vy * vy;

  if (len_sq < 1e-9) {
    return (point.x - front.x) * (point.x - front.x) +
           (point.y - front.y) * (point.y - front.y);
  }

  const double t = std::clamp((wx * vx + wy * vy) / len_sq, 0.0, 1.0);
  const double proj_x = front.x + t * vx;
  const double proj_y = front.y + t * vy;
  return (point.x - proj_x) * (point.x - proj_x) +
         (point.y - proj_y) * (point.y - proj_y);
}

double signedDistanceToSegmentStart(
  const PathPoints & points,
  const size_t seg_idx,
  const geometry_msgs::msg::Point & target_point)
{
  if (points.size() < 2) {
    return 0.0;
  }

  const size_t front_idx = std::min(seg_idx, points.size() - 2);
  const auto & front = points[front_idx].pose.position;
  const auto & back = points[front_idx + 1].pose.position;
  const double vx = back.x - front.x;
  const double vy = back.y - front.y;
  const double len = std::hypot(vx, vy);

  if (len < 1e-9) {
    return 0.0;
  }

  const double wx = target_point.x - front.x;
  const double wy = target_point.y - front.y;
  return (wx * vx + wy * vy) / len;
}

double arcLengthToIndex(
  const PathPoints & points,
  const geometry_msgs::msg::Point & target_point,
  const size_t target_seg_idx,
  const size_t point_idx)
{
  const size_t seg_idx = std::min(target_seg_idx, points.size() - 2);
  double length = 0.0;

  if (point_idx <= seg_idx) {
    length = signedDistanceToSegmentStart(points, seg_idx, target_point);
    for (size_t i = point_idx; i < seg_idx; ++i) {
      length += distance2d(points[i].pose.position, points[i + 1].pose.position);
    }
    return -length;
  }

  length = distance2d(target_point, points[seg_idx + 1].pose.position);
  for (size_t i = seg_idx + 1; i < point_idx; ++i) {
    length += distance2d(points[i].pose.position, points[i + 1].pose.position);
  }
  return length;
}

}  // namespace

size_t findFirstNearestSegmentIndexWithSoftConstraints(
  const PathPoints & points,
  const geometry_msgs::msg::Pose & pose,
  const double dist_threshold,
  const double yaw_threshold)
{
  if (points.size() < 2) {
    return 0;
  }

  const double pose_yaw = tf2::getYaw(pose.orientation);
  const double dist_sq_threshold = dist_threshold * dist_threshold;
  size_t best_idx = 0;
  double best_dist_sq = std::numeric_limits<double>::max();
  size_t fallback_idx = 0;
  double fallback_dist_sq = std::numeric_limits<double>::max();

  for (size_t i = 0; i + 1 < points.size(); ++i) {
    const double dist_sq = squaredDistanceToSegment2d(
      pose.position, points[i].pose.position, points[i + 1].pose.position);
    if (dist_sq < fallback_dist_sq) {
      fallback_dist_sq = dist_sq;
      fallback_idx = i;
    }

    const double yaw = segmentYaw(points[i], points[i + 1]);
    const double yaw_diff = std::abs(normalizeRadian(pose_yaw - yaw));
    if (dist_sq <= dist_sq_threshold && yaw_diff <= yaw_threshold && dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      best_idx = i;
    }
  }

  return best_dist_sq < std::numeric_limits<double>::max() ? best_idx : fallback_idx;
}

double distance2D(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  return distance2d(a, b);
}

std::string baseRouteId(const std::string & route_id)
{
  const auto pos = route_id.find('#');
  return (pos == std::string::npos) ? route_id : route_id.substr(0, pos);
}

geometry_msgs::msg::Quaternion createQuaternionFromYaw(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(yaw / 2.0);
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  return q;
}

double getYaw(const geometry_msgs::msg::Quaternion & q)
{
  return tf2::getYaw(q);
}

geometry_msgs::msg::Point interpolatePoint(
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & end,
  const double ratio)
{
  const double t = std::clamp(ratio, 0.0, 1.0);
  geometry_msgs::msg::Point out;
  out.x = start.x + t * (end.x - start.x);
  out.y = start.y + t * (end.y - start.y);
  out.z = start.z + t * (end.z - start.z);
  return out;
}

autoware_planning_msgs::msg::PathPoint interpolatePathPoint(
  const autoware_planning_msgs::msg::PathPoint & start,
  const autoware_planning_msgs::msg::PathPoint & end,
  const double ratio)
{
  const double t = std::clamp(ratio, 0.0, 1.0);

  auto out = start;
  out.pose.position.x = start.pose.position.x +
    t * (end.pose.position.x - start.pose.position.x);
  out.pose.position.y = start.pose.position.y +
    t * (end.pose.position.y - start.pose.position.y);
  out.pose.position.z = start.pose.position.z +
    t * (end.pose.position.z - start.pose.position.z);

  const double yaw0 = tf2::getYaw(start.pose.orientation);
  const double yaw1 = tf2::getYaw(end.pose.orientation);
  const double yaw = yaw0 + t * normalizeRadian(yaw1 - yaw0);
  out.pose.orientation = createQuaternionFromYaw(yaw);

  out.longitudinal_velocity_mps = static_cast<float>(
    start.longitudinal_velocity_mps +
    t * (end.longitudinal_velocity_mps - start.longitudinal_velocity_mps));
  out.lateral_velocity_mps = static_cast<float>(
    start.lateral_velocity_mps +
    t * (end.lateral_velocity_mps - start.lateral_velocity_mps));
  out.heading_rate_rps = static_cast<float>(
    start.heading_rate_rps + t * (end.heading_rate_rps - start.heading_rate_rps));
  out.is_final = false;
  return out;
}

void shiftResolvedRangesAfterReplacement(
  std::vector<std::pair<size_t, size_t>> & ranges,
  const size_t replaced_end_idx,
  const std::ptrdiff_t delta)
{
  if (delta == 0) {
    return;
  }

  const auto shift_index = [delta](const size_t idx) {
    if (delta < 0) {
      const auto magnitude = static_cast<size_t>(-delta);
      return idx > magnitude ? idx - magnitude : static_cast<size_t>(0);
    }
    return idx + static_cast<size_t>(delta);
  };

  for (auto & range : ranges) {
    if (range.first <= replaced_end_idx) {
      continue;
    }
    range.first = shift_index(range.first);
    range.second = shift_index(range.second);
  }
}

bool truncatePathFromEgoPose(
  autoware_planning_msgs::msg::Path & path,
  const geometry_msgs::msg::Pose & ego_pose,
  const double nearest_dist_threshold,
  const double nearest_yaw_threshold,
  const double back_buffer_length,
  const double min_projection_distance)
{
  if (path.points.size() < 2) return false;

  size_t ego_seg_idx = findFirstNearestSegmentIndexWithSoftConstraints(
    path.points, ego_pose, nearest_dist_threshold, nearest_yaw_threshold);
  ego_seg_idx = std::min(ego_seg_idx, path.points.size() - 2);

  const auto & seg_start = path.points[ego_seg_idx];
  const auto & seg_end = path.points[ego_seg_idx + 1];
  const auto & p0 = seg_start.pose.position;
  const auto & p1 = seg_end.pose.position;
  const double dx = p1.x - p0.x;
  const double dy = p1.y - p0.y;
  const double dz = p1.z - p0.z;
  const double seg_len_sq = dx * dx + dy * dy + dz * dz;
  const bool have_bounds =
    path.left_bound.size() == path.points.size() &&
    path.right_bound.size() == path.points.size();

  if (seg_len_sq < 1e-6) {
    if (ego_seg_idx == 0) return false;
    path.points.erase(path.points.begin(), path.points.begin() + ego_seg_idx);
    if (have_bounds) {
      path.left_bound.erase(path.left_bound.begin(), path.left_bound.begin() + ego_seg_idx);
      path.right_bound.erase(path.right_bound.begin(), path.right_bound.begin() + ego_seg_idx);
    } else {
      path.left_bound.clear();
      path.right_bound.clear();
    }
    return true;
  }

  const auto & ego = ego_pose.position;
  const double raw_ratio =
    ((ego.x - p0.x) * dx + (ego.y - p0.y) * dy + (ego.z - p0.z) * dz) / seg_len_sq;
  const double ratio = std::clamp(raw_ratio, 0.0, 1.0);
  const double back_buffer = std::max(0.0, back_buffer_length);
  const double ego_segment_len = distance2D(p0, p1);

  size_t start_seg_idx = ego_seg_idx;
  double start_ratio = ratio;
  if (back_buffer > 0.0 && ego_segment_len > 1e-6) {
    double remaining_back = back_buffer;
    const double dist_from_seg_start_to_ego = ratio * ego_segment_len;

    if (remaining_back <= dist_from_seg_start_to_ego || ego_seg_idx == 0) {
      start_ratio = std::clamp(
        (dist_from_seg_start_to_ego - remaining_back) / ego_segment_len, 0.0, 1.0);
    } else {
      remaining_back -= dist_from_seg_start_to_ego;
      bool found_start = false;
      size_t candidate_seg = ego_seg_idx;
      while (candidate_seg > 0) {
        const auto & prev = path.points[candidate_seg - 1].pose.position;
        const auto & curr = path.points[candidate_seg].pose.position;
        const double seg_len = distance2D(prev, curr);
        --candidate_seg;
        if (seg_len < 1e-6) continue;

        if (remaining_back <= seg_len) {
          start_seg_idx = candidate_seg;
          start_ratio = std::clamp((seg_len - remaining_back) / seg_len, 0.0, 1.0);
          found_start = true;
          break;
        }
        remaining_back -= seg_len;
      }

      if (!found_start) {
        start_seg_idx = 0;
        start_ratio = 0.0;
      }
    }
  }

  const auto start_point =
    interpolatePathPoint(path.points[start_seg_idx], path.points[start_seg_idx + 1], start_ratio);

  const double min_dist = std::max(0.0, min_projection_distance);
  const auto & start_seg_start = path.points[start_seg_idx].pose.position;
  const auto & start_seg_end = path.points[start_seg_idx + 1].pose.position;
  const double dist_to_start = distance2D(start_point.pose.position, start_seg_start);
  const double dist_to_end = distance2D(start_point.pose.position, start_seg_end);

  std::vector<autoware_planning_msgs::msg::PathPoint> truncated;
  std::vector<geometry_msgs::msg::Point> truncated_left;
  std::vector<geometry_msgs::msg::Point> truncated_right;
  truncated.reserve(path.points.size() - start_seg_idx + 1);
  if (have_bounds) {
    truncated_left.reserve(path.left_bound.size() - start_seg_idx + 1);
    truncated_right.reserve(path.right_bound.size() - start_seg_idx + 1);
  }

  if (dist_to_start <= min_dist) {
    truncated.insert(truncated.end(), path.points.begin() + start_seg_idx, path.points.end());
    if (have_bounds) {
      truncated_left.insert(
        truncated_left.end(), path.left_bound.begin() + start_seg_idx, path.left_bound.end());
      truncated_right.insert(
        truncated_right.end(), path.right_bound.begin() + start_seg_idx, path.right_bound.end());
    }
  } else if (dist_to_end <= min_dist) {
    truncated.insert(truncated.end(), path.points.begin() + start_seg_idx + 1, path.points.end());
    if (have_bounds) {
      truncated_left.insert(
        truncated_left.end(), path.left_bound.begin() + start_seg_idx + 1, path.left_bound.end());
      truncated_right.insert(
        truncated_right.end(), path.right_bound.begin() + start_seg_idx + 1, path.right_bound.end());
    }
  } else {
    truncated.push_back(start_point);
    truncated.insert(truncated.end(), path.points.begin() + start_seg_idx + 1, path.points.end());
    if (have_bounds) {
      truncated_left.push_back(
        interpolatePoint(
          path.left_bound[start_seg_idx], path.left_bound[start_seg_idx + 1], start_ratio));
      truncated_left.insert(
        truncated_left.end(), path.left_bound.begin() + start_seg_idx + 1, path.left_bound.end());
      truncated_right.push_back(
        interpolatePoint(
          path.right_bound[start_seg_idx], path.right_bound[start_seg_idx + 1], start_ratio));
      truncated_right.insert(
        truncated_right.end(), path.right_bound.begin() + start_seg_idx + 1, path.right_bound.end());
    }
  }

  if (truncated.empty()) {
    truncated.push_back(path.points.back());
    if (have_bounds) {
      truncated_left.push_back(path.left_bound.back());
      truncated_right.push_back(path.right_bound.back());
    }
  }

  const bool changed =
    truncated.size() != path.points.size() ||
    distance2D(truncated.front().pose.position, path.points.front().pose.position) > 1e-6;
  path.points = std::move(truncated);
  if (have_bounds) {
    path.left_bound = std::move(truncated_left);
    path.right_bound = std::move(truncated_right);
  } else {
    path.left_bound.clear();
    path.right_bound.clear();
  }
  return changed;
}

size_t findNearestIndex(
  const autoware_planning_msgs::msg::Path & path,
  const geometry_msgs::msg::Point & point)
{
  size_t nearest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < path.points.size(); ++i) {
    const double dist = distance2D(path.points[i].pose.position, point);
    if (dist < min_dist) {
      min_dist = dist;
      nearest_idx = i;
    }
  }

  return nearest_idx;
}

double computeCurvatureAtIndex(
  const PathPoints & points,
  const size_t index)
{
  if (points.size() < 3) {
    return 0.0;
  }

  size_t i0;
  size_t i1;
  size_t i2;
  if (index == 0) {
    i0 = 0;
    i1 = 1;
    i2 = 2;
  } else if (index >= points.size() - 1) {
    i0 = points.size() - 3;
    i1 = points.size() - 2;
    i2 = points.size() - 1;
  } else {
    i0 = index - 1;
    i1 = index;
    i2 = index + 1;
  }

  const auto & p0 = points[i0].pose.position;
  const auto & p1 = points[i1].pose.position;
  const auto & p2 = points[i2].pose.position;

  const double a = std::hypot(p1.x - p0.x, p1.y - p0.y);
  const double b = std::hypot(p2.x - p1.x, p2.y - p1.y);
  const double c = std::hypot(p2.x - p0.x, p2.y - p0.y);

  if (a < 1e-6 || b < 1e-6 || c < 1e-6) {
    return 0.0;
  }

  const double cross = (p1.x - p0.x) * (p2.y - p0.y) -
    (p1.y - p0.y) * (p2.x - p0.x);
  const double area = 0.5 * std::abs(cross);
  return 4.0 * area / (a * b * c);
}

double calculateMinimumLaneWidth(
  const double curvature,
  const double vehicle_length,
  const double vehicle_width,
  const double vehicle_front_overhang,
  const double vehicle_wheel_base,
  const double min_bound_margin,
  const double max_width)
{
  if (std::abs(curvature) < 1e-6) {
    return vehicle_width + 2.0 * min_bound_margin;
  }

  const double radius = 1.0 / std::abs(curvature);
  const double rear_overhang = vehicle_length - vehicle_front_overhang - vehicle_wheel_base;

  const double front_outer_radius = std::sqrt(
    std::pow(radius + vehicle_width / 2.0, 2) +
    std::pow(vehicle_wheel_base + vehicle_front_overhang, 2));
  const double rear_outer_radius = std::sqrt(
    std::pow(radius + vehicle_width / 2.0, 2) +
    std::pow(rear_overhang, 2));

  const double outer_sweep_radius = std::max(front_outer_radius, rear_outer_radius);
  double min_width = 2.0 * (outer_sweep_radius - radius) + vehicle_width;
  min_width += 2.0 * min_bound_margin;

  return std::min(min_width, max_width);
}

void smoothDrivableBounds(
  autoware_planning_msgs::msg::Path & path,
  const double max_width_change_rate)
{
  if (path.left_bound.size() < 3 || path.right_bound.size() < 3) {
    return;
  }

  auto smooth_bounds = [](std::vector<geometry_msgs::msg::Point> & bounds) {
    std::vector<geometry_msgs::msg::Point> smoothed = bounds;
    const size_t n = bounds.size();

    for (size_t i = 0; i < n; ++i) {
      const size_t window_start = (i >= 2) ? (i - 2) : 0;
      const size_t window_end = (i + 2 < n) ? (i + 2) : (n - 1);

      double sum_x = 0.0;
      double sum_y = 0.0;
      double sum_z = 0.0;
      size_t count = 0;

      for (size_t j = window_start; j <= window_end; ++j) {
        sum_x += bounds[j].x;
        sum_y += bounds[j].y;
        sum_z += bounds[j].z;
        ++count;
      }

      if (count > 0) {
        smoothed[i].x = sum_x / count;
        smoothed[i].y = sum_y / count;
        smoothed[i].z = sum_z / count;
      }
    }

    bounds = std::move(smoothed);
  };

  smooth_bounds(path.left_bound);
  smooth_bounds(path.right_bound);

  const double max_rate = std::max(0.0, max_width_change_rate);
  auto apply_rate_limit = [max_rate](
    std::vector<geometry_msgs::msg::Point> & bounds,
    const PathPoints & path_points) {
    if (bounds.size() != path_points.size()) {
      return;
    }

    for (size_t i = 1; i < bounds.size(); ++i) {
      const double dx_curr = bounds[i].x - path_points[i].pose.position.x;
      const double dy_curr = bounds[i].y - path_points[i].pose.position.y;
      const double dist_curr = std::hypot(dx_curr, dy_curr);

      const double dx_prev = bounds[i - 1].x - path_points[i - 1].pose.position.x;
      const double dy_prev = bounds[i - 1].y - path_points[i - 1].pose.position.y;
      const double dist_prev = std::hypot(dx_prev, dy_prev);

      const double change = dist_curr - dist_prev;
      if (std::abs(change) > max_rate) {
        const double limited_dist = dist_prev + std::copysign(max_rate, change);
        const double scale = (dist_curr > 1e-6) ? (limited_dist / dist_curr) : 1.0;

        bounds[i].x = path_points[i].pose.position.x + dx_curr * scale;
        bounds[i].y = path_points[i].pose.position.y + dy_curr * scale;
      }
    }
  };

  apply_rate_limit(path.left_bound, path.points);
  apply_rate_limit(path.right_bound, path.points);
}

double calcLateralOffset(
  const PathPoints & points,
  const geometry_msgs::msg::Point & target_point)
{
  if (points.size() < 2) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  geometry_msgs::msg::Pose pose;
  pose.position = target_point;
  pose.orientation = points.front().pose.orientation;
  const size_t seg_idx = findFirstNearestSegmentIndexWithSoftConstraints(
    points, pose, std::numeric_limits<double>::max(), std::numeric_limits<double>::max());

  const auto & front = points[seg_idx].pose.position;
  const auto & back = points[seg_idx + 1].pose.position;
  const double vx = back.x - front.x;
  const double vy = back.y - front.y;
  const double len = std::hypot(vx, vy);
  if (len < 1e-9) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const double wx = target_point.x - front.x;
  const double wy = target_point.y - front.y;
  return (vx * wy - vy * wx) / len;
}

PathPoints cropPoints(
  const PathPoints & points,
  const geometry_msgs::msg::Point & target_point,
  const size_t target_seg_idx,
  const double forward_length,
  const double backward_length)
{
  if (points.size() < 2) {
    return points;
  }

  const size_t seg_idx = std::min(target_seg_idx, points.size() - 2);
  size_t begin_idx = 0;
  size_t end_idx = points.size();

  for (size_t i = 0; i < points.size(); ++i) {
    if (arcLengthToIndex(points, target_point, seg_idx, i) >= -backward_length) {
      begin_idx = i;
      break;
    }
  }

  for (size_t i = seg_idx + 1; i < points.size(); ++i) {
    if (arcLengthToIndex(points, target_point, seg_idx, i) > forward_length) {
      end_idx = i + 1;
      break;
    }
  }

  if (end_idx <= begin_idx + 1) {
    return points;
  }

  return PathPoints{points.begin() + begin_idx, points.begin() + end_idx};
}

}  // namespace igv_path_planner::utils
