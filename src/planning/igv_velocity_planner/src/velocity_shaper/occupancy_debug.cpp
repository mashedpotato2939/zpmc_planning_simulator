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

#include "igv_velocity_planner/velocity_shaper/occupancy_debug.hpp"

#include "igv_velocity_planner/velocity_shaper/geometry_utils.hpp"

#include <geometry_msgs/msg/point.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace igv_velocity_planner::velocity_shaper
{

namespace
{
using Marker = visualization_msgs::msg::Marker;
using TrajectoryPoint = autoware_planning_msgs::msg::TrajectoryPoint;

struct DistanceSample
{
  geometry_msgs::msg::Point point;
  double s{0.0};
  double distance{0.0};
};

struct CorridorSample
{
  geometry_msgs::msg::Point point;
  double s{0.0};
};

double distance2d(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::hypot(dx, dy);
}

std::vector<CorridorSample> makeCorridorSamples(
  const std::vector<TrajectoryPoint> & grid_frame_points, double distance_limit)
{
  std::vector<CorridorSample> samples;
  if (grid_frame_points.empty()) return samples;

  samples.reserve(grid_frame_points.size());
  samples.push_back(CorridorSample{grid_frame_points.front().pose.position, 0.0});

  double s_accum = 0.0;
  for (size_t i = 1; i < grid_frame_points.size(); ++i) {
    const auto & p0 = grid_frame_points[i - 1].pose.position;
    const auto & p1 = grid_frame_points[i].pose.position;
    const double seg_len = distance2d(p0, p1);
    if (seg_len <= 1e-9) continue;

    const double next_s = s_accum + seg_len;
    if (next_s <= distance_limit + 1e-9) {
      samples.push_back(CorridorSample{p1, next_s});
      s_accum = next_s;
      continue;
    }

    if (s_accum < distance_limit) {
      const double ratio = (distance_limit - s_accum) / seg_len;
      geometry_msgs::msg::Point clipped;
      clipped.x = p0.x + ratio * (p1.x - p0.x);
      clipped.y = p0.y + ratio * (p1.y - p0.y);
      clipped.z = p0.z + ratio * (p1.z - p0.z);
      samples.push_back(CorridorSample{clipped, distance_limit});
    }
    break;
  }

  return samples;
}

Marker makeLineMarker(
  const std_msgs::msg::Header & header, const std::string & ns, int32_t id, float r, float g,
  float b, float alpha, double width)
{
  Marker marker;
  marker.header = header;
  marker.ns = ns;
  marker.id = id;
  marker.type = Marker::LINE_STRIP;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = width;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = alpha;
  return marker;
}

Marker makeDeleteAllMarker(const std_msgs::msg::Header & header)
{
  Marker marker;
  marker.header = header;
  marker.ns = "fms_velocity_shaper/occupancy_corridor";
  marker.action = Marker::DELETEALL;
  return marker;
}

Marker makeMinPointMarker(
  const std_msgs::msg::Header & header, const DistanceSample & sample)
{
  Marker marker;
  marker.header = header;
  marker.ns = "fms_velocity_shaper/occupancy_corridor_min_point";
  marker.id = 0;
  marker.type = Marker::SPHERE;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.pose.position = sample.point;
  marker.scale.x = 0.6;
  marker.scale.y = 0.6;
  marker.scale.z = 0.6;
  marker.color.r = 1.0f;
  marker.color.g = 0.15f;
  marker.color.b = 0.15f;
  marker.color.a = 0.9f;
  return marker;
}

Marker makeTextMarker(
  const std_msgs::msg::Header & header, const DistanceSample & sample, int32_t id,
  const std::string & prefix, float r, float g, float b)
{
  Marker marker;
  marker.header = header;
  marker.ns = "fms_velocity_shaper/occupancy_corridor_distance_text";
  marker.id = id;
  marker.type = Marker::TEXT_VIEW_FACING;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.pose.position = sample.point;
  marker.pose.position.z += 1.0;
  marker.scale.z = 0.45;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0f;

  char text[64];
  std::snprintf(text, sizeof(text), "%sd=%.2fm", prefix.c_str(), sample.distance);
  marker.text = text;
  return marker;
}
}  // namespace

std::optional<nav_msgs::msg::OccupancyGrid> makeOccupancyDistanceMapGrid(
  const nav_msgs::msg::OccupancyGrid & source_grid, const DistanceMap & distance_map,
  double display_distance_cap)
{
  if (!distance_map.valid() || source_grid.info.width == 0 || source_grid.info.height == 0) {
    return std::nullopt;
  }

  const size_t cell_count =
    static_cast<size_t>(source_grid.info.width) * static_cast<size_t>(source_grid.info.height);
  if (source_grid.data.size() != cell_count) return std::nullopt;

  nav_msgs::msg::OccupancyGrid out;
  out.header = source_grid.header;
  out.info = source_grid.info;
  out.data.assign(cell_count, 100);

  const double cap = std::max(display_distance_cap, std::numeric_limits<double>::epsilon());
  for (uint32_t y = 0; y < distance_map.roiHeight(); ++y) {
    const uint32_t grid_y = distance_map.roiCellY0() + y;
    if (grid_y >= source_grid.info.height) continue;
    for (uint32_t x = 0; x < distance_map.roiWidth(); ++x) {
      const uint32_t grid_x = distance_map.roiCellX0() + x;
      if (grid_x >= source_grid.info.width) continue;

      const double d = std::clamp(distance_map.distanceAtCell(x, y) / cap, 0.0, 1.0);
      out.data[static_cast<size_t>(grid_y) * source_grid.info.width + grid_x] =
        static_cast<int8_t>(std::lround(d * 100.0));
    }
  }

  return out;
}

visualization_msgs::msg::MarkerArray makeOccupancyCorridorMarkers(
  const std_msgs::msg::Header & header, const nav_msgs::msg::OccupancyGrid & source_grid,
  const DistanceMap & distance_map, const std::vector<TrajectoryPoint> & grid_frame_points,
  double ego_half_width, double max_distance, double label_distance_cap)
{
  visualization_msgs::msg::MarkerArray markers;
  if (!distance_map.valid() || grid_frame_points.size() < 2) return markers;

  Marker center = makeLineMarker(
    header, "fms_velocity_shaper/occupancy_corridor_center", 1, 0.05f, 0.35f, 1.0f, 0.9f, 0.08);
  Marker left = makeLineMarker(
    header, "fms_velocity_shaper/occupancy_corridor_boundary", 2, 0.0f, 0.85f, 0.25f, 0.9f,
    0.06);
  Marker right = makeLineMarker(
    header, "fms_velocity_shaper/occupancy_corridor_boundary", 3, 0.0f, 0.85f, 0.25f, 0.9f,
    0.06);

  const double half_width = std::max(0.0, ego_half_width);
  const double distance_limit = max_distance > 0.0 ? max_distance : std::numeric_limits<double>::max();
  const double label_cap = std::max(label_distance_cap, 0.0);
  const auto samples = makeCorridorSamples(grid_frame_points, distance_limit);
  if (samples.size() < 2) return markers;

  const double ox = source_grid.info.origin.position.x;
  const double oy = source_grid.info.origin.position.y;
  const double grid_yaw = yawFromQuat(source_grid.info.origin.orientation);
  const double cy = std::cos(-grid_yaw);
  const double sy = std::sin(-grid_yaw);
  const auto to_local = [&](double wx, double wy, double & lx, double & ly) {
    const double dx = wx - ox;
    const double dy = wy - oy;
    lx = cy * dx - sy * dy;
    ly = sy * dx + cy * dy;
  };

  const size_t n = samples.size();
  std::vector<double> lx(n), ly(n);
  for (size_t i = 0; i < n; ++i) {
    to_local(samples[i].point.x, samples[i].point.y, lx[i], ly[i]);
  }

  std::vector<DistanceSample> distance_samples;
  distance_samples.reserve(n);

  for (size_t i = 0; i < samples.size(); ++i) {
    double tx = 0.0;
    double ty = 0.0;
    if (i + 1 < samples.size()) {
      tx = samples[i + 1].point.x - samples[i].point.x;
      ty = samples[i + 1].point.y - samples[i].point.y;
    } else {
      tx = samples[i].point.x - samples[i - 1].point.x;
      ty = samples[i].point.y - samples[i - 1].point.y;
    }

    const double tn = std::hypot(tx, ty);
    double px = 0.0;
    double py = 0.0;
    if (tn > 1e-9) {
      px = -ty / tn;
      py = tx / tn;
    }

    double local_tx = 0.0;
    double local_ty = 0.0;
    if (i + 1 < samples.size()) {
      local_tx = lx[i + 1] - lx[i];
      local_ty = ly[i + 1] - ly[i];
    } else {
      local_tx = lx[i] - lx[i - 1];
      local_ty = ly[i] - ly[i - 1];
    }

    const double local_tn = std::hypot(local_tx, local_ty);
    double local_px = 0.0;
    double local_py = 0.0;
    if (local_tn > 1e-9) {
      local_px = -local_ty / local_tn;
      local_py = local_tx / local_tn;
    }

    double d_min = std::numeric_limits<double>::infinity();
    for (const double off : {-half_width, 0.0, half_width}) {
      d_min = std::min(
        d_min, distance_map.queryMetric(lx[i] + off * local_px, ly[i] + off * local_py));
    }

    const auto p = samples[i].point;
    center.points.push_back(p);
    if (std::isfinite(d_min)) distance_samples.push_back(DistanceSample{p, samples[i].s, d_min});

    auto lp = p;
    lp.x += half_width * px;
    lp.y += half_width * py;
    left.points.push_back(lp);

    auto rp = p;
    rp.x -= half_width * px;
    rp.y -= half_width * py;
    right.points.push_back(rp);
  }

  if (center.points.size() < 2) return markers;

  markers.markers.push_back(makeDeleteAllMarker(header));
  markers.markers.push_back(std::move(center));
  markers.markers.push_back(std::move(left));
  markers.markers.push_back(std::move(right));

  if (!distance_samples.empty()) {
    const auto closest = std::min_element(
      distance_samples.begin(), distance_samples.end(),
      [](const auto & a, const auto & b) { return a.distance < b.distance; });
    const double closest_s = closest->s;
    markers.markers.push_back(makeMinPointMarker(header, *closest));
    markers.markers.push_back(
      makeTextMarker(header, *closest, 100, "min ", 1.0f, 0.82f, 0.1f));

    constexpr double kLabelSpacingM = 5.0;
    double last_label_s = -std::numeric_limits<double>::infinity();
    int32_t id = 101;
    for (const auto & sample : distance_samples) {
      if (sample.distance > label_cap) continue;
      if (std::abs(sample.s - closest_s) < 1e-6) continue;
      if (sample.s - last_label_s < kLabelSpacingM) continue;
      markers.markers.push_back(makeTextMarker(header, sample, id++, "", 1.0f, 1.0f, 1.0f));
      last_label_s = sample.s;
    }
  }
  return markers;
}

}  // namespace igv_velocity_planner::velocity_shaper
