// Copyright 2024 ZPMC
// Licensed under the Apache License, Version 2.0
#include "igv_path_planner/map_path_generator.hpp"
#include "igv_path_planner/path_generator.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <sstream>
#include <unordered_set>

namespace igv_path_planner
{
namespace
{
using Vec3 = MapPathGenerator::Vec3;
double distance2d(const Vec3 & a, const Vec3 & b) { return std::hypot(a.x - b.x, a.y - b.y); }
Vec3 lerp(const Vec3 & a, const Vec3 & b, double t)
{
  return {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.z + t * (b.z - a.z)};
}
std::vector<uint64_t> parseIds(const std::string & value)
{
  std::vector<uint64_t> ids;
  std::stringstream input(value);
  std::string token;
  while (std::getline(input, token, ',')) {
    try { if (!token.empty()) ids.push_back(std::stoull(token)); }
    catch (const std::exception &) {}
  }
  return ids;
}
}  // namespace

MapPathGenerator::MapPathGenerator(const Parameters & params) : params_(params) {}

void MapPathGenerator::setMap(const lanelet::LaneletMapPtr & map)
{
  lanes_.clear();
  lane_index_.clear();
  has_elevation_ = false;
  if (!map) return;
  for (const auto & ll : map->laneletLayer) {
    if (ll.centerline().size() < 2) continue;
    Lane lane;
    lane.id = ll.id();
    for (const auto & p : ll.centerline()) lane.center.push_back({p.x(), p.y(), p.z()});
    for (const auto & p : lane.center) {
      if (std::abs(p.z) > 0.5) has_elevation_ = true;
    }
    for (const auto & p : ll.leftBound()) lane.left.push_back({p.x(), p.y(), p.z()});
    for (const auto & p : ll.rightBound()) lane.right.push_back({p.x(), p.y(), p.z()});
    lane.s.push_back(0.0);
    for (size_t i = 1; i < lane.center.size(); ++i) {
      lane.s.push_back(lane.s.back() + distance2d(lane.center[i - 1], lane.center[i]));
    }
    if (ll.hasAttribute("speed_limit")) {
      try { lane.speed_limit = std::stod(ll.attribute("speed_limit").value()); }
      catch (const std::exception &) {}
    }
    lane_index_[lane.id] = lanes_.size();
    lanes_.push_back(std::move(lane));
  }
  std::unordered_map<uint64_t, LaneId> zpmc_ids;
  for (const auto & ll : map->laneletLayer) {
    if (ll.hasAttribute("zpmc:lane_id")) {
      try { zpmc_ids[std::stoull(ll.attribute("zpmc:lane_id").value())] = ll.id(); }
      catch (const std::exception &) {}
    }
  }
  for (const auto & ll : map->laneletLayer) {
    const auto found = lane_index_.find(ll.id());
    if (found == lane_index_.end()) continue;
    auto & lane = lanes_[found->second];
    const auto add = [&zpmc_ids](const std::string & value, std::vector<LaneId> & out) {
      for (auto id : parseIds(value)) {
        const auto mapped = zpmc_ids.find(id);
        if (mapped != zpmc_ids.end()) out.push_back(mapped->second);
      }
    };
    if (ll.hasAttribute("zpmc:successor_ids")) {
      add(ll.attribute("zpmc:successor_ids").value(), lane.successors);
    }
    if (ll.hasAttribute("zpmc:predecessor_ids")) {
      add(ll.attribute("zpmc:predecessor_ids").value(), lane.predecessors);
    }
  }
  // Lanelet2 maps without ZPMC topology tags connect at shared endpoints.
  for (auto & a : lanes_) {
    for (const auto & b : lanes_) {
      if (a.id == b.id) continue;
      if (distance2d(a.center.back(), b.center.front()) < 0.5 &&
          std::abs(a.center.back().z - b.center.front().z) < 1.0) {
        if (std::find(a.successors.begin(), a.successors.end(), b.id) == a.successors.end())
          a.successors.push_back(b.id);
      }
      if (distance2d(a.center.front(), b.center.back()) < 0.5 &&
          std::abs(a.center.front().z - b.center.back().z) < 1.0) {
        if (std::find(a.predecessors.begin(), a.predecessors.end(), b.id) == a.predecessors.end())
          a.predecessors.push_back(b.id);
      }
    }
  }
}

MapPathGenerator::Projection MapPathGenerator::projectWaypoint(double x, double y, double z) const
{
  Projection best;
  best.distance = params_.snap_distance_threshold;
  for (size_t li = 0; li < lanes_.size(); ++li) {
    const auto & lane = lanes_[li];
    for (size_t i = 1; i < lane.center.size(); ++i) {
      const auto & a = lane.center[i - 1];
      const auto & b = lane.center[i];
      const double dx = b.x - a.x, dy = b.y - a.y;
      const double length_sq = dx * dx + dy * dy;
      if (length_sq < 1e-9) continue;
      const double t = std::clamp(((x - a.x) * dx + (y - a.y) * dy) / length_sq, 0.0, 1.0);
      auto p = lerp(a, b, t);
      const double dist = std::hypot(x - p.x, y - p.y);
      // Some OSM exports, including nanhui_double_deck_fat_v2, flatten all
      // lane elevations to zero while their PCD retains absolute height.
      if (has_elevation_ && z != 0.0 &&
          std::abs(z - p.z) > params_.snap_distance_threshold) continue;
      if (dist < best.distance || (!best.valid && dist <= best.distance)) {
        if (!has_elevation_) p.z = z;
        best = {li, lane.s[i - 1] + t * (lane.s[i] - lane.s[i - 1]), p, dist, true};
      }
    }
  }
  return best;
}

MapPathGenerator::Vec3 MapPathGenerator::atS(const Lane & lane, double s)
{
  if (s <= 0.0) return lane.center.front();
  if (s >= lane.s.back()) return lane.center.back();
  const auto it = std::lower_bound(lane.s.begin(), lane.s.end(), s);
  const size_t i = static_cast<size_t>(it - lane.s.begin());
  const double ds = lane.s[i] - lane.s[i - 1];
  return lerp(lane.center[i - 1], lane.center[i], ds > 0 ? (s - lane.s[i - 1]) / ds : 0.0);
}

std::vector<MapPathGenerator::LaneId> MapPathGenerator::findLaneSequenceByTopology(
  LaneId from, LaneId to, const LaneNeighborProvider & successors,
  const LaneNeighborProvider & predecessors, bool * follows_successors)
{
  if (from == to) { if (follows_successors) *follows_successors = true; return {from}; }
  for (bool forward : {true, false}) {
    const auto & neighbors = forward ? successors : predecessors;
    std::queue<std::vector<LaneId>> queue;
    std::unordered_set<LaneId> visited{from};
    queue.push({from});
    while (!queue.empty()) {
      auto path = queue.front(); queue.pop();
      if (path.size() > 20) continue;
      for (const auto next : neighbors(path.back())) {
        if (next == to) {
          path.push_back(next);
          if (follows_successors) *follows_successors = forward;
          return path;
        }
        if (visited.insert(next).second) {
          auto copy = path; copy.push_back(next); queue.push(std::move(copy));
        }
      }
    }
  }
  return {};
}

std::vector<MapPathGenerator::LaneId> MapPathGenerator::findLaneSequenceByEndpointContinuity(
  LaneId from, const Vec3 & from_first, const Vec3 & from_last,
  LaneId to, const Vec3 & to_first, const Vec3 & to_last,
  double max_distance, bool * follows_successors)
{
  if (distance2d(from_last, to_first) <= max_distance &&
      std::abs(from_last.z - to_first.z) <= max_distance) {
    if (follows_successors) *follows_successors = true;
    return {from, to};
  }
  if (distance2d(from_first, to_last) <= max_distance &&
      std::abs(from_first.z - to_last.z) <= max_distance) {
    if (follows_successors) *follows_successors = false;
    return {from, to};
  }
  return {};
}

std::vector<size_t> MapPathGenerator::findLaneSequence(
  size_t from, size_t to, bool * follows_successors) const
{
  auto neighbors = [this](LaneId id, bool successor) {
    const auto it = lane_index_.find(id);
    if (it == lane_index_.end()) return std::vector<LaneId>{};
    return successor ? lanes_[it->second].successors : lanes_[it->second].predecessors;
  };
  auto ids = findLaneSequenceByTopology(
    lanes_[from].id, lanes_[to].id,
    [&neighbors](LaneId id) { return neighbors(id, true); },
    [&neighbors](LaneId id) { return neighbors(id, false); }, follows_successors);
  if (ids.empty()) {
    ids = findLaneSequenceByEndpointContinuity(
      lanes_[from].id, lanes_[from].center.front(), lanes_[from].center.back(),
      lanes_[to].id, lanes_[to].center.front(), lanes_[to].center.back(),
      1.0, follows_successors);
  }
  std::vector<size_t> result;
  for (const auto id : ids) {
    const auto it = lane_index_.find(id);
    if (it != lane_index_.end()) result.push_back(it->second);
  }
  return result;
}

void MapPathGenerator::resampleAndOrient(
  const std::vector<Vec3> & slice, double interval, bool forward, bool reverse,
  double velocity, std::vector<autoware_planning_msgs::msg::PathPoint> & out)
{
  out.clear();
  if (slice.size() < 2 || interval <= 0.0) return;
  std::vector<Vec3> sampled{slice.front()};
  double accumulated = 0.0;
  for (size_t i = 1; i < slice.size(); ++i) {
    accumulated += distance2d(slice[i - 1], slice[i]);
    if (accumulated >= interval || i + 1 == slice.size()) {
      if (distance2d(sampled.back(), slice[i]) >= 0.05) sampled.push_back(slice[i]);
      accumulated = 0.0;
    }
  }
  if (!forward) std::reverse(sampled.begin(), sampled.end());
  if (sampled.size() < 2) return;
  for (const auto & p : sampled) {
    autoware_planning_msgs::msg::PathPoint pt;
    pt.pose.position.x = p.x; pt.pose.position.y = p.y; pt.pose.position.z = p.z;
    pt.longitudinal_velocity_mps = static_cast<float>(reverse ? -velocity : velocity);
    out.push_back(pt);
  }
  for (size_t i = 0; i + 1 < out.size(); ++i) {
    const auto & a = out[i].pose.position;
    const auto & b = out[i + 1].pose.position;
    out[i].pose.orientation = quaternionFromYaw(std::atan2(b.y - a.y, b.x - a.x) + (reverse ? M_PI : 0.0));
  }
  if (out.size() > 1) out.back().pose.orientation = out[out.size() - 2].pose.orientation;
}

void MapPathGenerator::extractLane(
  size_t index, double from_s, double to_s, bool reverse, double velocity,
  SnapResult & result) const
{
  const auto & lane = lanes_[index];
  const bool forward = from_s <= to_s;
  const double lo = std::min(from_s, to_s), hi = std::max(from_s, to_s);
  std::vector<Vec3> slice{atS(lane, lo)};
  for (size_t i = 1; i + 1 < lane.s.size(); ++i) {
    if (lane.s[i] > lo && lane.s[i] < hi) slice.push_back(lane.center[i]);
  }
  slice.push_back(atS(lane, hi));
  std::vector<autoware_planning_msgs::msg::PathPoint> points;
  const double speed = params_.use_lane_speed_limits && lane.speed_limit > 0.1 ? lane.speed_limit : velocity;
  resampleAndOrient(slice, params_.path_interval, forward, reverse, speed, points);
  for (const auto & p : points) {
    if (!result.points.empty() &&
        std::hypot(p.pose.position.x - result.points.back().pose.position.x,
                   p.pose.position.y - result.points.back().pose.position.y) < 0.05) continue;
    result.points.push_back(p);
  }
  // Bounds are kept in motion order and swapped when traveling against lane direction.
  const auto & left = forward ? lane.left : lane.right;
  const auto & right = forward ? lane.right : lane.left;
  auto append = [forward](const std::vector<Vec3> & src, std::vector<geometry_msgs::msg::Point> & dst) {
    auto push = [&dst](const Vec3 & p) { geometry_msgs::msg::Point q; q.x=p.x; q.y=p.y; q.z=p.z; dst.push_back(q); };
    if (forward) { for (const auto & p : src) push(p); }
    else { for (auto it = src.rbegin(); it != src.rend(); ++it) push(*it); }
  };
  append(left, result.left_bound);
  append(right, result.right_bound);
}

MapPathGenerator::SnapResult MapPathGenerator::generateMapSnappedPath(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & sparse,
  const std::vector<uint8_t> & lane_changes, bool reverse, PathGenerator * path_gen) const
{
  SnapResult result;
  if (sparse.size() < 2 || lanes_.empty()) return result;
  std::vector<Projection> projections;
  size_t snapped = 0;
  for (const auto & pt : sparse) {
    auto p = projectWaypoint(pt.pose.position.x, pt.pose.position.y, pt.pose.position.z);
    projections.push_back(p);
    InputProjection input;
    input.valid = p.valid;
    input.snap_dist = static_cast<float>(p.distance);
    input.x = p.valid ? p.point.x : pt.pose.position.x;
    input.y = p.valid ? p.point.y : pt.pose.position.y;
    input.z = p.valid ? p.point.z : pt.pose.position.z;
    input.lane_id = p.valid ? lanes_[p.lane_index].id : 0;
    result.input_projections.push_back(input);
    if (p.valid) ++snapped;
  }
  if (!snapped) return result;
  result.fully_snapped = snapped == sparse.size();
  for (size_t i = 0; i + 1 < sparse.size();) {
    const bool change = i < lane_changes.size() && lane_changes[i] != 0;
    size_t end = i + 1;
    if (change) {
      while (end + 1 < sparse.size() && end < lane_changes.size() && lane_changes[end] != 0) ++end;
    }
    const auto & a = projections[i];
    const auto & b = projections[end];
    const size_t first_output = result.points.size();
    if (change) {
      auto from = sparse[i], to = sparse[end];
      if (a.valid) { from.pose.position.x=a.point.x; from.pose.position.y=a.point.y; from.pose.position.z=a.point.z; }
      if (b.valid) { to.pose.position.x=b.point.x; to.pose.position.y=b.point.y; to.pose.position.z=b.point.z; }
      if (path_gen && a.valid && b.valid) {
        auto points = path_gen->generateObliqueLaneChangeSegment(from, to);
        for (const auto & pt : points) {
          if (!result.points.empty() && distance2d(
            {pt.pose.position.x, pt.pose.position.y, 0},
            {result.points.back().pose.position.x, result.points.back().pose.position.y, 0}) < 0.05) continue;
          result.points.push_back(pt);
        }
      } else interpolateSegment(from, to, reverse, result.points);
    } else if (a.valid && b.valid) {
      double velocity = std::abs(sparse[i].longitudinal_velocity_mps);
      if (velocity < 0.1) velocity = params_.default_velocity;
      if (a.lane_index == b.lane_index) {
        extractLane(a.lane_index, a.s, b.s, reverse, velocity, result);
      } else {
        bool forward = true;
        const auto sequence = findLaneSequence(a.lane_index, b.lane_index, &forward);
        if (sequence.empty()) interpolateSegment(sparse[i], sparse[end], reverse, result.points);
        else for (size_t k = 0; k < sequence.size(); ++k) {
          const double from_s = k == 0 ? a.s : (forward ? 0.0 : lanes_[sequence[k]].s.back());
          const double to_s = k + 1 == sequence.size() ? b.s :
            (forward ? lanes_[sequence[k]].s.back() : 0.0);
          extractLane(sequence[k], from_s, to_s, reverse, velocity, result);
        }
      }
    } else interpolateSegment(sparse[i], sparse[end], reverse, result.points);
    if (!has_elevation_ && !change) {
      const size_t count = result.points.size() - first_output;
      for (size_t j = 0; j < count; ++j) {
        const double t = count > 1 ? static_cast<double>(j) / (count - 1) : 0.0;
        result.points[first_output + j].pose.position.z =
          sparse[i].pose.position.z +
          t * (sparse[end].pose.position.z - sparse[i].pose.position.z);
      }
    }
    i = end;
  }
  if (!result.points.empty()) result.points.back().is_final = true;
  return result;
}

void MapPathGenerator::interpolateSegment(
  const autoware_planning_msgs::msg::PathPoint & from,
  const autoware_planning_msgs::msg::PathPoint & to, bool reverse,
  std::vector<autoware_planning_msgs::msg::PathPoint> & out) const
{
  const auto & a = from.pose.position;
  const auto & b = to.pose.position;
  const double dist = std::hypot(b.x - a.x, b.y - a.y);
  const size_t steps = std::max<size_t>(1, static_cast<size_t>(dist / params_.path_interval));
  const auto q = quaternionFromYaw(std::atan2(b.y - a.y, b.x - a.x) + (reverse ? M_PI : 0.0));
  for (size_t i = 0; i < steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    auto pt = from;
    pt.pose.position.x = a.x + t * (b.x - a.x);
    pt.pose.position.y = a.y + t * (b.y - a.y);
    pt.pose.position.z = a.z + t * (b.z - a.z);
    pt.pose.orientation = q;
    pt.longitudinal_velocity_mps = from.longitudinal_velocity_mps +
      static_cast<float>(t) * (to.longitudinal_velocity_mps - from.longitudinal_velocity_mps);
    pt.is_final = false;
    if (!out.empty() && std::hypot(pt.pose.position.x - out.back().pose.position.x,
                                  pt.pose.position.y - out.back().pose.position.y) < 0.05) continue;
    out.push_back(pt);
  }
}

geometry_msgs::msg::Quaternion MapPathGenerator::quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(yaw / 2.0);
  q.z = std::sin(yaw / 2.0);
  return q;
}
}  // namespace igv_path_planner
