// Copyright 2024 ZPMC
// Licensed under the Apache License, Version 2.0
#ifndef IGV_PATH_PLANNER__MAP_PATH_GENERATOR_HPP_
#define IGV_PATH_PLANNER__MAP_PATH_GENERATOR_HPP_

#include <autoware_planning_msgs/msg/path_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <lanelet2_core/LaneletMap.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace igv_path_planner
{
class PathGenerator;

class MapPathGenerator
{
public:
  struct Parameters {
    double snap_distance_threshold = 5.0;
    double path_interval = 1.0;
    bool use_lane_speed_limits = false;
    double default_velocity = 5.0;
  };
  struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };
  struct InputProjection {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    uint64_t lane_id = 0;
    float snap_dist = 0.0f;
    bool valid = false;
  };
  struct SnapResult {
    std::vector<autoware_planning_msgs::msg::PathPoint> points;
    std::vector<geometry_msgs::msg::Point> left_bound;
    std::vector<geometry_msgs::msg::Point> right_bound;
    std::vector<InputProjection> input_projections;
    bool fully_snapped = false;
  };

  explicit MapPathGenerator(const Parameters & params);
  void setMap(const lanelet::LaneletMapPtr & map);
  bool hasMap() const { return !lanes_.empty(); }
  SnapResult generateMapSnappedPath(
    const std::vector<autoware_planning_msgs::msg::PathPoint> & sparse_points,
    const std::vector<uint8_t> & lane_change_types, bool is_reverse,
    PathGenerator * path_gen = nullptr) const;

  static void resampleAndOrient(
    const std::vector<Vec3> & centerline_slice, double path_interval,
    bool fms_matches_lane, bool is_reverse, double seg_velocity,
    std::vector<autoware_planning_msgs::msg::PathPoint> & out);

private:
  using LaneId = uint64_t;
  using LaneNeighborProvider = std::function<std::vector<LaneId>(LaneId)>;
  friend class MapPathGeneratorTestAccess;
  struct Lane {
    LaneId id = 0;
    std::vector<Vec3> center, left, right;
    std::vector<double> s;
    std::vector<LaneId> successors, predecessors;
    double speed_limit = 0.0;
  };
  struct Projection {
    size_t lane_index = 0;
    double s = 0.0;
    Vec3 point;
    double distance = 0.0;
    bool valid = false;
  };
  Projection projectWaypoint(double x, double y, double z) const;
  std::vector<size_t> findLaneSequence(
    size_t from, size_t to, bool * follows_successors) const;
  static std::vector<LaneId> findLaneSequenceByTopology(
    LaneId from, LaneId to, const LaneNeighborProvider & successors,
    const LaneNeighborProvider & predecessors, bool * follows_successors);
  static std::vector<LaneId> findLaneSequenceByEndpointContinuity(
    LaneId from, const Vec3 & from_first, const Vec3 & from_last,
    LaneId to, const Vec3 & to_first, const Vec3 & to_last,
    double max_distance, bool * follows_successors);
  void extractLane(
    size_t index, double from_s, double to_s, bool reverse, double velocity,
    SnapResult & result) const;
  void interpolateSegment(
    const autoware_planning_msgs::msg::PathPoint & from,
    const autoware_planning_msgs::msg::PathPoint & to, bool reverse,
    std::vector<autoware_planning_msgs::msg::PathPoint> & out) const;
  static geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw);
  static Vec3 atS(const Lane & lane, double s);
  Parameters params_;
  bool has_elevation_ = false;
  std::vector<Lane> lanes_;
  std::unordered_map<LaneId, size_t> lane_index_;
};
}  // namespace igv_path_planner
#endif
