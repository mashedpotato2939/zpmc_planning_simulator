// Truth-table coverage for MapPathGenerator::resampleAndOrient — the function that
// builds motion-order, vehicle-facing-yaw path points from a centerline slice.
//
// Bug context: prior versions of extractCenterlineSegment used lane.heading()[k]
// directly and applied an unconditional +π flip for reverse driving, which gives
// wrong yaw when FMS travels against the lane's natural direction. The new helper
// derives yaw from consecutive output positions (motion direction) and flips only
// for reverse. These tests pin all four combinations of (forward/reverse) ×
// (matches/opposite).

#include <cmath>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include <autoware_planning_msgs/msg/path_point.hpp>

#include "igv_path_planner/map_path_generator.hpp"

using igv_path_planner::MapPathGenerator;

namespace igv_path_planner
{

class MapPathGeneratorTestAccess
{
public:
  static std::vector<uint64_t> findLaneSequenceByTopology(
    uint64_t from_lane, uint64_t to_lane,
    const std::unordered_map<uint64_t, std::vector<uint64_t>> & successors,
    const std::unordered_map<uint64_t, std::vector<uint64_t>> & predecessors,
    bool * follows_successors)
  {
    const auto lookup =
      [](const auto & adjacency, const uint64_t id) -> std::vector<uint64_t> {
        const auto it = adjacency.find(id);
        return it == adjacency.end() ? std::vector<uint64_t>{} : it->second;
      };

    return MapPathGenerator::findLaneSequenceByTopology(
      from_lane, to_lane,
      [&successors, &lookup](const uint64_t id) { return lookup(successors, id); },
      [&predecessors, &lookup](const uint64_t id) { return lookup(predecessors, id); },
      follows_successors);
  }

  static std::vector<uint64_t> findLaneSequenceByEndpointContinuity(
    uint64_t from_lane,
    const MapPathGenerator::Vec3 & from_first,
    const MapPathGenerator::Vec3 & from_last,
    uint64_t to_lane,
    const MapPathGenerator::Vec3 & to_first,
    const MapPathGenerator::Vec3 & to_last,
    bool * follows_successors)
  {
    constexpr float kThresholdM = 1.0f;
    return MapPathGenerator::findLaneSequenceByEndpointContinuity(
      from_lane, from_first, from_last, to_lane, to_first, to_last, kThresholdM,
      follows_successors);
  }
};

}  // namespace igv_path_planner

namespace
{

constexpr double kYawTol = 1e-3;  // ~0.057°
constexpr double kPosTol = 1e-3;

double quatToYaw(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double wrapAngle(double a)
{
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// Build a straight 10-point centerline of total length 9.0 m in direction
// `lane_dir_rad`, starting at the origin.
std::vector<MapPathGenerator::Vec3> makeStraightSlice(double lane_dir_rad)
{
  std::vector<MapPathGenerator::Vec3> cl;
  for (int i = 0; i < 10; ++i) {
    MapPathGenerator::Vec3 v;
    v.x = static_cast<float>(i * std::cos(lane_dir_rad));
    v.y = static_cast<float>(i * std::sin(lane_dir_rad));
    v.z = 0.0f;
    cl.push_back(v);
  }
  return cl;
}

// Assert every interior point's yaw equals `expected_yaw_rad` (wrapped).
void expectAllYawsEqual(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & pts,
  double expected_yaw_rad,
  const char * label)
{
  ASSERT_GE(pts.size(), 2u) << label;
  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    const double y = quatToYaw(pts[i].pose.orientation);
    const double diff = wrapAngle(y - expected_yaw_rad);
    EXPECT_NEAR(diff, 0.0, kYawTol)
      << label << ": pt[" << i << "] yaw=" << y
      << " expected=" << expected_yaw_rad;
  }
  // Last point inherits previous orientation
  EXPECT_NEAR(
    wrapAngle(quatToYaw(pts.back().pose.orientation) - expected_yaw_rad),
    0.0, kYawTol) << label << ": last-point yaw mismatch";
}

}  // namespace

using igv_path_planner::MapPathGenerator;

// Case 1: forward + matches.
// Lane direction = motion direction = vehicle-facing. yaw == lane direction.
// Order: s-ascending. Velocity: positive.
TEST(MapPathGeneratorReverse, ForwardMatches)
{
  const double lane_dir = 0.5;  // ~28.6°
  auto slice = makeStraightSlice(lane_dir);

  std::vector<autoware_planning_msgs::msg::PathPoint> out;
  MapPathGenerator::resampleAndOrient(
    slice, /*path_interval=*/1.0,
    /*fms_matches_lane=*/true, /*is_reverse=*/false,
    /*seg_velocity=*/5.0, out);

  ASSERT_FALSE(out.empty());
  // s-ascending: first output point is the slice start.
  EXPECT_NEAR(out.front().pose.position.x, slice.front().x, kPosTol);
  EXPECT_NEAR(out.front().pose.position.y, slice.front().y, kPosTol);
  EXPECT_NEAR(out.back().pose.position.x,  slice.back().x,  kPosTol);
  EXPECT_NEAR(out.back().pose.position.y,  slice.back().y,  kPosTol);
  // yaw == lane direction
  expectAllYawsEqual(out, lane_dir, "forward+matches");
  // velocity positive
  EXPECT_GT(out.front().longitudinal_velocity_mps, 0.0f);
}

// Case 2: forward + opposite (FMS travels against the lane direction).
// Motion direction = -lane_dir (lane_dir + π). Vehicle-facing = motion.
// Order: s-descending. Velocity: positive.
TEST(MapPathGeneratorReverse, ForwardOpposite)
{
  const double lane_dir = 0.5;
  auto slice = makeStraightSlice(lane_dir);

  std::vector<autoware_planning_msgs::msg::PathPoint> out;
  MapPathGenerator::resampleAndOrient(
    slice, /*path_interval=*/1.0,
    /*fms_matches_lane=*/false, /*is_reverse=*/false,
    /*seg_velocity=*/5.0, out);

  ASSERT_FALSE(out.empty());
  // s-descending: first output point is the slice END.
  EXPECT_NEAR(out.front().pose.position.x, slice.back().x,  kPosTol);
  EXPECT_NEAR(out.front().pose.position.y, slice.back().y,  kPosTol);
  EXPECT_NEAR(out.back().pose.position.x,  slice.front().x, kPosTol);
  EXPECT_NEAR(out.back().pose.position.y,  slice.front().y, kPosTol);
  // yaw == motion direction == lane_dir + π
  expectAllYawsEqual(out, wrapAngle(lane_dir + M_PI), "forward+opposite");
  EXPECT_GT(out.front().longitudinal_velocity_mps, 0.0f);
}

// Case 3: reverse + matches.
// Lane direction = motion direction. Vehicle-facing = motion + π = lane_dir + π.
// Order: s-ascending (motion = lane direction). Velocity: negative.
TEST(MapPathGeneratorReverse, ReverseMatches)
{
  const double lane_dir = 0.5;
  auto slice = makeStraightSlice(lane_dir);

  std::vector<autoware_planning_msgs::msg::PathPoint> out;
  MapPathGenerator::resampleAndOrient(
    slice, /*path_interval=*/1.0,
    /*fms_matches_lane=*/true, /*is_reverse=*/true,
    /*seg_velocity=*/5.0, out);

  ASSERT_FALSE(out.empty());
  EXPECT_NEAR(out.front().pose.position.x, slice.front().x, kPosTol);
  EXPECT_NEAR(out.back().pose.position.x,  slice.back().x,  kPosTol);
  expectAllYawsEqual(out, wrapAngle(lane_dir + M_PI), "reverse+matches");
  EXPECT_LT(out.front().longitudinal_velocity_mps, 0.0f);
}

// Case 4: reverse + opposite — the bag's case.
// Motion direction = -lane_dir. Vehicle-facing = motion + π = lane_dir.
// Order: s-descending. Velocity: negative.
// Concrete numbers matching the analyzed bag: lane oriented SE (-42° ≈ -0.733 rad),
// FMS reverses NW. Expected vehicle-facing yaw == lane direction (-0.733).
TEST(MapPathGeneratorReverse, ReverseOppositeBagCase)
{
  const double lane_dir = -0.733;  // ~ -42° (matches the bag we analyzed)
  auto slice = makeStraightSlice(lane_dir);

  std::vector<autoware_planning_msgs::msg::PathPoint> out;
  MapPathGenerator::resampleAndOrient(
    slice, /*path_interval=*/1.0,
    /*fms_matches_lane=*/false, /*is_reverse=*/true,
    /*seg_velocity=*/5.0, out);

  ASSERT_FALSE(out.empty());
  // Output starts at slice.back() (s-descending iteration)
  EXPECT_NEAR(out.front().pose.position.x, slice.back().x,  kPosTol);
  EXPECT_NEAR(out.front().pose.position.y, slice.back().y,  kPosTol);
  EXPECT_NEAR(out.back().pose.position.x,  slice.front().x, kPosTol);
  EXPECT_NEAR(out.back().pose.position.y,  slice.front().y, kPosTol);
  // Vehicle-facing yaw == lane direction (NO flip, despite is_reverse)
  expectAllYawsEqual(out, lane_dir, "reverse+opposite (bag case)");
  // Body-frame velocity is negative
  EXPECT_LT(out.front().longitudinal_velocity_mps, 0.0f);
  // Magnitude matches input seg_velocity
  EXPECT_NEAR(std::abs(out.front().longitudinal_velocity_mps), 5.0, 1e-4);
}

// Sanity: a too-short slice (< 2 points) yields no output rather than UB.
TEST(MapPathGeneratorReverse, DegenerateSliceProducesEmptyOutput)
{
  std::vector<MapPathGenerator::Vec3> slice;
  slice.push_back({0.0f, 0.0f, 0.0f});

  std::vector<autoware_planning_msgs::msg::PathPoint> out;
  MapPathGenerator::resampleAndOrient(
    slice, 1.0, true, false, 5.0, out);
  EXPECT_TRUE(out.empty());
}

TEST(MapPathGeneratorReverse, SuccessorTopologyKeepsTravelOrder)
{
  const std::unordered_map<uint64_t, std::vector<uint64_t>> successors = {
    {10, {20}},
    {20, {30}},
  };
  const std::unordered_map<uint64_t, std::vector<uint64_t>> predecessors;

  bool follows_successors = false;
  const auto sequence = igv_path_planner::MapPathGeneratorTestAccess::findLaneSequenceByTopology(
    10, 30, successors, predecessors, &follows_successors);

  EXPECT_TRUE(follows_successors);
  const std::vector<uint64_t> expected = {10, 20, 30};
  EXPECT_EQ(sequence, expected);
}

TEST(MapPathGeneratorReverse, PredecessorTopologyKeepsTravelOrder)
{
  const std::unordered_map<uint64_t, std::vector<uint64_t>> successors;
  const std::unordered_map<uint64_t, std::vector<uint64_t>> predecessors = {
    {30, {20}},
    {20, {10}},
  };

  bool follows_successors = true;
  const auto sequence = igv_path_planner::MapPathGeneratorTestAccess::findLaneSequenceByTopology(
    30, 10, successors, predecessors, &follows_successors);

  EXPECT_FALSE(follows_successors);
  const std::vector<uint64_t> expected = {30, 20, 10};
  EXPECT_EQ(sequence, expected);
}

TEST(MapPathGeneratorReverse, EndpointContinuityUsesSuccessorDirection)
{
  bool follows_successors = false;
  const auto sequence =
    igv_path_planner::MapPathGeneratorTestAccess::findLaneSequenceByEndpointContinuity(
      10, {0.0f, 0.0f, 0.0f}, {5.0f, 0.0f, 0.0f},
      20, {5.5f, 0.0f, 0.0f}, {10.0f, 0.0f, 0.0f},
      &follows_successors);

  EXPECT_TRUE(follows_successors);
  const std::vector<uint64_t> expected = {10, 20};
  EXPECT_EQ(sequence, expected);
}

TEST(MapPathGeneratorReverse, EndpointContinuityUsesPredecessorDirection)
{
  bool follows_successors = true;
  const auto sequence =
    igv_path_planner::MapPathGeneratorTestAccess::findLaneSequenceByEndpointContinuity(
      34359738495u, {41.5988f, -77.0379f, 0.0f}, {58.9723f, -57.3553f, 0.0f},
      30064771199u, {22.9615f, -78.1484f, 0.0f}, {41.5988f, -77.0379f, 0.0f},
      &follows_successors);

  EXPECT_FALSE(follows_successors);
  const std::vector<uint64_t> expected = {34359738495u, 30064771199u};
  EXPECT_EQ(sequence, expected);
}

TEST(MapPathGeneratorReverse, EndpointContinuityRejectsDistantEndpoints)
{
  bool follows_successors = true;
  const auto sequence =
    igv_path_planner::MapPathGeneratorTestAccess::findLaneSequenceByEndpointContinuity(
      10, {0.0f, 0.0f, 0.0f}, {5.0f, 0.0f, 0.0f},
      20, {7.0f, 0.0f, 0.0f}, {12.0f, 0.0f, 0.0f},
      &follows_successors);

  EXPECT_TRUE(sequence.empty());
}

// Exercises the Lanelet2-to-planner path with a flattened map and elevated FMS
// waypoints, as in the Nanhui OSM/PCD pair.
TEST(MapPathGeneratorReverse, SnapsAcrossLaneletsAndPreservesInputElevation)
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  const auto add_lane = [&map](double x0, double x1) {
    lanelet::LineString3d left(
      lanelet::utils::getId(), {lanelet::Point3d(lanelet::utils::getId(), x0, 2.0, 0.0),
                         lanelet::Point3d(lanelet::utils::getId(), x1, 2.0, 0.0)});
    lanelet::LineString3d right(
      lanelet::utils::getId(), {lanelet::Point3d(lanelet::utils::getId(), x0, -2.0, 0.0),
                         lanelet::Point3d(lanelet::utils::getId(), x1, -2.0, 0.0)});
    map->add(lanelet::Lanelet(lanelet::utils::getId(), left, right));
  };
  add_lane(0.0, 10.0);
  add_lane(10.0, 20.0);
  MapPathGenerator generator(MapPathGenerator::Parameters{});
  generator.setMap(map);
  ASSERT_TRUE(generator.hasMap());

  std::vector<autoware_planning_msgs::msg::PathPoint> sparse(2);
  sparse[0].pose.position.x = 2.0;
  sparse[0].pose.position.y = 1.0;
  sparse[0].pose.position.z = 20.0;
  sparse[0].longitudinal_velocity_mps = 5.0f;
  sparse[1].pose.position.x = 18.0;
  sparse[1].pose.position.y = 1.0;
  sparse[1].pose.position.z = 22.0;
  sparse[1].longitudinal_velocity_mps = 5.0f;

  const auto result = generator.generateMapSnappedPath(sparse, {0, 0}, false);
  ASSERT_TRUE(result.fully_snapped);
  ASSERT_GT(result.points.size(), 2u);
  EXPECT_NEAR(result.points.front().pose.position.x, 2.0, 1e-3);
  EXPECT_NEAR(result.points.back().pose.position.x, 18.0, 1e-3);
  EXPECT_NEAR(result.points.front().pose.position.y, 0.0, 1e-3);
  EXPECT_NEAR(result.points.back().pose.position.y, 0.0, 1e-3);
  EXPECT_NEAR(result.points.front().pose.position.z, 20.0, 1e-3);
  EXPECT_NEAR(result.points.back().pose.position.z, 22.0, 1e-3);
}
