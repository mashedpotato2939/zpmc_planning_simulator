// Compare Q_TURN trajectories from two implementations:
//   - Reference: ported map_traj_server::traj_creator_turn_optimize math
//   - SUT:       PathGenerator::generateQTurnSegment in igv_path_planner
// Both are driven by the same (start, end, R, R_yaw, vel_max); compared after
// arc-length resampling so differences in sample count / placement wash out.

#include <cmath>
#include <cstdio>
#include <vector>

#include <gtest/gtest.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <autoware_planning_msgs/msg/path_point.hpp>

#include "igv_path_planner/path_generator.hpp"
#include "reference_map_traj_qturn.hpp"

namespace
{

geometry_msgs::msg::Quaternion yawToQuat(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

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

struct XYYaw
{
  double x;
  double y;
  double yaw;
};

std::vector<XYYaw> toXYYaw(const std::vector<autoware_planning_msgs::msg::PathPoint> & pts)
{
  std::vector<XYYaw> out;
  out.reserve(pts.size());
  for (const auto & p : pts) {
    out.push_back({p.pose.position.x, p.pose.position.y, quatToYaw(p.pose.orientation)});
  }
  return out;
}

std::vector<XYYaw> toXYYaw(const std::vector<reference_qturn::ReferencePoint> & pts)
{
  std::vector<XYYaw> out;
  out.reserve(pts.size());
  for (const auto & p : pts) out.push_back({p.x, p.y, p.yaw});
  return out;
}

// Linearly interpolate along cumulative arc length onto `N` uniform samples.
std::vector<XYYaw> arcLengthResample(const std::vector<XYYaw> & in, size_t N)
{
  std::vector<XYYaw> out;
  if (in.size() < 2 || N < 2) return out;

  std::vector<double> cum(in.size(), 0.0);
  for (size_t i = 1; i < in.size(); ++i) {
    const double dx = in[i].x - in[i - 1].x;
    const double dy = in[i].y - in[i - 1].y;
    cum[i] = cum[i - 1] + std::hypot(dx, dy);
  }
  const double total = cum.back();
  if (total <= 0.0) return out;

  out.reserve(N);
  size_t j = 1;
  for (size_t i = 0; i < N; ++i) {
    const double s = (static_cast<double>(i) / (N - 1)) * total;
    while (j < cum.size() - 1 && cum[j] < s) ++j;
    const double s0 = cum[j - 1];
    const double s1 = cum[j];
    const double t  = (s1 > s0) ? (s - s0) / (s1 - s0) : 0.0;
    XYYaw p;
    p.x   = in[j - 1].x + t * (in[j].x - in[j - 1].x);
    p.y   = in[j - 1].y + t * (in[j].y - in[j - 1].y);
    p.yaw = in[j - 1].yaw + t * wrapAngle(in[j].yaw - in[j - 1].yaw);
    out.push_back(p);
  }
  return out;
}

}  // namespace

TEST(QTurnCompare, PortedClothoidMatchesReference)
{
  geometry_msgs::msg::Point start;  start.x = 0.0;  start.y = 0.0;  start.z = 0.0;
  geometry_msgs::msg::Point end;    end.x   = 16.0; end.y   = 16.0; end.z   = 0.0;
  constexpr double R       = 15.5;
  constexpr double R_yaw   = 8.1;
  constexpr double vel_max = 3.0;
  constexpr int    pts_num = 100;

  // Reference: direct port of map_traj_server math. The reference samples the
  // curve at s = ds*(m+1) for m in [0, pts_num-1], so its first point lies one
  // step into the curve (not at `start`). The SUT emits `start` as an explicit
  // seed before sampling the curve. Prepend `start` to the reference so both
  // polylines begin at (start, yaw=0) and arc-length resampling aligns.
  auto ref = reference_qturn::generate_reference_qturn(
    start, end, vel_max, R, R_yaw, pts_num);
  ASSERT_EQ(ref.size(), static_cast<size_t>(pts_num));
  ref.insert(ref.begin(), reference_qturn::ReferencePoint{start.x, start.y, 0.0});

  // SUT: igv_path_planner PathGenerator.
  igv_path_planner::PathGenerator::Parameters p;
  p.qturn_min_radius     = R;
  p.qturn_yaw_min_radius = R_yaw;
  p.qturn_speed_limit    = vel_max;
  p.path_interval        = 0.5;
  // Ensure the 16x16 endpoint yaw of π/2 stays inside the gate window.
  p.qturn_yaw_min_threshold = 1.3;
  p.qturn_yaw_max_threshold = 1.9;
  igv_path_planner::PathGenerator gen(p);

  autoware_planning_msgs::msg::PathPoint s_pt{}, e_pt{};
  s_pt.pose.position = start;
  e_pt.pose.position = end;
  s_pt.pose.orientation = yawToQuat(0.0);
  e_pt.pose.orientation = yawToQuat(M_PI / 2.0);
  s_pt.longitudinal_velocity_mps = static_cast<float>(vel_max);
  e_pt.longitudinal_velocity_mps = static_cast<float>(vel_max);  // not a goal → no decel

  const auto sut = gen.generateQTurnSegment(s_pt, e_pt);
  ASSERT_FALSE(gen.lastQTurnFellBack())
    << "Q_TURN fell back for (0,0)→(16,16) R=" << R;
  ASSERT_GE(sut.size(), 2u);

  const auto ref_xy = toXYYaw(ref);
  const auto sut_xy = toXYYaw(sut);

  // Dump both trajectories to /tmp for external plotting.
  if (auto * f = std::fopen("/tmp/qturn_ref.csv", "w")) {
    std::fprintf(f, "x,y,yaw\n");
    for (const auto & p : ref_xy) std::fprintf(f, "%.6f,%.6f,%.6f\n", p.x, p.y, p.yaw);
    std::fclose(f);
  }
  if (auto * f = std::fopen("/tmp/qturn_sut.csv", "w")) {
    std::fprintf(f, "x,y,yaw\n");
    for (const auto & p : sut_xy) std::fprintf(f, "%.6f,%.6f,%.6f\n", p.x, p.y, p.yaw);
    std::fclose(f);
  }

  constexpr size_t N = 50;
  const auto ref_rs = arcLengthResample(ref_xy, N);
  const auto sut_rs = arcLengthResample(sut_xy, N);
  ASSERT_EQ(ref_rs.size(), N);
  ASSERT_EQ(sut_rs.size(), N);

  // Endpoint-of-input sanity first — easier to diagnose than a per-point loop.
  EXPECT_NEAR(sut.back().pose.position.x, 16.0, 0.05);
  EXPECT_NEAR(sut.back().pose.position.y, 16.0, 0.05);
  EXPECT_NEAR(quatToYaw(sut.back().pose.orientation), M_PI / 2.0, 1e-2);

  // Per-sample parity along the shared arc-length grid.
  for (size_t i = 0; i < N; ++i) {
    EXPECT_NEAR(sut_rs[i].x, ref_rs[i].x, 0.05) << "xy parity i=" << i;
    EXPECT_NEAR(sut_rs[i].y, ref_rs[i].y, 0.05) << "xy parity i=" << i;
    EXPECT_NEAR(wrapAngle(sut_rs[i].yaw - ref_rs[i].yaw), 0.0, 1e-2)
      << "yaw parity i=" << i;
  }
}
