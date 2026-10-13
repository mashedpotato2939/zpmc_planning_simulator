// Copyright 2024 ZPMC
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

// Oblique (crab-walking) lane-change generator. Implements
// PathGenerator::generateObliqueLaneChangeSegment plus the piecewise-parabolic
// lateral-profile solver ported from map_traj_server. The solver and its
// sampler are TU-private (anonymous namespace) so they no longer leak into
// path_generator.hpp.

#include "igv_path_planner/path_generator.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

namespace igv_path_planner
{
namespace
{

struct CrabResult
{
  int state = 1;
  double ddy = 0.0;
  double dx1 = 0.0;
  double dx2 = 0.0;
  double dx3 = 0.0;
};

// Solve for the lateral acceleration profile between (0, y0, dy0) and (dx, y1)
// subject to |dy/dx| <= dymax and |d^2y/dx^2| in [ddymin, ddymax].
// state in {0,2,3,4} is feasible; {1,5} is infeasible.
CrabResult crabwalkingTrajGenerate(
  double y0, double y1, double dy0, double dx,
  double dymax, double ddymax, double ddymin)
{
  const double delta_y = y1 - y0;
  double ddy = 0.0, dx1 = 0.0, dx2 = 0.0;
  double dx3 = std::numeric_limits<double>::quiet_NaN();
  double dyPeak = 0.0;

  if (delta_y == 0.0) {
    if (dy0 == 0.0) {
      return {0, 0.0, dx, std::numeric_limits<double>::quiet_NaN(), dx3};
    }
    // Symmetric accel+decel to cancel initial slope
    ddy = -(std::sqrt(2.0) + 1.0) * dy0 / dx;
    dx1 = std::sqrt(2.0) / 2.0 * dx;
    dx2 = dx - dx1;
    return {3, ddy, dx1, dx2, dx3};
  }

  if (delta_y > 0.0) {
    if (dy0 * dx > 2.0 * delta_y) {
      return {1, std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(), dx3};
    }
    // Case 1: const slope then decelerate
    if (dy0 * dx > delta_y && dy0 * dx <= 2.0 * delta_y) {
      ddy = dy0 * dy0 / (y0 - y1 + dy0 * dx) / 2.0;
      dx2 = 2.0 * (y0 - y1 + dy0 * dx) / dy0;
      dx1 = dx - dx2;
      if (ddy < ddymin) {
        return {1, std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(), dx3};
      }
      if (ddy > ddymin && ddy <= ddymax) {
        return {2, ddy, dx1, dx2, dx3};
      }
    }
    // Case 2: accel then decel (symmetric peak)
    const double ddy_t1 = -dx * dy0 + 2.0 * delta_y;
    const double ddy_t2 = std::sqrt(2.0 * (dx * dx * dy0 * dy0 - 2.0 * dx * dy0 * delta_y +
                                           2.0 * delta_y * delta_y));
    ddy = (ddy_t1 + ddy_t2) / dx / dx;
    dx1 = dx / 2.0 - dy0 / 2.0 / ddy;
    dx2 = dx / 2.0 + dy0 / 2.0 / ddy;
    if (ddy < ddymin) {
      return {1, std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(), dx3};
    } else if (ddy > ddymax) {
      // fall through to Case 3 (3-phase with ddy = ddymax)
    } else {
      dyPeak = dy0 + ddy * dx1;
      if (dyPeak <= dymax) {
        return {3, ddy, dx1, dx2, dx3};
      }
    }
    // Case 3: accel, cruise, decel with ddy = ddymax
    ddy = ddymax;
    const double t1 = ddy * dx - dy0;
    const double t2 = std::sqrt(ddy * ddy * dx * dx + 2.0 * ddy * dy0 * dx -
                                dy0 * dy0 - 4.0 * ddy * delta_y);
    dx1 = (t1 - t2) / 2.0 / ddy;
    dx2 = dx1 + dy0 / ddy;
    dx3 = dx - 2.0 * dx1 - dy0 / ddy;
    dyPeak = dy0 + ddy * dx1;
    if (dyPeak <= dymax) {
      return {4, ddy, dx1, dx2, dx3};
    }
    return {5, std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN()};
  }

  // delta_y < 0.0 — mirror of the positive branch
  if (dy0 * dx < 2.0 * delta_y) {
    return {1, std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(), dx3};
  }
  if (dy0 * dx < delta_y && dy0 * dx >= 2.0 * delta_y) {
    ddy = dy0 * dy0 / (y0 - y1 + dy0 * dx) / 2.0;
    dx2 = 2.0 * (y0 - y1 + dy0 * dx) / dy0;
    dx1 = dx - dx2;
    if (ddy > -ddymin) {
      return {1, std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(), dx3};
    }
    if (ddy <= -ddymin && ddy >= -ddymax) {
      return {2, ddy, dx1, dx2, dx3};
    }
  }
  const double ddy_t1n = -dx * dy0 + 2.0 * delta_y;
  const double ddy_t2n = std::sqrt(2.0 * (dx * dx * dy0 * dy0 - 2.0 * dx * dy0 * delta_y +
                                          2.0 * delta_y * delta_y));
  ddy = (ddy_t1n - ddy_t2n) / dx / dx;
  dx1 = dx / 2.0 - dy0 / 2.0 / ddy;
  dx2 = dx / 2.0 + dy0 / 2.0 / ddy;
  if (ddy > -ddymin) {
    return {1, std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(), dx3};
  } else if (ddy < -ddymax) {
    // fall through
  } else {
    dyPeak = dy0 + ddy * dx1;
    if (dyPeak >= -dymax) {
      return {3, ddy, dx1, dx2, dx3};
    }
  }
  ddy = -ddymax;
  const double t1n = ddy * dx - dy0;
  const double t2n = std::sqrt(ddy * ddy * dx * dx + 2.0 * ddy * dy0 * dx -
                               dy0 * dy0 - 4.0 * ddy * delta_y);
  dx1 = (t1n + t2n) / 2.0 / ddy;
  dx2 = dx1 + dy0 / ddy;
  dx3 = dx - 2.0 * dx1 - dy0 / ddy;
  dyPeak = dy0 + ddy * dx1;
  if (dyPeak >= -dymax) {
    return {4, ddy, dx1, dx2, dx3};
  }
  return {5, std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN()};
}

// Evaluate y(x) and dy/dx at x given the packed trajectory vector
// (format: [state, x0_offset, y0, dy0, ddy, dx1, dx2, dx3, dx4, dxsum, y1]).
void calcPosyVy(
  const std::vector<double> & traj, double x, double & y, double & dy)
{
  const int state = static_cast<int>(traj[0]);
  const double y0 = traj[2];
  const double y1 = traj[10];
  const double dy0 = traj[3];
  const double ddy = traj[4];
  const double dx1 = traj[5];
  const double dx3 = traj[7];
  const double dxsum = traj[9];

  y = 0.0;
  dy = 0.0;
  if (state == 0) {
    y = y0;
    dy = dy0;
  } else if (state == 2) {
    if (x >= 0.0 && x < dx1) {
      y = y0 + dy0 * x;
      dy = dy0;
    } else if (x >= dx1 && x <= dxsum) {
      y = y0 + dy0 * x - 0.5 * ddy * (x - dx1) * (x - dx1);
      dy = dy0 - ddy * (x - dx1);
    }
  } else if (state == 3) {
    if (x >= 0.0 && x < dx1) {
      y = y0 + dy0 * x + 0.5 * ddy * x * x;
      dy = dy0 + ddy * x;
    } else if (x >= dx1 && x <= dxsum) {
      const double y_t1 = y0 + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
      const double v_t1 = dy0 + ddy * dx1;
      y = y_t1 + v_t1 * (x - dx1) - 0.5 * ddy * (x - dx1) * (x - dx1);
      dy = v_t1 - ddy * (x - dx1);
    }
  } else if (state == 4) {
    if (x >= 0.0 && x < dx1) {
      y = y0 + dy0 * x + 0.5 * ddy * x * x;
      dy = dy0 + ddy * x;
    } else if (x >= dx1 && x < (dx1 + dx3)) {
      const double y_t1 = y0 + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
      const double v_t1 = dy0 + ddy * dx1;
      y = y_t1 + v_t1 * (x - dx1);
      dy = v_t1;
    } else if (x >= (dx1 + dx3) && x <= dxsum) {
      const double y_t1 = y0 + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
      const double v_t1 = dy0 + ddy * dx1;
      const double y_t2 = y_t1 + v_t1 * dx3;
      const double v_t2 = v_t1;
      y = y_t2 + v_t2 * (x - (dx1 + dx3)) -
          0.5 * ddy * (x - (dx1 + dx3)) * (x - (dx1 + dx3));
      dy = v_t2 - ddy * (x - (dx1 + dx3));
    }
  }

  if (x < 0.0) {
    y = y0;
  } else if (x > dxsum) {
    y = y1;
  }
}

}  // namespace

std::vector<autoware_planning_msgs::msg::PathPoint> PathGenerator::generateObliqueLaneChangeSegment(
  const autoware_planning_msgs::msg::PathPoint & start,
  const autoware_planning_msgs::msg::PathPoint & end)
{
  std::vector<autoware_planning_msgs::msg::PathPoint> points;

  const double start_yaw = getYaw(start.pose.orientation);
  // For reverse driving, vehicle facing direction (start_yaw) is opposite to
  // motion. The crab profile must be solved against motion direction so dx is
  // positive; output orientations stay at start_yaw (constant facing).
  const double motion_yaw = is_reverse_ ? (start_yaw + M_PI) : start_yaw;
  const double cos_m = std::cos(motion_yaw);
  const double sin_m = std::sin(motion_yaw);

  // Project end into motion-aligned local frame: x along motion, y perpendicular.
  const double ex = end.pose.position.x - start.pose.position.x;
  const double ey = end.pose.position.y - start.pose.position.y;
  const double dx_local = ex * cos_m + ey * sin_m;
  const double y1_local = -ex * sin_m + ey * cos_m;

  if (dx_local <= params_.path_interval) {
    // Degenerate: no forward distance — fall back to lane-change quintic.
    return generateLaneChangeSegment(start, end);
  }

  const auto crab = crabwalkingTrajGenerate(
    0.0, y1_local, 0.0, dx_local,
    params_.crab_dymax, params_.crab_ddymax, params_.crab_ddymin);

  if (crab.state == 1 || crab.state == 5) {
    if (params_.debug) {
      std::cerr << "[PathGen] Oblique infeasible (state=" << crab.state
                << ", dx=" << dx_local << ", y1=" << y1_local
                << "), falling back to LANE_CHANGE" << std::endl;
    }
    return generateLaneChangeSegment(start, end);
  }

  // Pack trajectory vector for calcPosyVy (format matches map_traj_server).
  std::vector<double> traj(11);
  traj[0] = static_cast<double>(crab.state);
  traj[1] = 0.0;            // x offset (unused here, we sample local x directly)
  traj[2] = 0.0;            // y0
  traj[3] = 0.0;            // dy0
  traj[4] = crab.ddy;
  traj[5] = crab.dx1;
  traj[6] = crab.dx2;
  traj[7] = crab.dx3;
  traj[8] = 0.0;            // dx4 padding
  traj[9] = dx_local;       // dxsum
  traj[10] = y1_local;

  const bool end_is_goal = (std::abs(end.longitudinal_velocity_mps) < 0.1);
  const int n_samples = std::max(2, static_cast<int>(dx_local / params_.path_interval));

  for (int i = 1; i <= n_samples; ++i) {
    const double t = static_cast<double>(i) / n_samples;
    const double x_local = t * dx_local;
    double y_local = 0.0, dy_local = 0.0;
    calcPosyVy(traj, x_local, y_local, dy_local);

    autoware_planning_msgs::msg::PathPoint pt;
    pt.pose.position.x = start.pose.position.x + x_local * cos_m - y_local * sin_m;
    pt.pose.position.y = start.pose.position.y + x_local * sin_m + y_local * cos_m;
    pt.pose.position.z = start.pose.position.z +
                         t * (end.pose.position.z - start.pose.position.z);
    pt.pose.orientation = createQuaternionFromYaw(start_yaw);  // constant heading

    const double vx_raw = (1.0 - t) * start.longitudinal_velocity_mps +
                          t * end.longitudinal_velocity_mps;
    const double vx = (vx_raw >= 0.0)
      ? std::min(vx_raw, params_.oblique_speed_limit)
      : std::max(vx_raw, -params_.oblique_speed_limit);
    const bool is_last = (i == n_samples);
    pt.longitudinal_velocity_mps = (is_last && end_is_goal) ? 0.0f : static_cast<float>(vx);
    // Body-frame lateral velocity: dy_local is the slope in the motion frame
    // and vx carries its sign (negative for reverse). The product yields the
    // correct sign in the vehicle-facing body frame for both directions.
    pt.lateral_velocity_mps = static_cast<float>(dy_local * vx);
    pt.heading_rate_rps = 0.0f;
    pt.is_final = is_last;
    points.push_back(pt);
  }

  if (params_.debug) {
    std::cerr << "[PathGen] Oblique: state=" << crab.state
              << ", dx=" << dx_local << "m, y1=" << y1_local << "m"
              << ", ddy=" << crab.ddy << ", dx1=" << crab.dx1
              << ", dx3=" << crab.dx3 << ", points=" << points.size()
              << ", reverse=" << (is_reverse_ ? 1 : 0) << std::endl;
  }

  return points;
}

}  // namespace igv_path_planner
