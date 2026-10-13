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

// Q-turn (90° Fresnel-clothoid elbow) generator. Implements
// PathGenerator::generateQTurnSegment plus the Fresnel/clothoid numerics
// ported from map_traj_server::traj_creator_turn_optimize. The numerics are
// TU-private (anonymous namespace) so they no longer leak into
// path_generator.hpp.

#include "igv_path_planner/path_generator.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <vector>

namespace igv_path_planner
{
namespace
{

constexpr double kYawClothoidDsMargin = 0.01;  // m

double adaptiveSimpson(
  const std::function<double(double)> & f, double a, double b, double eps, int max_depth)
{
  auto simpson = [&](double aa, double bb) {
    return (bb - aa) / 6.0 * (f(aa) + 4.0 * f((aa + bb) / 2.0) + f(bb));
  };
  const double m = (a + b) / 2.0;
  const double whole = simpson(a, b);
  const double left = simpson(a, m);
  const double right = simpson(m, b);
  if (std::abs(left + right - whole) < 15.0 * eps || max_depth <= 0) {
    return left + right + (left + right - whole) / 15.0;
  }
  return adaptiveSimpson(f, a, m, eps / 2.0, max_depth - 1) +
         adaptiveSimpson(f, m, b, eps / 2.0, max_depth - 1);
}

double fresnelC(double x)
{
  if (x == 0.0) return 0.0;
  auto f = [](double t) { return std::cos(M_PI / 2.0 * t * t); };
  return adaptiveSimpson(f, 0.0, x, 1e-9, 20);
}

double fresnelS(double x)
{
  if (x == 0.0) return 0.0;
  auto f = [](double t) { return std::sin(M_PI / 2.0 * t * t); };
  return adaptiveSimpson(f, 0.0, x, 1e-9, 20);
}

void clothoidPoint(
  double km, double L1, double s, double & x, double & y, double & theta)
{
  const double A = std::sqrt(L1 / km);
  const double u = s / (A * std::sqrt(M_PI));
  x = A * std::sqrt(M_PI) * fresnelC(u);
  y = A * std::sqrt(M_PI) * fresnelS(u);
  theta = (km / (2.0 * L1)) * s * s;
}

void clothoidLocalOffset(
  double km, double L1, double s, double & dx, double & dy)
{
  if (s == 0.0) { dx = 0.0; dy = 0.0; return; }
  auto cos_theta = [km, L1](double u) {
    const double theta_off = km * u - (km / (2.0 * L1)) * u * u;
    return std::cos(theta_off);
  };
  auto sin_theta = [km, L1](double u) {
    const double theta_off = km * u - (km / (2.0 * L1)) * u * u;
    return std::sin(theta_off);
  };
  dx = adaptiveSimpson(cos_theta, 0.0, s, 1e-9, 20);
  dy = adaptiveSimpson(sin_theta, 0.0, s, 1e-9, 20);
}

double computeEndpointX(double km, double L1)
{
  const double R = 1.0 / km;
  const double L2 = M_PI / (2.0 * km) - L1;
  if (L2 <= 0.0) return std::nan("");
  double x1, y1, theta1;
  clothoidPoint(km, L1, L1, x1, y1, theta1);
  const double theta_arc = km * L2;
  const double Ox = x1 - R * std::sin(theta1);
  const double Oy = y1 + R * std::cos(theta1);
  const double x2 = Ox + R * std::sin(theta1 + theta_arc);
  const double y2 = Oy - R * std::cos(theta1 + theta_arc);
  const double theta2 = theta1 + theta_arc;
  double dx_local, dy_local;
  clothoidLocalOffset(km, L1, L1, dx_local, dy_local);
  (void)y2;
  return x2 + dx_local * std::cos(theta2) - dy_local * std::sin(theta2);
}

double computeEndpointY(double km, double L1)
{
  const double R = 1.0 / km;
  const double L2 = M_PI / (2.0 * km) - L1;
  if (L2 <= 0.0) return std::nan("");
  double x1, y1, theta1;
  clothoidPoint(km, L1, L1, x1, y1, theta1);
  const double theta_arc = km * L2;
  const double Ox = x1 - R * std::sin(theta1);
  const double Oy = y1 + R * std::cos(theta1);
  const double x2 = Ox + R * std::sin(theta1 + theta_arc);
  const double y2 = Oy - R * std::cos(theta1 + theta_arc);
  const double theta2 = theta1 + theta_arc;
  double dx_local, dy_local;
  clothoidLocalOffset(km, L1, L1, dx_local, dy_local);
  (void)x2;
  return y2 + dx_local * std::sin(theta2) + dy_local * std::cos(theta2);
}

void samplingPointsXY(
  int pts_num, double ds_1, double ds_2, double dk_ds, double kappa_max,
  std::vector<double> & x_pts, std::vector<double> & y_pts)
{
  const double s_sum = 2.0 * ds_1 + ds_2;
  const double ds_d = s_sum / pts_num;
  x_pts.resize(pts_num);
  y_pts.resize(pts_num);

  const double R = 1.0 / kappa_max;
  double x1, y1, theta1;
  clothoidPoint(kappa_max, ds_1, ds_1, x1, y1, theta1);
  const double Ox = x1 - R * std::sin(theta1);
  const double Oy = y1 + R * std::cos(theta1);
  const double theta_pts2 = theta1 + M_PI / 2.0 - kappa_max * ds_1;
  const double x2 = Ox + R * std::cos(theta1 - M_PI / 2.0 + (ds_2 / R));
  const double y2 = Oy + R * std::sin(theta1 - M_PI / 2.0 + ds_2 / R);

  const double A = std::sqrt(ds_1 / kappa_max);
  const double B = std::sqrt(kappa_max * ds_1 / M_PI);
  const double S_B = fresnelS(B);
  const double C_B = fresnelC(B);

  for (int m = 0; m < pts_num; ++m) {
    const double s = ds_d * (m + 1);
    if (s <= ds_1) {
      const double u = std::sqrt(dk_ds / M_PI) * s;
      x_pts[m] = (1.0 / std::sqrt(dk_ds / M_PI)) * fresnelC(u);
      y_pts[m] = (1.0 / std::sqrt(dk_ds / M_PI)) * fresnelS(u);
    } else if (s > ds_1 && s < ds_1 + ds_2) {
      const double ds = s - ds_1;
      x_pts[m] = Ox + R * std::cos(theta1 - M_PI / 2.0 + ds / R);
      y_pts[m] = Oy + R * std::sin(theta1 - M_PI / 2.0 + ds / R);
    } else {
      const double ds = s - ds_1 - ds_2;
      const double t = 1.0 - ds / ds_1;
      const double u = B * t;
      x_pts[m] = x2 + A * std::sqrt(M_PI) * (S_B - fresnelS(u));
      y_pts[m] = y2 + A * std::sqrt(M_PI) * (C_B - fresnelC(u));
      (void)theta_pts2;
    }
  }
}

void samplingPointsYaw(
  int pts_num, double ds_1, double ds_2, double dk_ds, double kappa_max,
  std::vector<double> & theta_pts, std::vector<double> & kappa_pts)
{
  const double s_sum = 2.0 * ds_1 + ds_2;
  const double ds_d = s_sum / pts_num;
  theta_pts.resize(pts_num);
  kappa_pts.resize(pts_num);
  const double theta_pts1 = 0.5 * dk_ds * ds_1 * ds_1;
  const double theta_pts2 = theta_pts1 + M_PI / 2.0 - kappa_max * ds_1;
  const double R = 1.0 / kappa_max;
  for (int m = 0; m < pts_num; ++m) {
    const double s = ds_d * (m + 1);
    if (s <= ds_1) {
      theta_pts[m] = 0.5 * dk_ds * s * s;
      kappa_pts[m] = dk_ds * s;
    } else if (s > ds_1 && s < ds_1 + ds_2) {
      const double ds = s - ds_1;
      theta_pts[m] = theta_pts1 + ds / R;
      kappa_pts[m] = kappa_max;
    } else {
      const double ds = s - ds_1 - ds_2;
      const double dtheta = kappa_max * ds - (dk_ds / 2.0) * ds * ds;
      theta_pts[m] = theta_pts2 + dtheta;
      kappa_pts[m] = kappa_max - dk_ds * ds;
    }
  }
}

}  // namespace

std::vector<autoware_planning_msgs::msg::PathPoint> PathGenerator::generateQTurnSegment(
  const autoware_planning_msgs::msg::PathPoint & start,
  const autoware_planning_msgs::msg::PathPoint & end)
{
  std::vector<autoware_planning_msgs::msg::PathPoint> points;
  last_qturn_fell_back_ = false;

  const double start_yaw = getYaw(start.pose.orientation);
  const double end_yaw = getYaw(end.pose.orientation);

  // Compute yaw difference (signed). Algorithm is hardcoded for left turns;
  // mirror local y and output sign for right turns.
  const double yaw_diff_signed = normalizeAngle(end_yaw - start_yaw);
  const double yaw_diff = std::abs(yaw_diff_signed);
  const double turn_sign = (yaw_diff_signed >= 0) ? 1.0 : -1.0;

  // Gate: strict ~90° elbow window. Outside this, fall back to Bezier turn.
  if (yaw_diff < params_.qturn_yaw_min_threshold ||
      yaw_diff > params_.qturn_yaw_max_threshold) {
    if (params_.debug) {
      std::cerr << "[PathGen] Q_TURN: yaw_diff=" << (yaw_diff * 180.0 / M_PI)
                << " deg outside [75,109] — falling back to TURN" << std::endl;
    }
    last_qturn_fell_back_ = true;
    return generateTurnSegment(start, end);
  }

  // Check if end point is a goal
  const bool end_is_goal = (std::abs(end.longitudinal_velocity_mps) < 0.1);
  const double start_vel = start.longitudinal_velocity_mps;
  const double qturn_vel = (start_vel >= 0)
    ? std::min(static_cast<double>(start_vel), params_.qturn_speed_limit)
    : std::max(static_cast<double>(start_vel), -params_.qturn_speed_limit);

  // Transform end position into MOTION-aligned local frame; mirror y so the
  // curve math always runs as a left turn. For reverse driving, motion is
  // opposite to vehicle facing, so the spatial frame must align with motion
  // (otherwise dx_local would be negative and the leg_a/leg_b reachability
  // checks would force a fallback to Bezier). Output vehicle facing yaw still
  // uses start_yaw — the body rotates the same way regardless of motion
  // direction.
  const double motion_yaw = is_reverse_ ? (start_yaw + M_PI) : start_yaw;
  const double cos_m = std::cos(motion_yaw);
  const double sin_m = std::sin(motion_yaw);
  const double dx_world = end.pose.position.x - start.pose.position.x;
  const double dy_world = end.pose.position.y - start.pose.position.y;
  const double dx_local =  dx_world * cos_m + dy_world * sin_m;
  const double dy_local_raw = -dx_world * sin_m + dy_world * cos_m;
  const double dy_local = dy_local_raw * turn_sign;

  // In the (start-aligned, y-mirrored) local frame, start tangent is +x and
  // end tangent is +y (since mirrored yaw_diff = +π/2). The apex — the
  // intersection of the two tangent rays — sits at (dx_local, 0). Leg
  // lengths from each endpoint to the apex are therefore:
  const double leg_a = dx_local;   // start → apex, along +x
  const double leg_b = dy_local;   // end   → apex, along -y (reversed end tangent)
  if (leg_a <= 0.1 || leg_b <= 0.1) {
    std::cerr << "[PathGen] Q_TURN fallback: leg_a=" << leg_a << " leg_b=" << leg_b
              << " (apex not reachable from both tangent rays)" << std::endl;
    last_qturn_fell_back_ = true;
    return generateTurnSegment(start, end);
  }

  // Position clothoid. Inscribe a SYMMETRIC 90° clothoid-arc-clothoid whose
  // tangent distance t (= endpoint X = endpoint Y) is clamped to
  // min(leg_a, leg_b); pad the remaining leg length with a straight segment.
  const double km = 1.0 / params_.qturn_min_radius;
  const double R  = params_.qturn_min_radius;
  const double t_need = std::min(leg_a, leg_b);
  if (t_need < R - 1e-3) {
    // Even a pure arc of apex radius R has tangent distance R; can't fit.
    std::cerr << "[PathGen] Q_TURN fallback: t_need=" << t_need << " < R=" << R
              << " (leg_a=" << leg_a << " leg_b=" << leg_b
              << ")  shrink qturn_min_radius or widen the elbow" << std::endl;
    last_qturn_fell_back_ = true;
    return generateTurnSegment(start, end);
  }

  // Max tangent distance = endpoint X with L2 → 0 (pure Euler spiral).
  const double L1_hi = M_PI / (2.0 * km) - 1e-6;

  // The secondary yaw clothoid is parameterized over the same curve arc length:
  //   ds_total = L1 + (π/2)R
  // and is feasible only when:
  //   (π/2)R_yaw < ds_total < πR_yaw
  // Convert that feasible ds interval back into an allowed position-clothoid
  // L1 interval, then into tangent distance. If the FMS legs are longer than
  // the allowed curve, cap the curve and absorb the remainder with the existing
  // pre/post straight segments.
  const double km_yaw = 1.0 / params_.qturn_yaw_min_radius;
  const double ds_position_base = M_PI / (2.0 * km);
  const double yaw_ds_min = M_PI / (2.0 * km_yaw) + kYawClothoidDsMargin;
  const double yaw_ds_max = M_PI / km_yaw - kYawClothoidDsMargin;
  const double L1_allowed_min = std::max(1e-6, yaw_ds_min - ds_position_base);
  const double L1_allowed_max = std::min(L1_hi, yaw_ds_max - ds_position_base);

  if (!std::isfinite(L1_allowed_min) || !std::isfinite(L1_allowed_max) ||
      L1_allowed_max <= L1_allowed_min) {
    std::cerr << "[PathGen] Q_TURN fallback: no overlap between position and yaw"
              << " clothoid feasibility intervals"
              << " (R=" << R << " R_yaw=" << params_.qturn_yaw_min_radius
              << " yaw_ds=[" << yaw_ds_min << "," << yaw_ds_max
              << "] ds_base=" << ds_position_base << ")" << std::endl;
    last_qturn_fell_back_ = true;
    return generateTurnSegment(start, end);
  }

  const double t_allowed_min = computeEndpointX(km, L1_allowed_min);
  const double t_allowed_max = computeEndpointX(km, L1_allowed_max);
  if (!std::isfinite(t_allowed_min) || !std::isfinite(t_allowed_max) ||
      t_allowed_max < t_allowed_min) {
    std::cerr << "[PathGen] Q_TURN fallback: invalid yaw-feasible tangent interval"
              << " (t_allowed_min=" << t_allowed_min
              << " t_allowed_max=" << t_allowed_max << ")" << std::endl;
    last_qturn_fell_back_ = true;
    return generateTurnSegment(start, end);
  }

  if (t_need + 1e-6 < t_allowed_min) {
    std::cerr << "[PathGen] Q_TURN fallback: t_need=" << t_need
              << " < yaw-feasible tangent distance " << t_allowed_min
              << " (leg_a=" << leg_a << " leg_b=" << leg_b
              << " R=" << R << " R_yaw=" << params_.qturn_yaw_min_radius << ")"
              << std::endl;
    last_qturn_fell_back_ = true;
    return generateTurnSegment(start, end);
  }

  const double t_target = std::min(t_need, t_allowed_max);

  // Bisect L1 so that computeEndpointX(km, L1) == t_target.
  double L1 = 0.0;
  {
    double lo = 1e-6, hi = L1_hi;
    auto eqn = [km, t_target](double v) { return computeEndpointX(km, v) - t_target; };
    const double f_lo = eqn(lo);
    double sol = (lo + hi) / 2.0;
    bool ok = false;
    for (int iter = 0; iter < 100; ++iter) {
      const double fmid = eqn(sol);
      if (!std::isfinite(fmid)) { hi = sol; sol = (lo + hi) / 2.0; continue; }
      if (std::abs(fmid) < 1e-6) { ok = true; break; }
      if (f_lo * fmid < 0) hi = sol; else lo = sol;
      sol = (lo + hi) / 2.0;
    }
    if (!ok) {
      std::cerr << "[PathGen] Q_TURN fallback: L1 bisection failed after 100 iters"
                << " (t_target=" << t_target << " R=" << R << ")" << std::endl;
      last_qturn_fell_back_ = true;
      return generateTurnSegment(start, end);
    }
    L1 = sol;
  }
  const double L2 = M_PI / (2.0 * km) - L1;
  if (L2 <= 0.0) {
    std::cerr << "[PathGen] Q_TURN fallback: L2=" << L2 << " <= 0 (L1=" << L1
              << " R=" << R << ")" << std::endl;
    last_qturn_fell_back_ = true;
    return generateTurnSegment(start, end);
  }
  const double dk_ds = km / L1;
  const double ds_total = 2.0 * L1 + L2;

  // Symmetric 90° clothoid-arc-clothoid has endpoint X == Y, both == t_target.
  const double curve_end_X = computeEndpointX(km, L1);
  const double curve_end_Y = computeEndpointY(km, L1);
  const double pre_straight  = std::max(0.0, leg_a - curve_end_X);
  const double post_straight = std::max(0.0, leg_b - curve_end_Y);

  // Yaw/heading clothoid (secondary curvature) produces the 4WS sideslip β.
  const double L1_yaw = ds_total - M_PI / (2.0 * km_yaw);
  const double L2_yaw = M_PI / km_yaw - ds_total;
  const bool yaw_clothoid_feasible = (L1_yaw > 0.0 && L2_yaw > 0.0);
  if (!yaw_clothoid_feasible) {
    std::cerr << "[PathGen] Q_TURN fallback: yaw clothoid infeasible after"
              << " tangent-distance sizing"
              << " (ds_total=" << ds_total
              << " yaw_ds=[" << yaw_ds_min << "," << yaw_ds_max
              << "] L1_yaw=" << L1_yaw << " L2_yaw=" << L2_yaw << ")"
              << std::endl;
    last_qturn_fell_back_ = true;
    return generateTurnSegment(start, end);
  }
  const double dk_ds_yaw = yaw_clothoid_feasible ? (km_yaw / L1_yaw) : 0.0;

  const int pts_curve = std::max(20, static_cast<int>(ds_total / params_.path_interval));

  std::vector<double> x_pts, y_pts;
  std::vector<double> yaw_pts_drive, kappa_pts_drive;
  std::vector<double> yaw_pts_path, kappa_pts_path;
  samplingPointsXY(pts_curve, L1, L2, dk_ds, km, x_pts, y_pts);
  samplingPointsYaw(pts_curve, L1, L2, dk_ds, km, yaw_pts_path, kappa_pts_path);
  samplingPointsYaw(pts_curve, L1_yaw, L2_yaw, dk_ds_yaw, km_yaw,
                    yaw_pts_drive, kappa_pts_drive);

  const double abs_vx = std::abs(qturn_vel);
  const double interval = std::max(params_.path_interval, 0.1);

  // Emit in world frame. Local→world: p_world = start + R(motion_yaw) * (xl, yl*turn_sign).
  // Vehicle facing yaw stays anchored at start_yaw + turn_sign * yaw_offset_local
  // (body rotation is direction-agnostic). heading_rate uses |v| so its sign comes
  // only from turn_sign — signed qturn_vel would flip it in reverse.
  auto emit_world = [&](double xl_mirror, double yl_mirror, double yaw_offset_local,
                        double kappa_drive, bool is_last_flag) {
    const double xl = xl_mirror;
    const double yl = yl_mirror * turn_sign;
    autoware_planning_msgs::msg::PathPoint pt;
    pt.pose.position.x = start.pose.position.x + xl * cos_m - yl * sin_m;
    pt.pose.position.y = start.pose.position.y + xl * sin_m + yl * cos_m;
    pt.pose.position.z = start.pose.position.z;
    pt.pose.orientation = createQuaternionFromYaw(start_yaw + turn_sign * yaw_offset_local);
    pt.longitudinal_velocity_mps =
      (is_last_flag && end_is_goal) ? 0.0f : static_cast<float>(qturn_vel);
    pt.lateral_velocity_mps = 0.0f;  // overridden below for curve pts
    pt.heading_rate_rps = static_cast<float>(turn_sign * abs_vx * kappa_drive);
    pt.is_final = is_last_flag;
    points.push_back(pt);
    return points.size() - 1;
  };

  // 1) Pre-straight along +x in local-mirrored frame, heading 0.
  if (pre_straight > 0.5 * interval) {
    const int n_pre = std::max(1, static_cast<int>(pre_straight / interval));
    for (int i = 0; i < n_pre; ++i) {
      const double xl = (static_cast<double>(i) / n_pre) * pre_straight;
      emit_world(xl, 0.0, 0.0, 0.0, false);
    }
  } else {
    emit_world(0.0, 0.0, 0.0, 0.0, false);
  }

  // 2) Curve — offset by pre_straight in local +x.
  for (int i = 0; i < pts_curve; ++i) {
    const double xl = pre_straight + x_pts[i];
    const double yl = y_pts[i];
    const double beta = yaw_pts_path[i] - yaw_pts_drive[i];
    const size_t idx = emit_world(xl, yl, yaw_pts_drive[i], kappa_pts_drive[i], false);
    // Body-frame y-rate: in reverse the motion frame is rotated π from the body
    // frame, flipping body_y velocity. Using signed qturn_vel (negative in
    // reverse) absorbs the sign flip — same trick as oblique_generator.
    points[idx].lateral_velocity_mps = static_cast<float>(turn_sign * beta * qturn_vel);
  }

  // 3) Post-straight along end tangent. In local-mirrored frame, end tangent
  //    is +y from the curve endpoint at (leg_a, curve_end_Y) heading +π/2.
  if (post_straight > 0.5 * interval) {
    const int n_post = std::max(1, static_cast<int>(post_straight / interval));
    for (int i = 1; i <= n_post; ++i) {
      const double yl = curve_end_Y + (static_cast<double>(i) / n_post) * post_straight;
      const bool is_last = (i == n_post);
      emit_world(leg_a, yl, M_PI / 2.0, 0.0, is_last);
    }
  } else {
    // Ensure the final point is exactly the goal with the goal heading.
    if (!points.empty()) {
      points.back().pose.position.x = end.pose.position.x;
      points.back().pose.position.y = end.pose.position.y;
      points.back().pose.orientation = createQuaternionFromYaw(end_yaw);
      points.back().is_final = true;
      points.back().longitudinal_velocity_mps =
        end_is_goal ? 0.0f : static_cast<float>(qturn_vel);
    }
  }

  if (params_.debug) {
    std::cerr << "[PathGen] Q_TURN: yaw_diff=" << (yaw_diff * 180.0 / M_PI)
              << " deg, leg_a=" << leg_a << " leg_b=" << leg_b
              << " t_target=" << t_target
              << " L1=" << L1 << " L2=" << L2
              << " ds_total=" << ds_total
              << " yaw_ds=[" << yaw_ds_min << "," << yaw_ds_max << "]"
              << " pre=" << pre_straight << " post=" << post_straight
              << " pts=" << points.size()
              << " reverse=" << (is_reverse_ ? 1 : 0) << std::endl;
  }

  return points;
}

}  // namespace igv_path_planner
