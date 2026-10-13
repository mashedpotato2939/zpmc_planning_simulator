#ifndef FMS_TRAJECTORY_BRIDGE__TEST__REFERENCE_MAP_TRAJ_QTURN_HPP_
#define FMS_TRAJECTORY_BRIDGE__TEST__REFERENCE_MAP_TRAJ_QTURN_HPP_

// Free-function port of the Q_TURN math from
//   motion_control/src/map_traj_server/map_traj_server.cpp (lines 1138..1319)
// stripped of Node/service wrapping and stdout prints. Keep function names
// identical to the source so drift is reviewable against the upstream file.

#include <cmath>
#include <functional>
#include <vector>

#include <geometry_msgs/msg/point.hpp>

namespace reference_qturn
{

struct ReferencePoint
{
  double x;
  double y;
  double yaw;
};

inline double adaptive_simpson(
  const std::function<double(double)> & f, double a, double b,
  double eps, int max_depth)
{
  auto simpson = [](const std::function<double(double)> & g, double x0, double x1) {
    return (x1 - x0) / 6.0 * (g(x0) + 4.0 * g((x0 + x1) / 2.0) + g(x1));
  };
  const double m = (a + b) / 2.0;
  const double whole = simpson(f, a, b);
  const double left  = simpson(f, a, m);
  const double right = simpson(f, m, b);
  if (std::abs(left + right - whole) < 15.0 * eps || max_depth <= 0) {
    return left + right + (left + right - whole) / 15.0;
  }
  return adaptive_simpson(f, a, m, eps / 2.0, max_depth - 1) +
         adaptive_simpson(f, m, b, eps / 2.0, max_depth - 1);
}

inline double fresnelc(double x)
{
  if (x == 0.0) return 0.0;
  auto f = [](double t) { return std::cos(M_PI / 2.0 * t * t); };
  return adaptive_simpson(f, 0.0, x, 1e-9, 20);
}

inline double fresnels(double x)
{
  if (x == 0.0) return 0.0;
  auto f = [](double t) { return std::sin(M_PI / 2.0 * t * t); };
  return adaptive_simpson(f, 0.0, x, 1e-9, 20);
}

inline void clothoid_point(
  double km, double L1, double s, double & x, double & y, double & theta)
{
  const double A = std::sqrt(L1 / km);
  const double u = s / (A * std::sqrt(M_PI));
  const double C = fresnelc(u);
  const double S = fresnels(u);
  x = A * std::sqrt(M_PI) * C;
  y = A * std::sqrt(M_PI) * S;
  theta = (km / (2.0 * L1)) * s * s;
}

inline void clothoid_local_offset(
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
  dx = adaptive_simpson(cos_theta, 0.0, s, 1e-9, 20);
  dy = adaptive_simpson(sin_theta, 0.0, s, 1e-9, 20);
}

inline double compute_endpoint_x(double km, double L1)
{
  const double R = 1.0 / km;
  const double L2 = M_PI / (2.0 * km) - L1;
  if (L2 <= 0.0) return NAN;
  double x1, y1, theta1;
  clothoid_point(km, L1, L1, x1, y1, theta1);
  const double theta_arc = km * L2;
  const double Ox = x1 - R * std::sin(theta1);
  const double x2 = Ox + R * std::sin(theta1 + theta_arc);
  const double theta2 = theta1 + theta_arc;
  double dx_local, dy_local;
  clothoid_local_offset(km, L1, L1, dx_local, dy_local);
  return x2 + dx_local * std::cos(theta2) - dy_local * std::sin(theta2);
}

inline double compute_endpoint_y(double km, double L1)
{
  const double R = 1.0 / km;
  const double L2 = M_PI / (2.0 * km) - L1;
  if (L2 <= 0.0) return NAN;
  double x1, y1, theta1;
  clothoid_point(km, L1, L1, x1, y1, theta1);
  const double theta_arc = km * L2;
  const double Oy = y1 + R * std::cos(theta1);
  const double y2 = Oy - R * std::cos(theta1 + theta_arc);
  const double theta2 = theta1 + theta_arc;
  double dx_local, dy_local;
  clothoid_local_offset(km, L1, L1, dx_local, dy_local);
  return y2 + dx_local * std::sin(theta2) + dy_local * std::cos(theta2);
}

inline void solve_clothoid_arc(
  double km, double dx, double /*dy*/, double & L1, double & L2)
{
  double L1_low = 1e-6;
  double L1_high = M_PI / (2.0 * km) - 1e-6;
  auto eqn = [km, dx](double L1v) { return compute_endpoint_x(km, L1v) - dx; };
  double L1_sol = (L1_low + L1_high) / 2.0;
  const double f_low = eqn(L1_low);
  for (int iter = 0; iter < 100; ++iter) {
    const double fmid = eqn(L1_sol);
    if (std::abs(fmid) < 1e-9) break;
    if (f_low * fmid < 0) { L1_high = L1_sol; } else { L1_low = L1_sol; }
    L1_sol = (L1_low + L1_high) / 2.0;
  }
  L1 = L1_sol;
  L2 = M_PI / (2.0 * km) - L1;
}

inline void sampling_points_xy(
  int pts_num, double ds_1, double ds_2, double dk_ds, double kappa_max,
  std::vector<double> & x_pts, std::vector<double> & y_pts)
{
  const double s_sum = 2.0 * ds_1 + ds_2;
  const double ds_d  = s_sum / pts_num;
  x_pts.resize(pts_num);
  y_pts.resize(pts_num);

  const double R = 1.0 / kappa_max;
  double x1, y1, theta1;
  clothoid_point(kappa_max, ds_1, ds_1, x1, y1, theta1);
  const double Ox = x1 - R * std::sin(theta1);
  const double Oy = y1 + R * std::cos(theta1);
  const double theta_pts2 = theta1 + M_PI / 2.0 - kappa_max * ds_1;
  const double x2 = Ox + R * std::cos(theta1 - M_PI / 2.0 + (ds_2 / R));
  const double y2 = Oy + R * std::sin(theta1 - M_PI / 2.0 + ds_2 / R);

  const double A = std::sqrt(ds_1 / kappa_max);
  const double B = std::sqrt(kappa_max * ds_1 / M_PI);
  const double S_B = fresnels(B);
  const double C_B = fresnelc(B);

  for (int m = 0; m < pts_num; ++m) {
    const double s = ds_d * (m + 1);
    if (s <= ds_1) {
      const double u = std::sqrt(dk_ds / M_PI) * s;
      x_pts[m] = (1.0 / std::sqrt(dk_ds / M_PI)) * fresnelc(u);
      y_pts[m] = (1.0 / std::sqrt(dk_ds / M_PI)) * fresnels(u);
    } else if (s > ds_1 && s < ds_1 + ds_2) {
      const double ds = s - ds_1;
      x_pts[m] = Ox + R * std::cos(theta1 - M_PI / 2.0 + ds / R);
      y_pts[m] = Oy + R * std::sin(theta1 - M_PI / 2.0 + ds / R);
    } else {
      const double ds = s - ds_1 - ds_2;
      const double t  = 1.0 - ds / ds_1;
      const double u  = B * t;
      x_pts[m] = x2 + A * std::sqrt(M_PI) * (S_B - fresnels(u));
      y_pts[m] = y2 + A * std::sqrt(M_PI) * (C_B - fresnelc(u));
    }
  }
  (void)theta_pts2;
}

inline void sampling_points_yaw(
  int pts_num, double ds_1, double ds_2, double dk_ds, double kappa_max,
  std::vector<double> & theta_pts, std::vector<double> & kappa_pts)
{
  const double s_sum = 2.0 * ds_1 + ds_2;
  const double ds_d  = s_sum / pts_num;
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

// Port of MapTrajServer::traj_creator_turn_optimize (map_traj_server.cpp:1256).
// Mirrors the reference exactly: position clothoid at R, heading clothoid at
// R_yaw. Returns points in world frame translated by `start` (no rotation,
// matching the reference which assumes start_yaw = 0).
inline std::vector<ReferencePoint> generate_reference_qturn(
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & end,
  double /*vx*/,
  double R,
  double R_yaw,
  int pts_num)
{
  const double kappa_max = 1.0 / R;
  const double end_x_rel = end.x - start.x;
  const double end_y_rel = end.y - start.y;

  double L1 = 0.0, L2 = 0.0;
  solve_clothoid_arc(kappa_max, end_x_rel, end_y_rel, L1, L2);
  const double dk_ds = kappa_max / L1;

  const double kappa_max2 = 1.0 / R_yaw;
  const double ds_total   = 2.0 * L1 + L2;
  const double L1_yaw     = ds_total - M_PI / 2.0 / kappa_max2;
  const double L2_yaw     = M_PI / kappa_max2 - ds_total;
  const double dk_ds_yaw  = kappa_max2 / L1_yaw;

  std::vector<double> x_pts, y_pts, yaw_pts, kappa_pts;
  sampling_points_xy(pts_num, L1, L2, dk_ds, kappa_max, x_pts, y_pts);
  sampling_points_yaw(
    pts_num, L1_yaw, L2_yaw, dk_ds_yaw, kappa_max2, yaw_pts, kappa_pts);

  std::vector<ReferencePoint> out;
  out.reserve(pts_num);
  for (int i = 0; i < pts_num; ++i) {
    ReferencePoint p;
    p.x   = start.x + x_pts[i];
    p.y   = start.y + y_pts[i];
    p.yaw = yaw_pts[i];
    out.push_back(p);
  }
  return out;
}

}  // namespace reference_qturn

#endif  // FMS_TRAJECTORY_BRIDGE__TEST__REFERENCE_MAP_TRAJ_QTURN_HPP_
