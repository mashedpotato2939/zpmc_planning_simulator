#include "simple_planning_simulator/vehicle_model/sim_model_delay_steer_vel_two_axis.hpp"

#include <algorithm>
#include <iostream>
#include <cmath>

SimModelDelaySteerVelTwoAxis::SimModelDelaySteerVelTwoAxis(
  double vx_lim, double steer_lim, double vx_rate_lim, double steer_rate_lim, double wheelbase_f, double wheelbase_r,
  double dt, double vx_delay, double vx_time_constant, double steer_delay,
  double steer_time_constant, double steer_dead_band, double steer_bias)
: SimModelInterface(6 /* dim x */, 3 /* dim u */),
  MIN_TIME_CONSTANT(0.03),
  vx_lim_(vx_lim),
  vx_rate_lim_(vx_rate_lim),
  steer_lim_(steer_lim),
  steer_rate_lim_(steer_rate_lim),
  wheelbase_f(wheelbase_f),
  wheelbase_r(wheelbase_r),
  vx_delay_(vx_delay),
  vx_time_constant_(std::max(vx_time_constant, MIN_TIME_CONSTANT)),
  steer_delay_(steer_delay),
  steer_time_constant_(std::max(steer_time_constant, MIN_TIME_CONSTANT)),
  steer_dead_band_(steer_dead_band),
  steer_bias_(steer_bias)
{
  initializeInputQueue(dt);
}

double SimModelDelaySteerVelTwoAxis::getX()
{
  return state_(IDX::X);
}
double SimModelDelaySteerVelTwoAxis::getY()
{
  return state_(IDX::Y);
}
double SimModelDelaySteerVelTwoAxis::getYaw()
{
  return state_(IDX::YAW);
}
double SimModelDelaySteerVelTwoAxis::getVx()
{
  return state_(IDX::VX);
}
double SimModelDelaySteerVelTwoAxis::getVy()
{
  return beta_ * state_(IDX::VX);
}
double SimModelDelaySteerVelTwoAxis::getAx()
{
  return current_ax_;
}
double SimModelDelaySteerVelTwoAxis::getWz()
{
  return omega_;
}
double SimModelDelaySteerVelTwoAxis::getSteer()
{
  return state_(IDX::STEER_F) + steer_bias_;
}

double SimModelDelaySteerVelTwoAxis::getSteerF()
{
  return state_(IDX::STEER_F) + steer_bias_;
}

double SimModelDelaySteerVelTwoAxis::getSteerR()
{
  return state_(IDX::STEER_R) + steer_bias_;
}


void SimModelDelaySteerVelTwoAxis::update(const double & dt)
{
  Eigen::VectorXd delayed_input = Eigen::VectorXd::Zero(dim_u_);

  vx_input_queue_.push_back(input_(IDX_U::VX_DES));
  delayed_input(IDX_U::VX_DES) = vx_input_queue_.front();
  vx_input_queue_.pop_front();

  steer_f_input_queue_.push_back(input_(IDX_U::STEER_F_DES));
  delayed_input(IDX_U::STEER_F_DES) = steer_f_input_queue_.front();
  steer_f_input_queue_.pop_front();

  steer_r_input_queue_.push_back(input_(IDX_U::STEER_R_DES));
  delayed_input(IDX_U::STEER_R_DES) = steer_r_input_queue_.front();
  steer_r_input_queue_.pop_front();

  // do not use deadzone_delta_steer (Steer IF does not exist in this model)
  updateRungeKutta(dt, delayed_input);
  current_ax_ = (input_(IDX_U::VX_DES) - prev_vx_) / dt;
  prev_vx_ = input_(IDX_U::VX_DES);
}

void SimModelDelaySteerVelTwoAxis::initializeInputQueue(const double & dt)
{
  size_t vx_input_queue_size = static_cast<size_t>(round(vx_delay_ / dt));
  for (size_t i = 0; i < vx_input_queue_size; i++) {
    vx_input_queue_.push_back(0.0);
  }

  size_t steer_input_queue_size = static_cast<size_t>(round(steer_delay_ / dt));
  for (size_t i = 0; i < steer_input_queue_size; i++) {
    steer_f_input_queue_.push_back(0.0);
    steer_r_input_queue_.push_back(0.0);
  }
}

Eigen::VectorXd SimModelDelaySteerVelTwoAxis::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  auto sat = [](double val, double u, double l) { return std::max(std::min(val, u), l); };

  const double vx = sat(state(IDX::VX), vx_lim_, -vx_lim_);
  const double steer_f = sat(state(IDX::STEER_F), steer_lim_, -steer_lim_);
  const double steer_r = sat(state(IDX::STEER_R), steer_lim_, -steer_lim_);
  const double yaw = state(IDX::YAW);

  const double delay_input_vx = input(IDX_U::VX_DES);
  const double delay_input_steer_f = input(IDX_U::STEER_F_DES);
  const double delay_input_steer_r = input(IDX_U::STEER_R_DES);

  const double delay_vx_des = sat(delay_input_vx, vx_lim_, -vx_lim_);
  const double vx_rate = sat(-(vx - delay_vx_des) / vx_time_constant_, vx_rate_lim_, -vx_rate_lim_);

  const double delay_steer_f_des = sat(delay_input_steer_f, steer_lim_, -steer_lim_);
  const double steer_diff_f = getSteerF() - delay_steer_f_des;
  const double delay_steer_r_des = sat(delay_input_steer_r, steer_lim_, -steer_lim_);
  const double steer_diff_r = getSteerR() - delay_steer_r_des;

  const auto dead_band = [&](const auto &steer_diff){
    if (steer_diff > steer_dead_band_) {
      return steer_diff - steer_dead_band_;
    } else if (steer_diff < -steer_dead_band_) {
      return steer_diff + steer_dead_band_;
    } else {
      return 0.0;
    }
  };

  const double steer_diff_f_with_deadband = dead_band(steer_diff_f);
  const double steer_diff_r_with_deadband = dead_band(steer_diff_r);
  const double steer_rate_f = sat(-steer_diff_f_with_deadband / steer_time_constant_, steer_rate_lim_, -steer_rate_lim_);
  const double steer_rate_r = sat(-steer_diff_r_with_deadband / steer_time_constant_, steer_rate_lim_, -steer_rate_lim_);

  const double beta_num = wheelbase_f * tan(steer_r) + wheelbase_r * tan(steer_f);
  const double beta_den = wheelbase_f + wheelbase_r;
  beta_ = atan(beta_num/beta_den);

  const double kapp_num = tan(steer_f) - tan(steer_r);
  const double kapp_den = std::sqrt(pow(beta_num,2) + pow(beta_den,2));
  kappa_ = kapp_num/kapp_den;

  const double vy = vx * beta_;

  v_ = vx * std::sqrt(1 + beta_ * beta_);
  omega_ = v_ * kappa_;

  // Debug logging for 4WS kinematics (commented out - high frequency)
  // static int log_counter = 0;
  // if (++log_counter >= 30) {  // Log every ~1 second at 30Hz
  //   log_counter = 0;
  //   // Detect 4WS mode: if front and rear have same sign = crab, opposite sign = q-turn
  //   bool is_4ws_active = std::abs(steer_f) > 0.01 || std::abs(steer_r) > 0.01;
  //   if (is_4ws_active) {
  //     const char* mode = "NORMAL";
  //     if (std::abs(steer_r) > 0.01) {
  //       if (steer_f * steer_r > 0) {
  //         mode = "CRAB/OBLIQUE";  // Same sign = crab motion
  //       } else {
  //         mode = "Q-TURN";  // Opposite sign = reduced turning radius
  //       }
  //     }
  //     std::cerr << "[SimModel2Axis] " << mode
  //               << ": steer_f=" << steer_f * 180.0 / M_PI << " deg"
  //               << ", steer_r=" << steer_r * 180.0 / M_PI << " deg"
  //               << ", beta=" << beta_ * 180.0 / M_PI << " deg"
  //               << ", kappa=" << kappa_
  //               << ", omega=" << omega_ << " rad/s"
  //               << ", vx=" << vx << " m/s"
  //               << ", vy=" << vy << " m/s\n";
  //   }
  // }

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X) = vx * cos(yaw) - vy * sin(yaw);
  d_state(IDX::Y) = vx * sin(yaw) + vy * cos(yaw);
  d_state(IDX::YAW) = omega_;
  d_state(IDX::VX) = vx_rate;
  d_state(IDX::STEER_F) = steer_rate_f;
  d_state(IDX::STEER_R) = steer_rate_r;

  return d_state;
}