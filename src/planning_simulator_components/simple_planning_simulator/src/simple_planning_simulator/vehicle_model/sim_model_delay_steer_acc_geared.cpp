// Copyright 2021 The Autoware Foundation.
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

#include "simple_planning_simulator/vehicle_model/sim_model_delay_steer_acc_geared.hpp"

#include "autoware_vehicle_msgs/msg/gear_command.hpp"

#include <algorithm>

SimModelDelaySteerAccGeared::SimModelDelaySteerAccGeared(
  double vx_lim, double steer_lim, double vx_rate_lim, double steer_rate_lim, double wheelbase,
  double dt, double acc_delay, double acc_time_constant, double steer_delay,
  double steer_time_constant, double steer_dead_band, double steer_bias,
  double debug_acc_scaling_factor, double debug_steer_scaling_factor,
  bool enable_4ws, double rear_wheelbase)
: SimModelInterface(6 /* dim x */, 2 /* dim u */),
  MIN_TIME_CONSTANT(0.03),
  vx_lim_(vx_lim),
  vx_rate_lim_(vx_rate_lim),
  steer_lim_(steer_lim),
  steer_rate_lim_(steer_rate_lim),
  wheelbase_(wheelbase),
  enable_4ws_(enable_4ws),
  rear_wheelbase_(rear_wheelbase),
  acc_delay_(acc_delay),
  acc_time_constant_(std::max(acc_time_constant, MIN_TIME_CONSTANT)),
  steer_delay_(steer_delay),
  steer_time_constant_(std::max(steer_time_constant, MIN_TIME_CONSTANT)),
  steer_dead_band_(steer_dead_band),
  steer_bias_(steer_bias),
  debug_acc_scaling_factor_(std::max(debug_acc_scaling_factor, 0.0)),
  debug_steer_scaling_factor_(std::max(debug_steer_scaling_factor, 0.0))
{
  initializeInputQueue(dt);
}

double SimModelDelaySteerAccGeared::getX()
{
  return state_(IDX::X);
}
double SimModelDelaySteerAccGeared::getY()
{
  return state_(IDX::Y);
}
double SimModelDelaySteerAccGeared::getYaw()
{
  return state_(IDX::YAW);
}
double SimModelDelaySteerAccGeared::getVx()
{
  return state_(IDX::VX);
}
double SimModelDelaySteerAccGeared::getVy()
{
  // Return the lateral velocity set from trajectory for 4WS simulation
  return enable_4ws_ ? lateral_velocity_ : 0.0;
}
double SimModelDelaySteerAccGeared::getAx()
{
  return state_(IDX::ACCX);
}
double SimModelDelaySteerAccGeared::getWz()
{
  return state_(IDX::VX) * std::tan(state_(IDX::STEER)) / wheelbase_;
}
double SimModelDelaySteerAccGeared::getSteer()
{
  // return measured values with bias added to actual values
  return state_(IDX::STEER) + steer_bias_;
}
void SimModelDelaySteerAccGeared::update(const double & dt)
{
  Eigen::VectorXd delayed_input = Eigen::VectorXd::Zero(dim_u_);

  acc_input_queue_.push_back(input_(IDX_U::ACCX_DES));
  delayed_input(IDX_U::ACCX_DES) = acc_input_queue_.front();
  acc_input_queue_.pop_front();
  steer_input_queue_.push_back(input_(IDX_U::STEER_DES));
  delayed_input(IDX_U::STEER_DES) = steer_input_queue_.front();
  steer_input_queue_.pop_front();

  const auto prev_state = state_;
  updateRungeKutta(dt, delayed_input);

  // take velocity limit explicitly
  state_(IDX::VX) = std::max(-vx_lim_, std::min(state_(IDX::VX), vx_lim_));

  // consider gear
  // update position and velocity first, and then acceleration is calculated naturally
  updateStateWithGear(state_, prev_state, gear_, dt);
}

void SimModelDelaySteerAccGeared::initializeInputQueue(const double & dt)
{
  size_t acc_input_queue_size = static_cast<size_t>(round(acc_delay_ / dt));
  acc_input_queue_.resize(acc_input_queue_size);
  std::fill(acc_input_queue_.begin(), acc_input_queue_.end(), 0.0);

  size_t steer_input_queue_size = static_cast<size_t>(round(steer_delay_ / dt));
  steer_input_queue_.resize(steer_input_queue_size);
  std::fill(steer_input_queue_.begin(), steer_input_queue_.end(), 0.0);
}

Eigen::VectorXd SimModelDelaySteerAccGeared::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  // Debug: Print all flags every 200 cycles
  static int debug_all_count = 0;
  if (++debug_all_count % 200 == 0) {
    std::cerr << "[SimModel] FLAGS: enable_4ws_=" << enable_4ws_
              << ", is_rear_steering_defined_=" << is_rear_steering_defined_
              << ", is_crab_mode_=" << is_crab_mode_ << std::endl;
  }

  auto sat = [](double val, double u, double l) { return std::max(std::min(val, u), l); };

  const double vel = sat(state(IDX::VX), vx_lim_, -vx_lim_);
  const double acc = sat(state(IDX::ACCX), vx_rate_lim_, -vx_rate_lim_);
  const double yaw = state(IDX::YAW);
  const double steer = state(IDX::STEER);  // front steering
  const double acc_des =
    sat(input(IDX_U::ACCX_DES), vx_rate_lim_, -vx_rate_lim_) * debug_acc_scaling_factor_;
  const double steer_des =
    sat(input(IDX_U::STEER_DES), steer_lim_, -steer_lim_) * debug_steer_scaling_factor_;
  // NOTE: `steer_des` is calculated by control from measured values. getSteer() also gets the
  // measured value. The steer_rate used in the motion calculation is obtained from these
  // differences.
  const double steer_diff = getSteer() - steer_des;
  const double steer_diff_with_dead_band = std::invoke([&]() {
    if (steer_diff > steer_dead_band_) {
      return steer_diff - steer_dead_band_;
    } else if (steer_diff < -steer_dead_band_) {
      return steer_diff + steer_dead_band_;
    } else {
      return 0.0;
    }
  });
  const double steer_rate =
    sat(-steer_diff_with_dead_band / steer_time_constant_, steer_rate_lim_, -steer_rate_lim_);

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);

  // Calculate yaw_rate and lateral velocity based on steering mode
  double yaw_rate;
  double vy;

  if (enable_4ws_ && is_rear_steering_defined_) {
    // 4WS MODE: Use explicit rear steering from control command
    const double rear_steer_cmd = rear_steering_angle_;

    // Use is_crab_mode_ flag set from control command (BEFORE delay queue)
    // This ensures consistent crab detection regardless of steering delay mismatch
    if (is_crab_mode_) {
      // CRAB MODE: Force yaw_rate = 0 regardless of steering delay mismatch
      // This ensures constant heading during crab motion
      yaw_rate = 0.0;

      // Lateral velocity from front steering state (use delayed/filtered value)
      vy = vel * std::tan(steer);

      // Debug output
      static int crab_debug_count = 0;
      if (++crab_debug_count % 100 == 0) {
        std::cerr << "[SimModel] CRAB MODE ACTIVE: yaw_rate=0, vy=" << vy
                  << ", steer=" << steer * 180.0 / M_PI << " deg" << std::endl;
      }
    } else {
      // 4WS turning mode: front != rear (e.g., Q-turn)
      // Use state steering for front (delayed) and commanded for rear
      yaw_rate = vel * (std::tan(steer) - std::tan(rear_steer_cmd)) / wheelbase_;

      // Lateral velocity from average steering angle
      const double crab_angle = (steer + rear_steer_cmd) / 2.0;
      vy = vel * std::tan(crab_angle);

      // Debug: not in crab mode
      static int qturn_debug_count = 0;
      if (++qturn_debug_count % 100 == 0) {
        std::cerr << "[SimModel] Q-TURN MODE: yaw_rate=" << yaw_rate
                  << ", is_crab_mode_=" << is_crab_mode_ << std::endl;
      }
    }
  } else if (enable_4ws_) {
    // Legacy 4WS: Use lateral_velocity from trajectory
    yaw_rate = vel * std::tan(steer) / wheelbase_;
    vy = lateral_velocity_;

    // Debug: legacy 4WS
    static int legacy_debug_count = 0;
    if (++legacy_debug_count % 100 == 0) {
      std::cerr << "[SimModel] LEGACY 4WS: enable_4ws_=" << enable_4ws_
                << ", is_rear_defined=" << is_rear_steering_defined_
                << ", yaw_rate=" << yaw_rate << std::endl;
    }
  } else {
    // Standard bicycle model
    yaw_rate = vel * std::tan(steer) / wheelbase_;
    vy = 0.0;
  }

  // Position update with lateral velocity (crab motion)
  // dx/dt = vx * cos(yaw) - vy * sin(yaw)
  // dy/dt = vx * sin(yaw) + vy * cos(yaw)
  d_state(IDX::X) = vel * std::cos(yaw) - vy * std::sin(yaw);
  d_state(IDX::Y) = vel * std::sin(yaw) + vy * std::cos(yaw);
  d_state(IDX::YAW) = yaw_rate;
  d_state(IDX::VX) = acc;
  d_state(IDX::STEER) = steer_rate;
  d_state(IDX::ACCX) = -(acc - acc_des) / acc_time_constant_;

  return d_state;
}

void SimModelDelaySteerAccGeared::updateStateWithGear(
  Eigen::VectorXd & state, const Eigen::VectorXd & prev_state, const uint8_t gear, const double dt)
{
  const auto setStopState = [&]() {
    state(IDX::VX) = 0.0;
    state(IDX::X) = prev_state(IDX::X);
    state(IDX::Y) = prev_state(IDX::Y);
    state(IDX::YAW) = prev_state(IDX::YAW);
    state(IDX::ACCX) = (state(IDX::VX) - prev_state(IDX::VX)) / std::max(dt, 1.0e-5);
  };

  using autoware_vehicle_msgs::msg::GearCommand;
  if (
    gear == GearCommand::DRIVE || gear == GearCommand::DRIVE_2 || gear == GearCommand::DRIVE_3 ||
    gear == GearCommand::DRIVE_4 || gear == GearCommand::DRIVE_5 || gear == GearCommand::DRIVE_6 ||
    gear == GearCommand::DRIVE_7 || gear == GearCommand::DRIVE_8 || gear == GearCommand::DRIVE_9 ||
    gear == GearCommand::DRIVE_10 || gear == GearCommand::DRIVE_11 ||
    gear == GearCommand::DRIVE_12 || gear == GearCommand::DRIVE_13 ||
    gear == GearCommand::DRIVE_14 || gear == GearCommand::DRIVE_15 ||
    gear == GearCommand::DRIVE_16 || gear == GearCommand::DRIVE_17 ||
    gear == GearCommand::DRIVE_18 || gear == GearCommand::LOW || gear == GearCommand::LOW_2) {
    if (state(IDX::VX) < 0.0) {
      setStopState();
    }
  } else if (gear == GearCommand::REVERSE || gear == GearCommand::REVERSE_2) {
    if (state(IDX::VX) > 0.0) {
      setStopState();
    }
  } else {  // including 'gear == GearCommand::PARK'
    setStopState();
  }
}
