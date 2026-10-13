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

#ifndef SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_INTERFACE_HPP_
#define SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_INTERFACE_HPP_

#include <Eigen/Core>

#include "autoware_vehicle_msgs/msg/gear_command.hpp"
#include "tier4_vehicle_msgs/msg/actuation_status_stamped.hpp"

#include <optional>

/**
 * @class SimModelInterface
 * @brief simple_planning_simulator vehicle model class to calculate vehicle dynamics
 */
class SimModelInterface
{
protected:
  using ActuationStatusStamped = tier4_vehicle_msgs::msg::ActuationStatusStamped;

  const int dim_x_;        //!< @brief dimension of state x
  const int dim_u_;        //!< @brief dimension of input u
  Eigen::VectorXd state_;  //!< @brief vehicle state vector
  Eigen::VectorXd input_;  //!< @brief vehicle input vector

  //!< @brief gear command defined in autoware_vehicle_msgs/GearCommand
  uint8_t gear_ = autoware_vehicle_msgs::msg::GearCommand::DRIVE;

  //!< @brief lateral velocity from trajectory for 4WS simulation [m/s]
  double lateral_velocity_ = 0.0;

  //!< @brief rear steering angle for 4WS simulation [rad]
  double rear_steering_angle_ = 0.0;

  //!< @brief whether rear steering is defined (from control command)
  bool is_rear_steering_defined_ = false;

  //!< @brief whether crab mode is active (front == rear steering commanded)
  bool is_crab_mode_ = false;

public:
  /**
   * @brief constructor
   * @param [in] dim_x dimension of state x
   * @param [in] dim_u dimension of input u
   */
  SimModelInterface(int dim_x, int dim_u);

  /**
   * @brief destructor
   */
  ~SimModelInterface() = default;

  /**
   * @brief get state vector of model
   * @param [out] state state vector
   */
  void getState(Eigen::VectorXd & state);

  /**
   * @brief get input vector of model
   * @param [out] input input vector
   */
  void getInput(Eigen::VectorXd & input);

  /**
   * @brief set state vector of model
   * @param [in] state state vector
   */
  void setState(const Eigen::VectorXd & state);

  /**
   * @brief set input vector of model
   * @param [in] input input vector
   */
  void setInput(const Eigen::VectorXd & input);

  /**
   * @brief set gear
   * @param [in] gear gear command defined in autoware_vehicle_msgs/GearCommand
   */
  void setGear(const uint8_t gear);

  /**
   * @brief set lateral velocity for 4WS simulation
   * @param [in] lateral_velocity lateral velocity from trajectory [m/s]
   */
  void setLateralVelocity(const double lateral_velocity) { lateral_velocity_ = lateral_velocity; }

  /**
   * @brief get lateral velocity for 4WS simulation
   * @return lateral velocity [m/s]
   */
  double getLateralVelocity() const { return lateral_velocity_; }

  /**
   * @brief set rear steering angle for 4WS simulation
   * @param [in] rear_steering_angle rear steering angle from control command [rad]
   */
  void setRearSteeringAngle(const double rear_steering_angle)
  {
    rear_steering_angle_ = rear_steering_angle;
  }

  /**
   * @brief set whether rear steering is defined
   * @param [in] is_defined whether rear steering is explicitly defined in control command
   */
  void setRearSteeringDefined(const bool is_defined) { is_rear_steering_defined_ = is_defined; }

  /**
   * @brief get rear steering angle for 4WS simulation
   * @return rear steering angle [rad]
   */
  double getRearSteeringAngle() const { return rear_steering_angle_; }

  /**
   * @brief check if rear steering is defined
   * @return true if rear steering is explicitly defined in control command
   */
  bool isRearSteeringDefined() const { return is_rear_steering_defined_; }

  /**
   * @brief set whether crab mode is active (front == rear steering commanded)
   * @param [in] is_crab whether crab mode is active
   */
  void setCrabMode(const bool is_crab) { is_crab_mode_ = is_crab; }

  /**
   * @brief check if crab mode is active
   * @return true if crab mode is active (front == rear steering commanded)
   */
  bool isCrabMode() const { return is_crab_mode_; }

  /**
   * @brief update vehicle states with Runge-Kutta methods
   * @param [in] dt delta time [s]
   * @param [in] input vehicle input
   */
  void updateRungeKutta(const double & dt, const Eigen::VectorXd & input);

  /**
   * @brief update vehicle states with Euler methods
   * @param [in] dt delta time [s]
   * @param [in] input vehicle input
   */
  void updateEuler(const double & dt, const Eigen::VectorXd & input);

  /**
   * @brief update vehicle states
   * @param [in] dt delta time [s]
   */
  virtual void update(const double & dt) = 0;

  /**
   * @brief get vehicle position x
   */
  virtual double getX() = 0;

  /**
   * @brief get vehicle position y
   */
  virtual double getY() = 0;

  /**
   * @brief get vehicle angle yaw
   */
  virtual double getYaw() = 0;

  /**
   * @brief get vehicle velocity vx
   */
  virtual double getVx() = 0;

  /**
   * @brief get vehicle lateral velocity
   */
  virtual double getVy() = 0;

  /**
   * @brief get vehicle longitudinal acceleration
   */
  virtual double getAx() = 0;

  /**
   * @brief get vehicle angular-velocity wz
   */
  virtual double getWz() = 0;

  /**
   * @brief get vehicle steering angle
   */
  virtual double getSteer() = 0;

  /**
   * @brief get vehicle gear
   */
  uint8_t getGear() const;

  /**
   * @brief get state vector dimension
   */
  inline int getDimX() { return dim_x_; }

  /**
   * @brief get input vector dimension
   */
  inline int getDimU() { return dim_u_; }

  /**
   * @brief is publish actuation status enabled
   */
  virtual bool shouldPublishActuationStatus() const { return false; }

  /*
   * @brief get actuation status
   */
  virtual std::optional<ActuationStatusStamped> getActuationStatus() const { return std::nullopt; }

  /**
   * @brief calculate derivative of states with vehicle model
   * @param [in] state current model state
   * @param [in] input input vector to model
   */
  virtual Eigen::VectorXd calcModel(
    const Eigen::VectorXd & state, const Eigen::VectorXd & input) = 0;
};

#endif  // SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_INTERFACE_HPP_
