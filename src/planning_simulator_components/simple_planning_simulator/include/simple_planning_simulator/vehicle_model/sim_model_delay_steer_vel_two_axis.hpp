#ifndef SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_DELAY_STEER_VEL_TWO_AXIS_HPP_
#define SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_DELAY_STEER_VEL_TWO_AXIS_HPP_

#include "simple_planning_simulator/vehicle_model/sim_model_interface.hpp"

#include <Eigen/Core>
#include <Eigen/LU>

#include <deque>
#include <iostream>
#include <queue>
/**
 * @class SimModelDelaySteerVel
 * @brief calculate delay steering dynamics
 */
class SimModelDelaySteerVelTwoAxis : public SimModelInterface
{
public:
  /**
   * @brief constructor
   * @param [in] vx_lim velocity limit [m/s]
   * @param [in] steer_lim steering limit [rad]
   * @param [in] vx_rate_lim acceleration limit [m/ss]
   * @param [in] steer_rate_lim steering angular velocity limit [rad/ss]
   * @param [in] wheelbase vehicle wheelbase length [m]
   * @param [in] dt delta time information to set input buffer for delay
   * @param [in] vx_delay time delay for velocity command [s]
   * @param [in] vx_time_constant time constant for 1D model of velocity dynamics
   * @param [in] steer_delay time delay for steering command [s]
   * @param [in] steer_time_constant time constant for 1D model of steering dynamics
   * @param [in] steer_dead_band dead band for steering angle [rad]
   * @param [in] steer_bias steering bias [rad]
   */
  SimModelDelaySteerVelTwoAxis(
    double vx_lim, double steer_lim, double vx_rate_lim, double steer_rate_lim, double wheelbase_f, double wheelbase_r,
    double dt, double vx_delay, double vx_time_constant, double steer_delay,
    double steer_time_constant, double steer_dead_band, double steer_bias);

  /**
   * @brief destructor
   */
  ~SimModelDelaySteerVelTwoAxis() = default;

private:
  const double MIN_TIME_CONSTANT;  //!< @brief minimum time constant

  enum IDX {
    X = 0,
    Y,
    YAW,
    VX,
    STEER_F,
    STEER_R,
  };
  enum IDX_U {
    VX_DES = 0,
    STEER_F_DES,
    STEER_R_DES,
  };

  const double vx_lim_;          //!< @brief velocity limit
  const double vx_rate_lim_;     //!< @brief acceleration limit
  const double steer_lim_;       //!< @brief steering limit [rad]
  const double steer_rate_lim_;  //!< @brief steering angular velocity limit [rad/s]
  const double wheelbase_f;      //!< @brief vehicle wheelbase length [m]
  const double wheelbase_r;
  double prev_vx_ = 0.0;
  double current_ax_ = 0.0;

  double beta_ = 0.0;
  double kappa_ = 0.0;
  double v_;
  double omega_;

  std::deque<double> vx_input_queue_;     //!< @brief buffer for velocity command
  std::deque<double> steer_f_input_queue_;  //!< @brief buffer for angular velocity command
  std::deque<double> steer_r_input_queue_;
  const double vx_delay_;                 //!< @brief time delay for velocity command [s]
  const double vx_time_constant_;
  //!< @brief time constant for 1D model of velocity dynamics
  const double steer_delay_;  //!< @brief time delay for angular-velocity command [s]
  const double
    steer_time_constant_;  //!< @brief time constant for 1D model of angular-velocity dynamics
  const double steer_dead_band_;  //!< @brief dead band for steering angle [rad]
  const double steer_bias_;       //!< @brief steering angle bias [rad]

  /**
   * @brief set queue buffer for input command
   * @param [in] dt delta time
   */
  void initializeInputQueue(const double & dt);

  /**
   * @brief get vehicle position x
   */
  double getX() override;

  /**
   * @brief get vehicle position y
   */
  double getY() override;

  /**
   * @brief get vehicle angle yaw
   */
  double getYaw() override;

  /**
   * @brief get vehicle longitudinal velocity
   */
  double getVx() override;

  /**
   * @brief get vehicle lateral velocity
   */
  double getVy() override;

  /**
   * @brief get vehicle longitudinal acceleration
   */
  double getAx() override;

  /**
   * @brief get vehicle angular-velocity wz
   */
  double getWz() override;

  /**
   * @brief get vehicle steering angle
   */
  double getSteer() override;
  double getSteerF();
  double getSteerR();

  /**
   * @brief update vehicle states
   * @param [in] dt delta time [s]
   */
  void update(const double & dt) override;

  /**
   * @brief calculate derivative of states with delay steering model
   * @param [in] state current model state
   * @param [in] input input vector to model
   */
  Eigen::VectorXd calcModel(const Eigen::VectorXd & state, const Eigen::VectorXd & input) override;
};

#endif  // SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_DELAY_STEER_VEL_HPP_