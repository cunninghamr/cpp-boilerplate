/**
 * @author Ryan Cunningham
 * @author Pablo Sanhueza
 * @file pid.hpp
 * @brief Implementation of a PID Controller
 * @copyright 2019
 *
 * PID Controller
 * https://en.wikipedia.org/wiki/PID_controller
 */

#ifndef INCLUDE_PID_HPP_
#define INCLUDE_PID_HPP_

class PID {
 public:
  /**
   * Create and initialize PID controller.
   * @param kp proportional gain
   * @param ki integral gain
   * @param kd derivative gain
   * @param setpoint desired position
   * @param dt time step
   */
  PID(double kp, double ki, double kd, double setpoint, double dt);

  /**
   * Compute PID control.
   * https://en.wikipedia.org/wiki/PID_controller#Pseudocode
   * @param processVariable measured position
   * @return control output
   */
  double compute(double processVariable);

  double getKp();
  double getKi();
  double getKd();
  double getDt();
  double getSetpoint();
  void setSetpoint();

 private:
  double kp;
  double ki;
  double kd;
  double dt;
  double setpoint;
};

#endif /* INCLUDE_PID_HPP_ */
