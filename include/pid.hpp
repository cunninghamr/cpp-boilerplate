/**
 * @author Ryan Cunningham
 * @author Pablo Sanhueza
 * @author Varun Asthana
 * @author Aman Virmani
 *
 * @file pid.hpp
 * @brief Implementation of a PID Controller
 *
 * Copyright [2019] Group12
 *
 */

#pragma once

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
  PID(double kp1, double ki1, double kd1, double setPoint1, double dt1);

  /**
   * Compute PID control.
   * https://en.wikipedia.org/wiki/PID_controller#Pseudocode
   * @param processVariable measured position
   * @return control output
   */
  double compute(double processVariable);

  /**
   * returns the private member Kp
   * @param null
   * @return kp double
   */
  double getKp();

  /**
   * returns the private member Ki
   * @param null
   * @return ki double
   */
  double getKi();

  /**
   * returns the private member Kd
   * @param null
   * @return kd double
   */
  double getKd();

  /**
   * returns the private member dt
   * @param null
   * @return dt double
   */
  double getDt();

  /**
   * returns the private member setPoint
   * @param null
   * @return setPoint double
   */
  double getSetPoint();

 private:
  double kp; /**< Proportional gain */
  double ki; /**< Integral gain */
  double kd; /**< Derivative gain*/
  double dt; /**< time step*/
  double setPoint; /** desired position*/
  double previousError;
  double integral;
};

