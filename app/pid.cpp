/**
 * @author Ryan Cunningham
 * @author Pablo Sanhueza
 * @author Varun Asthana
 * @author Aman Virmani
 *
 * @file pid.cpp
 * @brief Implementation of a PID Controller
 *
 * Copyright [2019] Group12
 */

#include "pid.hpp"

PID::PID(double kp1, double ki1, double kd1, double dt1, double setPoint1) {
  kp = kp1;
  ki = ki1;
  kd = kd1;
  setPoint = setPoint1;
  dt = dt1;
  integral = 0;
  previousError = 0;
}

double PID::getKp() {
  return kp;
}

double PID::getKi() {
  return ki;
}

double PID::getKd() {
  return kd;
}

double PID::getDt() {
  return dt;
}

double PID::getSetPoint() {
  return setPoint;
}

double PID::compute(double processVariable) {
  double u;
  double error = setPoint - processVariable;
  integral += error * dt;
  u = (error * kp) + (kd * (error - previousError) / dt) + (ki * integral);
  previousError = error;
  return u;
}
