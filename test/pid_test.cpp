/**
 * @author Ryan Cunningham
 * @author Pablo Sanhueza
 * @author Varun Asthana
 * @author Aman Virmani
 *
 * @file pid_test.cpp
 * @brief Unit Tests for PID Controller
 *
 * Copyright [2019] Group12
 */

#include <gtest/gtest.h>
#include "pid.hpp"

/**
 * Tests constructor for PID controller.
 */
TEST(PIDTest, testPIDConstructor) {
  PID pid = PID(1, 2, 3, 4, 5);

  EXPECT_DOUBLE_EQ(1, pid.getKp());
  EXPECT_DOUBLE_EQ(2, pid.getKi());
  EXPECT_DOUBLE_EQ(3, pid.getKd());
  EXPECT_DOUBLE_EQ(4, pid.getDt());
  EXPECT_DOUBLE_EQ(5, pid.getSetPoint());
}

/**
 * Tests the proportional control part of the PID controller independently.
 */
TEST(PIDTest, testPIDComputeProportional) {
  PID pid = PID(2, 0, 0, 1, 0);

  pid.compute(1);
  pid.compute(2);
  double control = pid.compute(3);

  EXPECT_DOUBLE_EQ(-6, control);
}

/**
 * Tests the integral control part of the PID controller independently.
 */
TEST(PIDTest, testPIDComputeIntegral) {
  PID pid = PID(0, 2, 0, 1, 0);

  pid.compute(1);
  pid.compute(2);
  double control = pid.compute(3);

  EXPECT_DOUBLE_EQ(-12, control);
}

/**
 * Tests the derivative control part of the PID controller independently.
 */
TEST(PIDTest, testPIDComputeDerivative) {
  PID pid = PID(0, 0, 2, 1, 0);

  pid.compute(1);
  pid.compute(2);
  double control = pid.compute(3);

  EXPECT_DOUBLE_EQ(-2, control);
}
