/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include <unity.h>

#include "control/canard/controllers.hpp"

using namespace canard;

void setUp() {}
void tearDown() {}

void Roll_PositiveRate_NegativeCommand() {
  RollController c;
  c.Reset();
  const float u = c.Update(50.0F, 1.0F, 0.05F, 0.0F, 8.0F, 0.01F);
  TEST_ASSERT_FLOAT_WITHIN(1e-5F, -2.5F, u);
}

void Roll_Integrator_AntiWindup() {
  RollController c;
  c.Reset();
  float u = 0.0F;
  for (int i = 0; i < 500; ++i) {
    u = c.Update(100.0F, 1.0F, 0.0F, 0.1F, 8.0F, 0.01F);
    TEST_ASSERT_TRUE(u >= -8.0F);
  }
  TEST_ASSERT_FLOAT_WITHIN(1e-3F, -8.0F, u);
  float prev = u;
  for (int i = 0; i < 20; ++i) {
    u = c.Update(-10.0F, 1.0F, 0.0F, 0.1F, 8.0F, 0.01F);
    TEST_ASSERT_TRUE(u >= prev);
    prev = u;
  }
  TEST_ASSERT_TRUE(u > -8.0F + 1e-3F);
}

void Roll_ZeroKi_NoClampCrash() {
  RollController c;
  c.Reset();
  const float u = c.Update(1000.0F, 1.0F, 0.05F, 0.0F, 8.0F, 0.01F);
  TEST_ASSERT_FLOAT_WITHIN(1e-5F, -8.0F, u);
}

void Tilt_ZeroBelowVmin() {
  TiltController c;
  const float tilt[2] = {5.0F, 5.0F};
  const float rate[2] = {10.0F, 10.0F};
  float u[2] = {9.0F, 9.0F};
  c.Update(tilt, rate, 1.0F, 1.0F, 0.1F, 69.0F, 70.0F, u);
  TEST_ASSERT_EQUAL_FLOAT(0.0F, u[0]);
  TEST_ASSERT_EQUAL_FLOAT(0.0F, u[1]);
}

void Tilt_PushesNoseBack() {
  TiltController c;
  const float tilt[2] = {5.0F, 0.0F};
  const float rate[2] = {0.0F, 0.0F};
  float u[2] = {9.0F, 9.0F};
  c.Update(tilt, rate, 1.0F, 1.0F, 0.0F, 100.0F, 70.0F, u);
  TEST_ASSERT_TRUE(u[0] < 0.0F);
  TEST_ASSERT_EQUAL_FLOAT(0.0F, u[1]);
}

void Tilt_RateDamping_Signs() {
  TiltController c;
  const float tilt[2] = {0.0F, 0.0F};
  const float r1[2] = {0.0F, 10.0F};
  const float r2[2] = {10.0F, 0.0F};
  float u[2] = {};
  c.Update(tilt, r1, 1.0F, 0.0F, 0.1F, 100.0F, 70.0F, u);
  TEST_ASSERT_FLOAT_WITHIN(1e-5F, -1.0F, u[0]);
  c.Update(tilt, r2, 1.0F, 0.0F, 0.1F, 100.0F, 70.0F, u);
  TEST_ASSERT_FLOAT_WITHIN(1e-5F, 1.0F, u[1]);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(Roll_PositiveRate_NegativeCommand);
  RUN_TEST(Roll_Integrator_AntiWindup);
  RUN_TEST(Roll_ZeroKi_NoClampCrash);
  RUN_TEST(Tilt_ZeroBelowVmin);
  RUN_TEST(Tilt_PushesNoseBack);
  RUN_TEST(Tilt_RateDamping_Signs);
  return UNITY_END();
}
