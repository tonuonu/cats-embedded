/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include <unity.h>

#include <cmath>

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
  const float tilt[2] = {5.0F, 5.0F};
  const float rate[2] = {10.0F, 10.0F};
  float u[2] = {9.0F, 9.0F};
  TiltController::Update(tilt, rate, 1.0F, 1.0F, 0.1F, 69.0F, 70.0F, u);
  TEST_ASSERT_EQUAL_FLOAT(0.0F, u[0]);
  TEST_ASSERT_EQUAL_FLOAT(0.0F, u[1]);
}

void Tilt_PushesNoseBack() {
  const float tilt[2] = {5.0F, 0.0F};
  const float rate[2] = {0.0F, 0.0F};
  float u[2] = {9.0F, 9.0F};
  TiltController::Update(tilt, rate, 1.0F, 1.0F, 0.0F, 100.0F, 70.0F, u);
  TEST_ASSERT_TRUE(u[0] < 0.0F);
  TEST_ASSERT_EQUAL_FLOAT(0.0F, u[1]);
}

void Tilt_RateDamping_Signs() {
  const float tilt[2] = {0.0F, 0.0F};
  const float r1[2] = {0.0F, 10.0F};
  const float r2[2] = {10.0F, 0.0F};
  float u[2] = {};
  TiltController::Update(tilt, r1, 1.0F, 0.0F, 0.1F, 100.0F, 70.0F, u);
  TEST_ASSERT_FLOAT_WITHIN(1e-5F, -1.0F, u[0]);
  TiltController::Update(tilt, r2, 1.0F, 0.0F, 0.1F, 100.0F, 70.0F, u);
  TEST_ASSERT_FLOAT_WITHIN(1e-5F, 1.0F, u[1]);
}

void Roll_Freeze_StopsIntegrationWhileSaturated() {
  RollController c;
  c.Reset();
  // kp alone saturates the output; the integral must not wind up during the hold.
  for (int i = 0; i < 500; ++i) {
    const float u = c.Update(100.0F, 1.0F, 1.0F, 0.01F, 8.0F, 0.01F);
    TEST_ASSERT_FLOAT_WITHIN(1e-4F, -8.0F, u);
  }
  // Rate drops to zero: output is -ki*I, which stays tiny only if I did not grow (unfrozen: -5).
  const float u = c.Update(0.0F, 1.0F, 1.0F, 0.01F, 8.0F, 0.01F);
  TEST_ASSERT_TRUE(std::fabs(u) < 0.1F);
}

void Roll_GainScale_Doubles() {
  RollController a;
  RollController b;
  a.Reset();
  b.Reset();
  const float u1 = a.Update(10.0F, 1.0F, 0.05F, 0.0F, 8.0F, 0.01F);
  const float u2 = b.Update(10.0F, 2.0F, 0.05F, 0.0F, 8.0F, 0.01F);
  TEST_ASSERT_FLOAT_WITHIN(1e-5F, 2.0F * u1, u2);
}

void Tilt_PushesNoseBack_T2() {
  const float tilt[2] = {0.0F, 5.0F};
  const float rate[2] = {0.0F, 0.0F};
  float u[2] = {9.0F, 9.0F};
  TiltController::Update(tilt, rate, 1.0F, 1.0F, 0.0F, 100.0F, 70.0F, u);
  TEST_ASSERT_EQUAL_FLOAT(0.0F, u[0]);
  TEST_ASSERT_TRUE(u[1] < 0.0F);
}

void Tilt_GainScale_Doubles() {
  const float tilt[2] = {3.0F, -2.0F};
  const float rate[2] = {4.0F, 5.0F};
  float a[2] = {};
  float b[2] = {};
  TiltController::Update(tilt, rate, 1.0F, 1.0F, 0.1F, 100.0F, 70.0F, a);
  TiltController::Update(tilt, rate, 2.0F, 1.0F, 0.1F, 100.0F, 70.0F, b);
  TEST_ASSERT_FLOAT_WITHIN(1e-5F, 2.0F * a[0], b[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-5F, 2.0F * a[1], b[1]);
}

void Tilt_AtVmin_Active() {
  const float tilt[2] = {5.0F, 0.0F};
  const float rate[2] = {0.0F, 0.0F};
  float u[2] = {};
  TiltController::Update(tilt, rate, 1.0F, 1.0F, 0.0F, 70.0F, 70.0F, u);
  TEST_ASSERT_TRUE(u[0] < 0.0F);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(Roll_PositiveRate_NegativeCommand);
  RUN_TEST(Roll_Integrator_AntiWindup);
  RUN_TEST(Roll_ZeroKi_NoClampCrash);
  RUN_TEST(Tilt_ZeroBelowVmin);
  RUN_TEST(Tilt_PushesNoseBack);
  RUN_TEST(Tilt_RateDamping_Signs);
  RUN_TEST(Roll_Freeze_StopsIntegrationWhileSaturated);
  RUN_TEST(Roll_GainScale_Doubles);
  RUN_TEST(Tilt_PushesNoseBack_T2);
  RUN_TEST(Tilt_GainScale_Doubles);
  RUN_TEST(Tilt_AtVmin_Active);
  return UNITY_END();
}
