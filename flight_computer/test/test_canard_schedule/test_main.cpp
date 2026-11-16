/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include <unity.h>

#include <cmath>

#include "control/canard/schedule.hpp"

using namespace canard;

void setUp() {}
void tearDown() {}

void Schedule_AtVref_IsOne() {
  TEST_ASSERT_FLOAT_WITHIN(1e-4F, 1.0F, GainScale(100.0F, 0.0F, 100.0F, 30.0F));
}

void Schedule_HalfSpeed_IsFour() {
  TEST_ASSERT_FLOAT_WITHIN(1e-3F, 4.0F, GainScale(50.0F, 0.0F, 100.0F, 30.0F));
}

void Schedule_BelowEnable_ClampsAtEnable() {
  TEST_ASSERT_FLOAT_WITHIN(1e-6F, GainScale(30.0F, 0.0F, 100.0F, 30.0F),
                           GainScale(10.0F, 0.0F, 100.0F, 30.0F));
}

void Schedule_VrefBelowVenable_Finite() {
  const float g = GainScale(0.0F, 0.0F, 20.0F, 30.0F);
  TEST_ASSERT_TRUE(std::isfinite(g));
  TEST_ASSERT_TRUE(g > 0.0F);
}

void Isa_1000m() { TEST_ASSERT_FLOAT_WITHIN(0.002F, 1.112F, IsaDensity(1000.0F)); }

int main() {
  UNITY_BEGIN();
  RUN_TEST(Schedule_AtVref_IsOne);
  RUN_TEST(Schedule_HalfSpeed_IsFour);
  RUN_TEST(Schedule_BelowEnable_ClampsAtEnable);
  RUN_TEST(Schedule_VrefBelowVenable_Finite);
  RUN_TEST(Isa_1000m);
  return UNITY_END();
}
