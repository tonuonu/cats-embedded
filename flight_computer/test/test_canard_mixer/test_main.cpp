/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include <unity.h>

#include <cmath>

#include "control/canard/mixer.hpp"

using namespace canard;

void setUp() {}
void tearDown() {}

void Mixer_NumOutputs() {
  TEST_ASSERT_EQUAL_UINT8(0, NumOutputs(Layout::kOff));
  TEST_ASSERT_EQUAL_UINT8(2, NumOutputs(Layout::kRoll2));
  TEST_ASSERT_EQUAL_UINT8(4, NumOutputs(Layout::kFull4));
}

void Mixer_Roll2_BothOutputsRoll() {
  float out[4] = {9, 9, 9, 9};
  Mix(Layout::kRoll2, 3.0F, 1.0F, 1.0F, 8.0F, out);
  TEST_ASSERT_FLOAT_WITHIN(1e-6F, 3.0F, out[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-6F, 3.0F, out[1]);
  TEST_ASSERT_FLOAT_WITHIN(1e-6F, 0.0F, out[2]);
  TEST_ASSERT_FLOAT_WITHIN(1e-6F, 0.0F, out[3]);
}

void Mixer_Roll2_ClampsRoll() {
  float out[4];
  Mix(Layout::kRoll2, -20.0F, 0.0F, 0.0F, 8.0F, out);
  TEST_ASSERT_FLOAT_WITHIN(1e-6F, -8.0F, out[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-6F, -8.0F, out[1]);
}

void Mixer_Off_AllZero() {
  float out[4] = {9, 9, 9, 9};
  Mix(Layout::kOff, 3.0F, 1.0F, 1.0F, 8.0F, out);
  for (int i = 0; i < 4; ++i) {
    TEST_ASSERT_FLOAT_WITHIN(1e-6F, 0.0F, out[i]);
  }
}

void Mixer_Full4_PureTilt() {
  float out[4];
  Mix(Layout::kFull4, 0.0F, 2.0F, 0.0F, 8.0F, out);
  TEST_ASSERT_FLOAT_WITHIN(1e-6F, 0.0F, out[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-6F, -2.0F, out[1]);
  TEST_ASSERT_FLOAT_WITHIN(1e-6F, 0.0F, out[2]);
  TEST_ASSERT_FLOAT_WITHIN(1e-6F, 2.0F, out[3]);
}

void Mixer_Full4_PureTilt_T2() {
  float out[4];
  Mix(Layout::kFull4, 0.0F, 0.0F, 2.0F, 8.0F, out);
  TEST_ASSERT_FLOAT_WITHIN(1e-6F, 2.0F, out[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-6F, 0.0F, out[1]);
  TEST_ASSERT_FLOAT_WITHIN(1e-6F, -2.0F, out[2]);
  TEST_ASSERT_FLOAT_WITHIN(1e-6F, 0.0F, out[3]);
}

void Mixer_RollPriority() {
  float out[4];
  Mix(Layout::kFull4, 6.0F, 6.0F, 0.0F, 8.0F, out);
  float mean = 0.0F;
  for (int i = 0; i < 4; ++i) {
    TEST_ASSERT_TRUE(std::fabs(out[i]) <= 8.0F + 1e-5F);
    mean += out[i] * 0.25F;
  }
  TEST_ASSERT_FLOAT_WITHIN(1e-5F, 6.0F, mean);
  // Tilt scaled to 2: delta1 = 6 - 2, delta3 = 6 + 2.
  TEST_ASSERT_FLOAT_WITHIN(1e-5F, 4.0F, out[1]);
  TEST_ASSERT_FLOAT_WITHIN(1e-5F, 8.0F, out[3]);
}

void Mixer_HugeTilt_BoundedAndRollPreserved() {
  float out[4];
  Mix(Layout::kFull4, 5.0F, 100.0F, -100.0F, 8.0F, out);
  float mean = 0.0F;
  for (int i = 0; i < 4; ++i) {
    TEST_ASSERT_TRUE(std::fabs(out[i]) <= 8.0F + 1e-5F);
    mean += out[i] * 0.25F;
  }
  TEST_ASSERT_FLOAT_WITHIN(1e-5F, 5.0F, mean);
}

void Mixer_HugeRollAndTilt_Bounded() {
  float out[4];
  Mix(Layout::kFull4, 100.0F, 100.0F, 100.0F, 8.0F, out);
  for (int i = 0; i < 4; ++i) {
    TEST_ASSERT_TRUE(std::fabs(out[i]) <= 8.0F + 1e-5F);
  }
}

void Mixer_ZeroMaxDefl_AllTrim() {
  float out[4];
  Mix(Layout::kFull4, 6.0F, 100.0F, -50.0F, 0.0F, out);
  for (int i = 0; i < 4; ++i) {
    TEST_ASSERT_FLOAT_WITHIN(1e-6F, 0.0F, out[i]);
  }
  Mix(Layout::kRoll2, 6.0F, 1.0F, 1.0F, 0.0F, out);
  for (int i = 0; i < 4; ++i) {
    TEST_ASSERT_FLOAT_WITHIN(1e-6F, 0.0F, out[i]);
  }
}

void Pulse_ReverseAndTrim() {
  TEST_ASSERT_EQUAL_UINT16(1490, PulseUs(2.0F, 1520, -10, true, 10.0F));
}

void Pulse_Clamped() {
  TEST_ASSERT_EQUAL_UINT16(2200, PulseUs(200.0F, 1500, 0, false, 10.0F));
  TEST_ASSERT_EQUAL_UINT16(800, PulseUs(-200.0F, 1500, 0, false, 10.0F));
}

void Pulse_RoundsToNearest_NegativeOffset() {
  TEST_ASSERT_EQUAL_UINT16(1496, PulseUs(-0.36F, 1500, 0, false, 10.0F));  // -3.6 -> -4
  TEST_ASSERT_EQUAL_UINT16(1504, PulseUs(0.36F, 1500, 0, false, 10.0F));
  TEST_ASSERT_EQUAL_UINT16(1500, PulseUs(0.0F, 1500, 0, false, 10.0F));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(Mixer_NumOutputs);
  RUN_TEST(Mixer_Roll2_BothOutputsRoll);
  RUN_TEST(Mixer_Roll2_ClampsRoll);
  RUN_TEST(Mixer_Off_AllZero);
  RUN_TEST(Mixer_Full4_PureTilt);
  RUN_TEST(Mixer_Full4_PureTilt_T2);
  RUN_TEST(Mixer_RollPriority);
  RUN_TEST(Mixer_HugeTilt_BoundedAndRollPreserved);
  RUN_TEST(Mixer_HugeRollAndTilt_Bounded);
  RUN_TEST(Mixer_ZeroMaxDefl_AllTrim);
  RUN_TEST(Pulse_ReverseAndTrim);
  RUN_TEST(Pulse_Clamped);
  RUN_TEST(Pulse_RoundsToNearest_NegativeOffset);
  return UNITY_END();
}
