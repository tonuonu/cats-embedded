/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include <unity.h>

#include "control/canard/canard_types.hpp"

using namespace control::canard;

void setUp() {}
void tearDown() {}

void TypesDefaults_NeutralOutputs() {
  const ControlOutputs out{};
  for (int i = 0; i < 4; ++i) {
    TEST_ASSERT_EQUAL_FLOAT(0.0F, out.defl_deg[i]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, out.cmd_deg[i]);
  }
  TEST_ASSERT_TRUE(out.state == EnvState::kDisabled);
  TEST_ASSERT_TRUE(out.reason == LatchReason::kNone);
}

void TypesDefaults_PlainDeclarationIsNeutral() {
  ControlOutputs o;
  ControlInputs in;
  CanardParams p;
  for (int i = 0; i < 4; ++i) {
    TEST_ASSERT_EQUAL_FLOAT(0.0F, o.defl_deg[i]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, o.cmd_deg[i]);
  }
  TEST_ASSERT_EQUAL_UINT8(0, o.n_outputs);
  TEST_ASSERT_TRUE(o.state == EnvState::kDisabled);
  TEST_ASSERT_TRUE(o.reason == LatchReason::kNone);
  TEST_ASSERT_EQUAL_FLOAT(0.0F, o.roll_rate_dps);
  TEST_ASSERT_EQUAL_FLOAT(0.0F, o.tilt_deg[1]);
  TEST_ASSERT_EQUAL_FLOAT(0.0F, in.gyro_board_dps[2]);
  TEST_ASSERT_EQUAL_FLOAT(0.0F, in.acc_board_mps2[0]);
  TEST_ASSERT_EQUAL_FLOAT(1.0F, in.long_sign);
  TEST_ASSERT_TRUE(in.phase == Phase::kCalibrating);
  TEST_ASSERT_FALSE(in.imu_faulty);
  TEST_ASSERT_FALSE(in.sim_running);
  TEST_ASSERT_TRUE(p.layout == Layout::kOff);
  TEST_ASSERT_TRUE(p.mode == FlightMode::kShadow);
  TEST_ASSERT_EQUAL_FLOAT(0.0F, p.max_defl_deg);
  TEST_ASSERT_EQUAL_FLOAT(0.0F, p.slew_dps);
  TEST_ASSERT_EQUAL_UINT8(0, p.step_count);
  TEST_ASSERT_EQUAL_FLOAT(180.0F, p.authority_rate_dps);
}

void TypesSize_LayoutIsOneByte() { TEST_ASSERT_EQUAL(1, sizeof(Layout)); }

int main() {
  UNITY_BEGIN();
  RUN_TEST(TypesDefaults_NeutralOutputs);
  RUN_TEST(TypesDefaults_PlainDeclarationIsNeutral);
  RUN_TEST(TypesSize_LayoutIsOneByte);
  return UNITY_END();
}
