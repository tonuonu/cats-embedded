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

void TypesSize_LayoutIsOneByte() { TEST_ASSERT_EQUAL(1, sizeof(Layout)); }

int main() {
  UNITY_BEGIN();
  RUN_TEST(TypesDefaults_NeutralOutputs);
  RUN_TEST(TypesSize_LayoutIsOneByte);
  return UNITY_END();
}
