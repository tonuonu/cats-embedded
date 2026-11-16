/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include <unity.h>

#include "control/canard/status_text.hpp"

using namespace canard;

void setUp() {}
void tearDown() {}

namespace {
LoggedReasonView View() {
  LoggedReasonView v;
  v.started = true;
  v.pad_sample_seen = true;
  v.frame_ok = true;
  v.on_pad = true;
  v.envelope_reason = LatchReason::kNone;
  v.logged_reason = LatchReason::kNone;
  return v;
}
}  // namespace

void Note_NotStarted() {
  LoggedReasonView v = View();
  v.started = false;
  TEST_ASSERT_EQUAL_STRING(" (control not started, nothing recorded)", LoggedReasonNote(v));
}

void Note_SameReason_Empty() {
  LoggedReasonView v = View();
  TEST_ASSERT_EQUAL_STRING("", LoggedReasonNote(v));
  v.envelope_reason = v.logged_reason = LatchReason::kImuFault;
  v.on_pad = false;
  TEST_ASSERT_EQUAL_STRING("", LoggedReasonNote(v));
}

void Note_FrameCheckFailsOnPad() {
  LoggedReasonView v = View();
  v.frame_ok = false;
  v.logged_reason = LatchReason::kImuFault;
  TEST_ASSERT_EQUAL_STRING(" (frame check fails; latches at activation)", LoggedReasonNote(v));
}

void Note_FrameCheckFailedPastPad() {
  // Off the pad (e.g. recovery, or thrust below ctrl_v_enable) no activation may come: no promise of a latch.
  LoggedReasonView v = View();
  v.frame_ok = false;
  v.on_pad = false;
  v.logged_reason = LatchReason::kImuFault;
  TEST_ASSERT_EQUAL_STRING(" (frame check failed)", LoggedReasonNote(v));
}

void Note_NoPadSamples() {
  LoggedReasonView v = View();
  v.pad_sample_seen = false;
  v.frame_ok = false;
  v.on_pad = false;
  v.logged_reason = LatchReason::kImuFault;
  TEST_ASSERT_EQUAL_STRING(" (no pad samples)", LoggedReasonNote(v));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(Note_NotStarted);
  RUN_TEST(Note_SameReason_Empty);
  RUN_TEST(Note_FrameCheckFailsOnPad);
  RUN_TEST(Note_FrameCheckFailedPastPad);
  RUN_TEST(Note_NoPadSamples);
  return UNITY_END();
}
