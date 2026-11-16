/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include <unity.h>

#include <initializer_list>

#include "control/canard/test_args.hpp"

using namespace canard;

void setUp() {}
void tearDown() {}

void LiveArgs_DefaultDuration() {
  const LiveTestArgs a = ParseLiveTestArgs("100");
  TEST_ASSERT_TRUE(a.ok);
  TEST_ASSERT_EQUAL_FLOAT(100.0F, a.speed_mps);
  TEST_ASSERT_EQUAL_FLOAT(kLiveTestDefaultS, a.duration_s);
  TEST_ASSERT_FALSE(a.capped);
}

void LiveArgs_DurationKept() {
  const LiveTestArgs a = ParseLiveTestArgs("100 5");
  TEST_ASSERT_TRUE(a.ok);
  TEST_ASSERT_EQUAL_FLOAT(5.0F, a.duration_s);
  TEST_ASSERT_FALSE(a.capped);
  const LiveTestArgs b = ParseLiveTestArgs("100 120");
  TEST_ASSERT_TRUE(b.ok);
  TEST_ASSERT_EQUAL_FLOAT(120.0F, b.duration_s);
  TEST_ASSERT_FALSE(b.capped);
}

void LiveArgs_DurationCappedAt120() {
  const LiveTestArgs a = ParseLiveTestArgs("100 500");
  TEST_ASSERT_TRUE(a.ok);
  TEST_ASSERT_EQUAL_FLOAT(kMaxTestS, a.duration_s);
  TEST_ASSERT_EQUAL_FLOAT(120.0F, a.duration_s);
  TEST_ASSERT_TRUE(a.capped);
}

void LiveArgs_BadDurationRejected() {
  for (const char* text : {"100 inf", "100 nan", "100 0", "100 -1", "100 x", "100 5x", "100 -inf"}) {
    TEST_ASSERT_FALSE_MESSAGE(ParseLiveTestArgs(text).ok, text);
  }
}

void LiveArgs_OverflowIsOutOfRange() {
  // strtof overflow (ERANGE) is reported as out of range, not as a usage error and not as the 120 s cap.
  const LiveTestArgs d = ParseLiveTestArgs("100 1e39");
  TEST_ASSERT_FALSE(d.ok);
  TEST_ASSERT_EQUAL(static_cast<int>(LiveTestArgs::Error::kDurationRange), static_cast<int>(d.error));
  const LiveTestArgs v = ParseLiveTestArgs("1e39 5");
  TEST_ASSERT_FALSE(v.ok);
  TEST_ASSERT_EQUAL(static_cast<int>(LiveTestArgs::Error::kSpeedRange), static_cast<int>(v.error));
  // Typed "inf" is a usage error; a valid command has no error.
  TEST_ASSERT_EQUAL(static_cast<int>(LiveTestArgs::Error::kUsage), static_cast<int>(ParseLiveTestArgs("100 inf").error));
  TEST_ASSERT_EQUAL(static_cast<int>(LiveTestArgs::Error::kNone), static_cast<int>(ParseLiveTestArgs("100 500").error));
}

void LiveArgs_GarbageBeforeRange() {
  // A token with trailing garbage is a usage error even when its number would overflow.
  using E = LiveTestArgs::Error;
  for (const char* text : {"100 1e39x", "1e39x 5", "1e39x", "-1e39x"}) {
    TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(E::kUsage), static_cast<int>(ParseLiveTestArgs(text).error), text);
  }
}

void LiveArgs_NegativeOverflowIsOutOfRange() {
  using E = LiveTestArgs::Error;
  TEST_ASSERT_EQUAL(static_cast<int>(E::kSpeedRange), static_cast<int>(ParseLiveTestArgs("-1e39").error));
  TEST_ASSERT_EQUAL(static_cast<int>(E::kSpeedRange), static_cast<int>(ParseLiveTestArgs("-1e39 5").error));
  TEST_ASSERT_EQUAL(static_cast<int>(E::kDurationRange), static_cast<int>(ParseLiveTestArgs("100 -1e39").error));
}

void LiveArgs_UnderflowReadsAsZero() {
  // Documented: an underflow to 0 is a usage error (<= 0), not out of range; errno is not used.
  using E = LiveTestArgs::Error;
  TEST_ASSERT_EQUAL(static_cast<int>(E::kUsage), static_cast<int>(ParseLiveTestArgs("100 1e-50").error));
  TEST_ASSERT_EQUAL(static_cast<int>(E::kUsage), static_cast<int>(ParseLiveTestArgs("1e-50").error));
}

void LiveArgs_BadSpeedRejected() {
  for (const char* text : {"", "x", "inf", "nan", "0", "-1", "-inf", "inf 5", "nan 5", "0 5"}) {
    TEST_ASSERT_FALSE_MESSAGE(ParseLiveTestArgs(text).ok, text);
  }
  TEST_ASSERT_FALSE(ParseLiveTestArgs(nullptr).ok);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(LiveArgs_DefaultDuration);
  RUN_TEST(LiveArgs_DurationKept);
  RUN_TEST(LiveArgs_DurationCappedAt120);
  RUN_TEST(LiveArgs_BadDurationRejected);
  RUN_TEST(LiveArgs_OverflowIsOutOfRange);
  RUN_TEST(LiveArgs_GarbageBeforeRange);
  RUN_TEST(LiveArgs_NegativeOverflowIsOutOfRange);
  RUN_TEST(LiveArgs_UnderflowReadsAsZero);
  RUN_TEST(LiveArgs_BadSpeedRejected);
  return UNITY_END();
}
