/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include <unity.h>

#include <cmath>
#include <cstdint>
#include <initializer_list>

#include "control/canard/attitude.hpp"
#include "control/canard/frame.hpp"

using namespace canard;

namespace {
constexpr float kDegToRad = 3.14159265358979F / 180.0F;
constexpr float kG = 9.81F;

void Cross(const float a[3], const float b[3], float out[3]) {
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}
}  // namespace

void setUp() {}
void tearDown() {}

void Frame_AllAxesAndSigns_RightHanded() {
  for (uint8_t a = 0; a < 3; ++a) {
    for (const float s : {1.0F, -1.0F}) {
      const float ex[3] = {1.0F, 0.0F, 0.0F};
      const float ey[3] = {0.0F, 1.0F, 0.0F};
      float exey[3] = {0.0F, 0.0F, 0.0F};
      Cross(ex, ey, exey);
      float bx[3] = {0.0F, 0.0F, 0.0F};
      float by[3] = {0.0F, 0.0F, 0.0F};
      float bxy[3] = {0.0F, 0.0F, 0.0F};
      BoardToBody(ex, a, s, bx);
      BoardToBody(ey, a, s, by);
      BoardToBody(exey, a, s, bxy);
      float c[3] = {0.0F, 0.0F, 0.0F};
      Cross(bx, by, c);
      for (int i = 0; i < 3; ++i) {
        TEST_ASSERT_FLOAT_WITHIN(1e-6F, bxy[i], c[i]);
      }
    }
  }
}

void Attitude_PadLevel_ConvergesToRailTilt() {
  ControlAttitude att;
  att.Reset();
  const float gyro[3] = {0.0F, 0.0F, 0.0F};
  const float acc[3] = {kG * std::sin(5.0F * kDegToRad), 0.0F, kG * std::cos(5.0F * kDegToRad)};
  for (int i = 0; i < 2000; ++i) {
    att.UpdatePad(gyro, acc, 0.01F);
  }
  float tilt[2] = {0.0F, 0.0F};
  att.TiltDeg(tilt);
  TEST_ASSERT_FLOAT_WITHIN(0.2F, -5.0F, tilt[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.2F, 0.0F, tilt[1]);
}

void Attitude_PadBias_Learned() {
  ControlAttitude att;
  att.Reset();
  const float gyro[3] = {0.3F, -0.2F, 0.1F};
  const float acc[3] = {0.0F, 0.0F, kG};
  for (int i = 0; i < 6000; ++i) {
    att.UpdatePad(gyro, acc, 0.01F);
  }
  float bias[3] = {0.0F, 0.0F, 0.0F};
  att.GyroBiasDps(bias);
  TEST_ASSERT_FLOAT_WITHIN(0.02F, 0.3F, bias[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.02F, -0.2F, bias[1]);
  TEST_ASSERT_FLOAT_WITHIN(0.02F, 0.1F, bias[2]);
}

void Attitude_Flight_IntegratesRoll() {
  ControlAttitude att;
  att.Reset();
  const float gyro[3] = {0.0F, 0.0F, 90.0F};
  for (int i = 0; i < 100; ++i) {
    att.UpdateFlight(gyro, 0.01F);
  }
  TEST_ASSERT_EQUAL_FLOAT(90.0F, att.RollRateDps());
  float tilt[2] = {0.0F, 0.0F};
  att.TiltDeg(tilt);
  TEST_ASSERT_FLOAT_WITHIN(0.01F, 0.0F, tilt[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.01F, 0.0F, tilt[1]);
}

void Attitude_Flight_IntegratesTilt() {
  ControlAttitude att;
  att.Reset();
  const float gyro[3] = {0.0F, 10.0F, 0.0F};
  for (int i = 0; i < 50; ++i) {
    att.UpdateFlight(gyro, 0.01F);
  }
  float tilt[2] = {0.0F, 0.0F};
  att.TiltDeg(tilt);
  TEST_ASSERT_FLOAT_WITHIN(0.05F, 5.0F, tilt[0]);
  float rates[2] = {0.0F, 0.0F};
  att.TransverseRatesDps(rates);
  TEST_ASSERT_EQUAL_FLOAT(0.0F, rates[0]);
  TEST_ASSERT_EQUAL_FLOAT(10.0F, rates[1]);
}

void Attitude_PadSkipsLevellingUnderAccel() {
  ControlAttitude att;
  att.Reset();
  const float gyro[3] = {0.0F, 0.0F, 0.0F};
  const float acc[3] = {10.0F, 0.0F, 11.2F};  // |acc| = 15
  for (int i = 0; i < 500; ++i) {
    att.UpdatePad(gyro, acc, 0.01F);
  }
  float tilt[2] = {1.0F, 1.0F};
  att.TiltDeg(tilt);
  TEST_ASSERT_FLOAT_WITHIN(1e-4F, 0.0F, tilt[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4F, 0.0F, tilt[1]);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(Frame_AllAxesAndSigns_RightHanded);
  RUN_TEST(Attitude_PadLevel_ConvergesToRailTilt);
  RUN_TEST(Attitude_PadBias_Learned);
  RUN_TEST(Attitude_Flight_IntegratesRoll);
  RUN_TEST(Attitude_Flight_IntegratesTilt);
  RUN_TEST(Attitude_PadSkipsLevellingUnderAccel);
  return UNITY_END();
}
