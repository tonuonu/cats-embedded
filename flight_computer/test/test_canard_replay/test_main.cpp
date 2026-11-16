/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include <unity.h>

#include <cmath>
#include <cstdint>
#include <cstdio>

#include "control/canard/attitude.hpp"
#include "control/canard/frame.hpp"
#include "fl002_imu.h"

using namespace canard;

namespace {
constexpr float kAccMps2PerLsb = 9.81F / 1024.0F;
constexpr float kGyroDpsPerLsb = 0.07F;
// st002 calibration (Axis 1, Angle -1.0088) via the generated header; firmware uses sign(angle).
constexpr uint8_t kLongAxis = static_cast<uint8_t>(kFl002CalAxis);
constexpr float kLongSign = kFl002CalAngleDeg < 0.0F ? -1.0F : 1.0F;
constexpr float kG = 9.81F;
constexpr float kDt = 0.01F;
}  // namespace

void setUp() {}
void tearDown() {}

// fl002 (J350): 1343 rows at 100 Hz; the log starts 0.755 s before liftoff (76 pad rows).
// Measured: max boost |tilt| 7.56 deg, tilt at apogee 65.3 deg (weathercock/pitch-over at
// -18..-28 dps), liftoff-row tilt 1.22 deg, boost mean nose-axis accel +9.45 g.
void Replay_Fl002_TiltSensible() {
  ControlAttitude att;
  att.Reset();
  float max_boost_tilt = 0.0F;
  float tilt_apogee = 0.0F;
  float max_roll_err = 0.0F;
  float tilt_liftoff = 0.0F;
  double boost_acc_z = 0.0;
  int boost_n = 0;
  for (size_t i = 0; i < kFl002Rows; ++i) {
    const Fl002ImuRow& r = kFl002Imu[i];
    const float acc_board[3] = {r.ax * kAccMps2PerLsb, r.ay * kAccMps2PerLsb,
                                r.az * kAccMps2PerLsb};
    const float gyro_board[3] = {r.gx * kGyroDpsPerLsb, r.gy * kGyroDpsPerLsb,
                                 r.gz * kGyroDpsPerLsb};
    float acc[3] = {0.0F, 0.0F, 0.0F};
    float gyro[3] = {0.0F, 0.0F, 0.0F};
    BoardToBody(acc_board, kLongAxis, kLongSign, acc);
    BoardToBody(gyro_board, kLongAxis, kLongSign, gyro);
    if (i < kFl002LiftoffIndex) {
      att.UpdatePad(gyro, acc, kDt);
    } else {
      att.UpdateFlight(gyro, kDt);
    }
    float bias[3] = {0.0F, 0.0F, 0.0F};
    att.GyroBiasDps(bias);
    // Roll rate is the long-axis gyro column (scaled, signed) minus the pad-learned bias.
    // Checked in flight only: on the pad the bias is updated after the rate is computed.
    if (i >= kFl002LiftoffIndex) {
      max_roll_err = std::fmax(
          max_roll_err,
          std::fabs(att.RollRateDps() - (kLongSign * r.gy * kGyroDpsPerLsb - bias[2])));
    }

    float tilt[2] = {0.0F, 0.0F};
    att.TiltDeg(tilt);
    const float mag = std::sqrt(tilt[0] * tilt[0] + tilt[1] * tilt[1]);
    // Boost = the first 1.86 s after liftoff (COASTING transition).
    if (i >= kFl002LiftoffIndex && r.ts_ms < kFl002Imu[kFl002LiftoffIndex].ts_ms + 1860U) {
      max_boost_tilt = std::fmax(max_boost_tilt, mag);
      boost_acc_z += acc[2];
      ++boost_n;
    }
    tilt_apogee = mag;
    if (i == kFl002LiftoffIndex) {
      tilt_liftoff = mag;
    }
  }
  std::printf("rows=%zu liftoff_idx=%zu max_boost_tilt=%.2f tilt_liftoff=%.2f tilt_apogee=%.2f "
              "boost_acc_g=%.2f roll_err=%g\n",
              kFl002Rows, kFl002LiftoffIndex, static_cast<double>(max_boost_tilt),
              static_cast<double>(tilt_liftoff), static_cast<double>(tilt_apogee),
              boost_acc_z / boost_n / kG, static_cast<double>(max_roll_err));
  TEST_ASSERT_TRUE(max_boost_tilt < 15.0F);
  TEST_ASSERT_TRUE(tilt_liftoff < 5.0F);
  // Pins the long-axis sign: TiltDeg cannot detect an inverted attitude.
  TEST_ASSERT_TRUE(boost_acc_z / boost_n > 5.0 * kG);
  TEST_ASSERT_TRUE(tilt_apogee < 90.0F);
  TEST_ASSERT_TRUE(max_roll_err < 1e-3F);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(Replay_Fl002_TiltSensible);
  return UNITY_END();
}
