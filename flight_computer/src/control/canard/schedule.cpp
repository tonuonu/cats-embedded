/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "control/canard/schedule.hpp"

#include <cmath>

namespace canard {

namespace {
constexpr float kSeaLevelDensity = 1.225F;
constexpr float kLapseCoeffPerM = 2.2558e-5F;
constexpr float kLapseExponent = 4.2559F;
constexpr float kMaxAltitudeM = 11000.0F;
constexpr float kMinPressurePa = 1.0F;
}  // namespace

float IsaDensity(float altitude_agl_m) {
  const float h = std::fmax(0.0F, std::fmin(kMaxAltitudeM, altitude_agl_m));
  return kSeaLevelDensity * std::pow(1.0F - kLapseCoeffPerM * h, kLapseExponent);
}

float DynamicPressure(float v_mps, float altitude_agl_m) {
  return 0.5F * IsaDensity(altitude_agl_m) * v_mps * v_mps;
}

float GainScale(float v_mps, float altitude_agl_m, float v_ref, float v_enable) {
  const float q_ref = DynamicPressure(v_ref, 0.0F);
  const float q_now = DynamicPressure(v_mps, altitude_agl_m);
  const float q_enable = DynamicPressure(v_enable, altitude_agl_m);
  const float denom = std::fmax(kMinPressurePa, std::fmax(q_now, q_enable));
  return q_ref / denom;
}

}  // namespace canard
