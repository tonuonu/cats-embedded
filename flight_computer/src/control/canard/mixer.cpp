/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "control/canard/mixer.hpp"

#include <cmath>

namespace canard {

namespace {

constexpr float kPulseMinUs = 800.0F;
constexpr float kPulseMaxUs = 2200.0F;

float Clamp(float v, float limit) { return std::fmax(-limit, std::fmin(limit, v)); }

/// Largest s in [0, 1] with |roll + s * tilt| <= max_defl (requires |roll| <= max_defl).
float TiltScale(float roll, float tilt, float max_defl) {
  const float full = roll + tilt;
  if (full > max_defl) {
    return (max_defl - roll) / tilt;
  }
  if (full < -max_defl) {
    return (-max_defl - roll) / tilt;
  }
  return 1.0F;
}

}  // namespace

std::uint8_t NumOutputs(Layout layout) {
  switch (layout) {
    case Layout::kRoll2:
      return 2;
    case Layout::kFull4:
      return 4;
    case Layout::kOff:
    default:
      return 0;
  }
}

void Mix(Layout layout, float u_roll, float u_t1, float u_t2, float max_defl_deg, float out_deg[4]) {
  for (int k = 0; k < 4; ++k) {
    out_deg[k] = 0.0F;
  }
  const float max_defl = std::fmax(0.0F, max_defl_deg);
  const float roll = Clamp(u_roll, max_defl);
  if (layout == Layout::kRoll2) {
    out_deg[0] = roll;
    out_deg[1] = roll;
    return;
  }
  if (layout != Layout::kFull4) {
    return;
  }

  // Per-output tilt component: delta_k = roll + tilt[k].
  const float tilt[4] = {u_t2, -u_t1, -u_t2, u_t1};
  float s = 1.0F;
  for (const float t : tilt) {
    s = std::fmin(s, TiltScale(roll, t, max_defl));
  }
  s = std::fmax(0.0F, s);
  for (int k = 0; k < 4; ++k) {
    out_deg[k] = Clamp(roll + s * tilt[k], max_defl);
  }
}

std::uint16_t PulseUs(float defl_deg, std::uint16_t center_us, std::int16_t trim_us, bool reverse, float us_per_deg) {
  const float signed_defl = reverse ? -defl_deg : defl_deg;
  const float us = static_cast<float>(center_us) + static_cast<float>(trim_us) + signed_defl * us_per_deg;
  // Clamp before rounding: the value is then positive, so lround is plain round-to-nearest.
  const float clamped = std::fmax(kPulseMinUs, std::fmin(kPulseMaxUs, us));
  return static_cast<std::uint16_t>(std::lround(clamped));
}

}  // namespace canard
