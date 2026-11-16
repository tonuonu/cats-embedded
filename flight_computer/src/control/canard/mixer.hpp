/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

#include "control/canard/canard_types.hpp"

namespace canard {

/// Number of active servo outputs for the layout.
std::uint8_t NumOutputs(Layout layout);

/// Maps (roll, tilt) commands to four canard deflections in degrees. Output k is the canard at body azimuth
/// k * 90 deg from +t1 toward +t2. This is the only output clamp: roll has priority, tilt is scaled so that
/// every |out_deg[k]| <= max_defl_deg for any finite inputs. Unused outputs are 0.
void Mix(Layout layout, float u_roll, float u_t1, float u_t2, float max_defl_deg, float out_deg[4]);

/// Servo pulse width in microseconds: center + trim + (reverse ? -1 : 1) * defl * us_per_deg, rounded to
/// nearest and clamped to [800, 2200].
std::uint16_t PulseUs(float defl_deg, std::uint16_t center_us, std::int16_t trim_us, bool reverse,
                      float us_per_deg);

}  // namespace canard
