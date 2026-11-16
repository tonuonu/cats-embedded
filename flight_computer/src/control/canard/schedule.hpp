/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

namespace canard {

/// ISA air density in kg/m^3 at height above ground level (clamped to [0, 11000] m).
float IsaDensity(float altitude_agl_m);

/// Dynamic pressure 0.5 * rho * v^2 in Pa at the given speed and height above ground.
float DynamicPressure(float v_mps, float altitude_agl_m);

/// Gain multiplier that normalises loop gain to the reference dynamic pressure.
/// Equals q(v_ref, 0) / max(q(v, h), q(v_enable, h), 1 Pa).
float GainScale(float v_mps, float altitude_agl_m, float v_ref, float v_enable);

}  // namespace canard
