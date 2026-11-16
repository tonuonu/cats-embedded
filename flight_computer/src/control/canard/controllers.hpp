/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

namespace canard {

/// PI roll-rate damper with back-calculation-free anti-windup (conditional integration).
class RollController {
 public:
  void Reset();

  /// Returns the roll command in degrees, u = -scale * (kp * p + ki * I), clamped to +-limit_deg.
  float Update(float roll_rate_dps, float gain_scale, float kp, float ki, float limit_deg, float dt_s);

 private:
  float m_integral = 0.0F;
  float m_prev_out = 0.0F;
};

/// PD tilt controller on the two transverse axes.
/// d(tilt_t1)/dt = +w_t2 and d(tilt_t2)/dt = -w_t1, hence the cross-coupled rate terms.
/// Stateless, so Update is static.
class TiltController {
 public:
  /// tilt_deg and rate_t_dps hold (t1, t2) and (w_t1, w_t2). out_u is {0, 0} below tilt_v_min.
  static void Update(const float tilt_deg[2], const float rate_t_dps[2], float gain_scale, float kp, float kd,
                     float v_mps, float tilt_v_min, float out_u[2]);
};

}  // namespace canard
