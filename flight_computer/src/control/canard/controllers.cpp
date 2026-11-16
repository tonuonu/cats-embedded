/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "control/canard/controllers.hpp"

#include <cmath>

namespace canard {

void RollController::Reset() {
  m_integral = 0.0F;
  m_prev_out = 0.0F;
}

float RollController::Update(float roll_rate_dps, float gain_scale, float kp, float ki, float limit_deg,
                             float dt_s) {
  // A positive rate drives the output negative: freeze when already saturated that way.
  const bool saturated_same_dir = (roll_rate_dps > 0.0F && m_prev_out <= -limit_deg) ||
                                  (roll_rate_dps < 0.0F && m_prev_out >= limit_deg);
  if (!saturated_same_dir) {
    m_integral += roll_rate_dps * dt_s;
  }
  const float gain_i = gain_scale * ki;
  if (gain_i != 0.0F) {
    const float i_max = std::fabs(limit_deg / gain_i);
    m_integral = std::fmax(-i_max, std::fmin(i_max, m_integral));
  }
  const float u = -gain_scale * (kp * roll_rate_dps + ki * m_integral);
  m_prev_out = std::fmax(-limit_deg, std::fmin(limit_deg, u));
  return m_prev_out;
}

void TiltController::Update(const float tilt_deg[2], const float rate_t_dps[2], float gain_scale, float kp,
                            float kd, float v_mps, float tilt_v_min, float out_u[2]) const {
  if (v_mps < tilt_v_min) {
    out_u[0] = 0.0F;
    out_u[1] = 0.0F;
    return;
  }
  out_u[0] = -gain_scale * (kp * tilt_deg[0] + kd * rate_t_dps[1]);
  out_u[1] = -gain_scale * (kp * tilt_deg[1] - kd * rate_t_dps[0]);
}

}  // namespace canard
