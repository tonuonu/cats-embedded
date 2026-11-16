/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "control/canard/attitude.hpp"

#include <cmath>

namespace canard {

namespace {
constexpr float kPi = 3.14159265358979F;
constexpr float kDegToRad = kPi / 180.0F;
constexpr float kRadToDeg = 180.0F / kPi;
constexpr float kGravity = 9.81F;
constexpr float kAccTolerance = 0.5F;
constexpr float kLevelTauS = 2.0F;
constexpr float kBiasTauS = 10.0F;
constexpr float kBiasGateDps = 2.0F;

float Clamp1(float v) { return std::fmax(-1.0F, std::fmin(1.0F, v)); }
}  // namespace

void ControlAttitude::Reset() {
  q_[0] = 1.0F;
  q_[1] = 0.0F;
  q_[2] = 0.0F;
  q_[3] = 0.0F;
  for (int i = 0; i < 3; ++i) {
    bias_dps_[i] = 0.0F;
    rate_dps_[i] = 0.0F;
  }
}

void ControlAttitude::Integrate(const float omega_rad_s[3], float dt_s) {
  // q <- q (x) dq, dq = rotation by omega*dt expressed in the body frame.
  const float hx = 0.5F * omega_rad_s[0] * dt_s;
  const float hy = 0.5F * omega_rad_s[1] * dt_s;
  const float hz = 0.5F * omega_rad_s[2] * dt_s;
  const float w = q_[0];
  const float x = q_[1];
  const float y = q_[2];
  const float z = q_[3];
  float nw = w - x * hx - y * hy - z * hz;
  float nx = x + w * hx + y * hz - z * hy;
  float ny = y + w * hy - x * hz + z * hx;
  float nz = z + w * hz + x * hy - y * hx;
  const float n = std::sqrt(nw * nw + nx * nx + ny * ny + nz * nz);
  if (n > 0.0F) {
    nw /= n;
    nx /= n;
    ny /= n;
    nz /= n;
  }
  q_[0] = nw;
  q_[1] = nx;
  q_[2] = ny;
  q_[3] = nz;
}

void ControlAttitude::UpdateFlight(const float gyro_body_dps[3], float dt_s) {
  float omega[3] = {0.0F, 0.0F, 0.0F};
  for (int i = 0; i < 3; ++i) {
    rate_dps_[i] = gyro_body_dps[i] - bias_dps_[i];
    omega[i] = rate_dps_[i] * kDegToRad;
  }
  Integrate(omega, dt_s);
}

void ControlAttitude::UpdatePad(const float gyro_body_dps[3], const float acc_body_mps2[3], float dt_s) {
  const float acc_norm = std::sqrt(acc_body_mps2[0] * acc_body_mps2[0] + acc_body_mps2[1] * acc_body_mps2[1] +
                                   acc_body_mps2[2] * acc_body_mps2[2]);
  const bool acc_valid = std::fabs(acc_norm - kGravity) <= kAccTolerance;

  float omega[3] = {0.0F, 0.0F, 0.0F};
  bool quiet = true;
  for (int i = 0; i < 3; ++i) {
    rate_dps_[i] = gyro_body_dps[i] - bias_dps_[i];
    omega[i] = rate_dps_[i] * kDegToRad;
    quiet = quiet && std::fabs(rate_dps_[i]) < kBiasGateDps;
  }

  if (acc_valid) {
    // Estimated and measured "up" in the body frame (up_b = third row of R).
    const float x = q_[1];
    const float y = q_[2];
    const float z = q_[3];
    const float w = q_[0];
    const float up[3] = {2.0F * (x * z - w * y), 2.0F * (y * z + w * x), 1.0F - 2.0F * (x * x + y * y)};
    const float m[3] = {acc_body_mps2[0] / acc_norm, acc_body_mps2[1] / acc_norm, acc_body_mps2[2] / acc_norm};
    // Correction rate (m x up) / tau rotates up toward m.
    omega[0] += (m[1] * up[2] - m[2] * up[1]) / kLevelTauS;
    omega[1] += (m[2] * up[0] - m[0] * up[2]) / kLevelTauS;
    omega[2] += (m[0] * up[1] - m[1] * up[0]) / kLevelTauS;

    if (quiet) {
      const float alpha = dt_s / kBiasTauS;
      for (int i = 0; i < 3; ++i) {
        bias_dps_[i] += alpha * (gyro_body_dps[i] - bias_dps_[i]);
      }
    }
  }
  Integrate(omega, dt_s);
}

void ControlAttitude::TransverseRatesDps(float out[2]) const {
  out[0] = rate_dps_[0];
  out[1] = rate_dps_[1];
}

void ControlAttitude::TiltDeg(float out[2]) const {
  const float x = q_[1];
  const float y = q_[2];
  const float z = q_[3];
  const float w = q_[0];
  const float up_x = 2.0F * (x * z - w * y);
  const float up_y = 2.0F * (y * z + w * x);
  out[0] = std::asin(Clamp1(-up_x)) * kRadToDeg;
  out[1] = std::asin(Clamp1(-up_y)) * kRadToDeg;
}

void ControlAttitude::GyroBiasDps(float out[3]) const {
  for (int i = 0; i < 3; ++i) {
    out[i] = bias_dps_[i];
  }
}

}  // namespace canard
