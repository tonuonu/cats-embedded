/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

namespace canard {

/// Body->world quaternion attitude estimator for canard roll/tilt control.
/// Body frame: x = t1, y = t2, z = roll axis (nose). World z is up.
class ControlAttitude {
 public:
  void Reset();
  /// On the pad: integrates the gyro, levels toward the accelerometer and learns the gyro bias.
  void UpdatePad(const float gyro_body_dps[3], const float acc_body_mps2[3], float dt_s);
  /// In flight: bias-corrected gyro integration only (no levelling, bias frozen).
  void UpdateFlight(const float gyro_body_dps[3], float dt_s);

  float RollRateDps() const { return rate_dps_[2]; }
  void TransverseRatesDps(float out[2]) const;
  /// Nose lean toward body +t1 / +t2 in degrees.
  void TiltDeg(float out[2]) const;
  /// Angle between the nose and world up in degrees, acos(up_z): 0 upright, 180 inverted. Unlike the TiltDeg
  /// components (asin, so 0 again when inverted) it tells an upside-down estimate apart from an upright one.
  float TiltMagDeg() const;
  void GyroBiasDps(float out[3]) const;

 private:
  void Integrate(const float omega_rad_s[3], float dt_s);

  float q_[4] = {1.0F, 0.0F, 0.0F, 0.0F};
  float bias_dps_[3] = {0.0F, 0.0F, 0.0F};
  float rate_dps_[3] = {0.0F, 0.0F, 0.0F};
};

}  // namespace canard
