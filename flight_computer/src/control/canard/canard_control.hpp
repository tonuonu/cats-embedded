/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "control/canard/attitude.hpp"
#include "control/canard/canard_types.hpp"
#include "control/canard/controllers.hpp"
#include "control/canard/envelope.hpp"

namespace canard {

/// Default gains at v_ref = 100 m/s for the camera-nose Peregrine model in test/test_canard_sim/vehicle.hpp.
/// Selection rule (the brief's pass criteria alone do not discriminate: aero damping meets most of them):
/// - roll: largest round kp with a small-signal gain margin >= 3 against the 10 ms delay and the servo, over
///   15-138 m/s and -15 % inertia; ki = 4/s * kp. Pinned by Sim_Roll_GainMargin (effectiveness x3 decays) and by
///   the residual-roll checks in Sim_Roll_DampsAt100, Sim_Roll_StableAcrossEffectiveness, Sim_Roll_LowSpeed and
///   Sim_MassSweep (the disturbance roll is removed to < 2 deg/s).
/// - tilt: 0 (Ruling R23). The rotation-only sim cannot validate tilt gains (the velocity stays vertical), so FULL_4
///   stops roll only until the stage-2 flight-path sim exists. The tuning-harness candidates (kp 1, kd 0.2) live in
///   test/test_canard_sim as kSimTiltKp/kSimTiltKd.
/// Roll: canard degrees per deg/s of roll rate and per degree of integrated roll angle.
constexpr float kDefaultRollKp = 0.05F;
constexpr float kDefaultRollKi = 0.2F;
/// Tilt: canard degrees per degree of tilt and per deg/s of transverse rate.
constexpr float kDefaultTiltKp = 0.0F;
constexpr float kDefaultTiltKd = 0.0F;
/// Output slew limit in deg/s (the CanardParams::slew_dps default). Also used whenever slew_dps is not a positive
/// finite number, which a configured value can still be.
constexpr float kDefaultSlewDps = 300.0F;

/// The canard control module behind one call per control step: body frame mapping, attitude, gain schedule,
/// roll and tilt controllers, envelope, mixer and output slew limit.
class CanardControl {
 public:
  /// Stores the parameters and resets all state.
  void Configure(const CanardParams& params);
  /// Back to the pad: attitude, controllers, envelope, test and outputs are cleared.
  void Reset();
  /// One control step. defl_deg is what the servos are sent; cmd_deg is the command of the active source, or,
  /// while the canards are held at trim, the controller's mixed would-be command.
  ControlOutputs Update(const ControlInputs& in);
  /// Starts a pad test (sweep or live). Accepted only when the envelope accepts it (on the pad, not latched,
  /// layout not kOff), the kind is not kNone, the duration is finite and > 0 and, for kLive, the forced speed is
  /// finite.
  bool StartTest(const TestCommand& cmd);
  void CancelTest();

 private:
  void TestCommandDeg(float dt_s, const float controller_deg[4], float out_deg[4]);
  void SlewTo(const float target_deg[4], float dt_s);

  CanardParams m_params{};
  ControlAttitude m_attitude{};
  RollController m_roll{};
  Envelope m_envelope{};
  TestCommand m_test{};
  bool m_test_running = false;
  float m_test_t_s = 0.0F;
  Phase m_last_phase = Phase::kCalibrating;
  EnvState m_last_state = EnvState::kDisabled;
  float m_defl_deg[4] = {};
  float m_roll_rate_dps = 0.0F;
  float m_tilt_deg[2] = {};
  float m_tilt_mag_deg = 0.0F;
  /// Frame check (Ruling R24): low-passed body z specific force on the pad. Activation needs it above 0.8 g, so a
  /// wrong long-axis sign (body z reads -1 g at rest) or no pad samples at all latches kImuFault instead.
  float m_pad_acc_z_mps2 = 0.0F;
  bool m_pad_acc_seen = false;
};

}  // namespace canard
