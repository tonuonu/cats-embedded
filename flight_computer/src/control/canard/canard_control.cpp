/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "control/canard/canard_control.hpp"

#include <cmath>

#include "control/canard/frame.hpp"
#include "control/canard/mixer.hpp"
#include "control/canard/schedule.hpp"

namespace canard {

namespace {

using Source = EnvelopeDecision::Source;

constexpr float kSaturationTolDeg = 1e-3F;
/// Sweep test: each claimed output holds +max, -max and trim for this long, one output after the other.
constexpr float kSweepHoldS = 1.0F;
constexpr int kSweepPositions = 3;
/// Frame check: time constant of the pad body z specific force filter and the activation threshold (0.8 g).
constexpr float kFrameCheckTauS = 1.0F;
constexpr float kFrameCheckMinMps2 = 0.8F * 9.81F;

bool IsPadPhase(Phase phase) { return phase == Phase::kCalibrating || phase == Phase::kReady; }

bool AllFinite(const ControlInputs& in) {
  bool ok = std::isfinite(in.long_sign) && std::isfinite(in.velocity_mps) && std::isfinite(in.altitude_agl_m) &&
            std::isfinite(in.dt_s);
  for (int i = 0; i < 3; ++i) {
    ok = ok && std::isfinite(in.gyro_board_dps[i]) && std::isfinite(in.acc_board_mps2[i]);
  }
  return ok;
}

}  // namespace

void CanardControl::Configure(const CanardParams& params) {
  m_params = params;
  m_envelope.Configure(params);
  Reset();
}

void CanardControl::Reset() {
  m_attitude.Reset();
  m_roll.Reset();
  m_envelope.Reset();
  m_test = TestCommand{};
  m_test_running = false;
  m_test_t_s = 0.0F;
  m_last_phase = Phase::kCalibrating;
  m_last_state = (m_params.layout == Layout::kOff) ? EnvState::kDisabled : EnvState::kNeutralPad;
  m_last_reason = LatchReason::kNone;
  for (float& d : m_defl_deg) {
    d = 0.0F;
  }
  m_roll_rate_dps = 0.0F;
  m_tilt_deg[0] = 0.0F;
  m_tilt_deg[1] = 0.0F;
  m_tilt_mag_deg = 0.0F;
  m_pad_acc_z_mps2 = 0.0F;
  m_pad_acc_seen = false;
}

bool CanardControl::StartTest(const TestCommand& cmd) {
  // A test only ends on its timeout, a cancel or a phase change, so the duration must be finite and positive. A
  // non-finite forced speed would reach the gain schedule: refused, so the canards stay at trim.
  const bool valid = cmd.kind != TestCommand::Kind::kNone && std::isfinite(cmd.duration_s) && cmd.duration_s > 0.0F &&
                     (cmd.kind != TestCommand::Kind::kLive || std::isfinite(cmd.forced_v_mps));
  if (!valid || !m_envelope.StartTest(cmd, m_last_phase)) {
    return false;
  }
  m_test = cmd;
  m_test_running = true;
  m_test_t_s = 0.0F;
  return true;
}

bool CanardControl::CancelTest() {
  const bool was_running = m_test_running;
  m_envelope.CancelTest();
  m_test_running = false;
  return was_running;
}

bool CanardControl::FrameCheckOk() const { return m_pad_acc_seen && m_pad_acc_z_mps2 > kFrameCheckMinMps2; }

void CanardControl::TestCommandDeg(float dt_s, const float controller_deg[4], float out_deg[4]) {
  for (int k = 0; k < 4; ++k) {
    out_deg[k] = 0.0F;
  }
  if (m_test.kind == TestCommand::Kind::kLive) {
    for (int k = 0; k < 4; ++k) {
      out_deg[k] = controller_deg[k];
    }
    return;
  }
  // Sweep. Half a sample of tolerance keeps the position edges on the intended sample.
  const int index = static_cast<int>((m_test_t_s + 0.5F * dt_s) / kSweepHoldS);
  m_test_t_s += dt_s;
  const int output = index / kSweepPositions;
  if (output < NumOutputs(m_params.layout)) {
    const float max_defl = std::fmax(0.0F, m_params.max_defl_deg);
    const float position[kSweepPositions] = {max_defl, -max_defl, 0.0F};
    out_deg[output] = position[index % kSweepPositions];
  }
}

void CanardControl::SlewTo(const float target_deg[4], float dt_s) {
  const float rate =
      (std::isfinite(m_params.slew_dps) && m_params.slew_dps > 0.0F) ? m_params.slew_dps : kDefaultSlewDps;
  const float max_step = rate * std::fmax(0.0F, dt_s);
  for (int k = 0; k < 4; ++k) {
    const float step = std::fmax(-max_step, std::fmin(max_step, target_deg[k] - m_defl_deg[k]));
    m_defl_deg[k] += step;
  }
}

ControlOutputs CanardControl::Update(const ControlInputs& in) {
  ControlOutputs out;
  out.n_outputs = NumOutputs(m_params.layout);

  EnvelopeInputs env;
  env.phase = in.phase;
  env.sim_running = in.sim_running;
  env.dt_s = in.dt_s;

  if (!AllFinite(in)) {
    // Fail safe: nothing non-finite reaches the estimator, schedule, controllers or mixer. The envelope sees a
    // non-finite gyro magnitude and latches kImuFault; the outputs go straight to trim.
    env.imu_faulty = in.imu_faulty;
    env.v_mps = in.velocity_mps;
    env.roll_rate_dps = m_roll_rate_dps;
    env.tilt_mag_deg = m_tilt_mag_deg;
    env.max_abs_gyro_dps = std::nanf("");
    const EnvelopeDecision d = m_envelope.Update(env);
    m_test_running = false;
    for (float& defl : m_defl_deg) {
      defl = 0.0F;
    }
    out.state = d.state;
    out.reason = d.reason;
    out.roll_rate_dps = m_roll_rate_dps;
    out.tilt_deg[0] = m_tilt_deg[0];
    out.tilt_deg[1] = m_tilt_deg[1];
    m_last_phase = in.phase;
    m_last_state = d.state;
    m_last_reason = d.reason;
    return out;
  }

  // 1. Body frame.
  float gyro_body[3];
  float acc_body[3];
  BoardToBody(in.gyro_board_dps, in.long_axis, in.long_sign, gyro_body);
  BoardToBody(in.acc_board_mps2, in.long_axis, in.long_sign, acc_body);

  // 2. Attitude, and the frame check on the pad: the filter starts at the first pad sample and freezes in flight.
  if (IsPadPhase(in.phase)) {
    m_attitude.UpdatePad(gyro_body, acc_body, in.dt_s);
    if (m_pad_acc_seen) {
      const float alpha = std::fmin(1.0F, std::fmax(0.0F, in.dt_s / kFrameCheckTauS));
      m_pad_acc_z_mps2 += alpha * (acc_body[2] - m_pad_acc_z_mps2);
    } else {
      m_pad_acc_z_mps2 = acc_body[2];
      m_pad_acc_seen = true;
    }
  } else {
    m_attitude.UpdateFlight(gyro_body, in.dt_s);
  }
  m_roll_rate_dps = m_attitude.RollRateDps();
  m_attitude.TiltDeg(m_tilt_deg);
  m_tilt_mag_deg = m_attitude.TiltMagDeg();
  float rate_t[2];
  m_attitude.TransverseRatesDps(rate_t);

  // 3. Gain schedule. A live pad test runs the controller as if at its forced speed.
  const bool live_test = m_test_running && m_test.kind == TestCommand::Kind::kLive;
  const float v_ctrl = live_test ? m_test.forced_v_mps : in.velocity_mps;
  const float scale = GainScale(v_ctrl, in.altitude_agl_m, m_params.v_ref, m_params.v_enable);

  // 4. Controllers, in every state so SHADOW and trim log the real would-be command. The roll integrator is
  // held at zero while waiting on the pad, so it starts the flight (or a live test) without pad wind-up.
  if (m_last_state == EnvState::kNeutralPad && !m_test_running) {
    m_roll.Reset();
  }
  const float max_defl = m_params.max_defl_deg;
  const float u_roll = m_roll.Update(m_roll_rate_dps, scale, m_params.roll_kp, m_params.roll_ki, max_defl, in.dt_s);
  float u_tilt[2] = {0.0F, 0.0F};
  if (m_params.layout == Layout::kFull4) {
    TiltController::Update(m_tilt_deg, rate_t, scale, m_params.tilt_kp, m_params.tilt_kd, v_ctrl, m_params.tilt_v_min,
                           u_tilt);
  }
  float controller_deg[4];
  Mix(m_params.layout, u_roll, u_tilt[0], u_tilt[1], max_defl, controller_deg);

  // 5. Envelope, on the real velocity.
  float max_abs_gyro = 0.0F;
  for (const float g : in.gyro_board_dps) {
    max_abs_gyro = std::fmax(max_abs_gyro, std::fabs(g));
  }
  // A failed frame check counts as an IMU fault: ignored on the pad, it latches kImuFault in the activating Update.
  const bool frame_ok = FrameCheckOk();
  env.imu_faulty = in.imu_faulty || !frame_ok;
  env.v_mps = in.velocity_mps;
  env.roll_rate_dps = m_roll_rate_dps;
  env.tilt_mag_deg = m_tilt_mag_deg;
  env.max_abs_gyro_dps = max_abs_gyro;
  env.roll_cmd_deg = u_roll;
  env.roll_cmd_saturated = std::fabs(u_roll) >= max_defl - kSaturationTolDeg;
  const EnvelopeDecision d = m_envelope.Update(env);

  // 6. Command by source.
  // At trim, cmd_deg logs the controller's would-be command while the servos are sent 0.
  float cmd[4];
  switch (d.source) {
    case Source::kStep: {
      const float s = d.step_cmd_deg;
      Mix(m_params.layout, d.step_axis == StepAxis::kRoll ? s : 0.0F, d.step_axis == StepAxis::kT1 ? s : 0.0F,
          d.step_axis == StepAxis::kT2 ? s : 0.0F, max_defl, cmd);
      break;
    }
    case Source::kTest:
      TestCommandDeg(in.dt_s, controller_deg, cmd);
      break;
    case Source::kController:
    case Source::kTrim:
    default:
      for (int k = 0; k < 4; ++k) {
        cmd[k] = controller_deg[k];
      }
      break;
  }
  if (d.source != Source::kTest) {
    m_test_running = false;
  }
  float target[4];
  for (int k = 0; k < 4; ++k) {
    target[k] = (d.source == Source::kTrim) ? 0.0F : cmd[k];
  }

  // 7. Slew limit.
  SlewTo(target, in.dt_s);

  // 8. Outputs.
  for (int k = 0; k < 4; ++k) {
    out.defl_deg[k] = m_defl_deg[k];
    out.cmd_deg[k] = cmd[k];
  }
  out.state = d.state;
  // Visible on the pad (CONTROL_INFO, ctrl_test status): a failed frame check reports IMU_FAULT while the envelope
  // still waits in NEUTRAL_PAD. It latches only at activation; the envelope is not changed by this.
  out.reason = (d.state == EnvState::kNeutralPad && !frame_ok) ? LatchReason::kImuFault : d.reason;
  out.roll_rate_dps = m_roll_rate_dps;
  out.tilt_deg[0] = m_tilt_deg[0];
  out.tilt_deg[1] = m_tilt_deg[1];
  m_last_phase = in.phase;
  m_last_state = d.state;
  m_last_reason = d.reason;
  return out;
}

}  // namespace canard
