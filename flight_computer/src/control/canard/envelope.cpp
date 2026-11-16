/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "control/canard/envelope.hpp"

#include <cmath>

namespace canard {

namespace {

using Source = EnvelopeDecision::Source;

bool IsPadPhase(Phase phase) { return phase == Phase::kCalibrating || phase == Phase::kReady; }

}  // namespace

void Envelope::Configure(const CanardParams& params) { m_params = params; }

void Envelope::Reset() {
  m_state = (m_params.layout == Layout::kOff) ? EnvState::kDisabled : EnvState::kNeutralPad;
  m_reason = LatchReason::kNone;
  m_authority_timer_s = 0.0F;
  m_step_started = false;
  m_step_t_s = 0.0F;
  m_test_running = false;
  m_test_elapsed_s = 0.0F;
}

bool Envelope::StartTest(const TestCommand& cmd, Phase phase) {
  if (m_state != EnvState::kNeutralPad || !IsPadPhase(phase)) {
    return false;
  }
  m_test_running = true;
  m_test_phase = phase;
  m_test_elapsed_s = 0.0F;
  m_test_duration_s = cmd.duration_s;
  return true;
}

void Envelope::CancelTest() { m_test_running = false; }

void Envelope::Latch(LatchReason reason) {
  m_state = EnvState::kNeutralLatched;
  m_reason = reason;
  m_test_running = false;
}

/// Exit priority when several conditions hit in the same Update (first match wins): kSim, kImuFault,
/// kGyroSat, kDrogue, kTilt, kStepRollRate, kAuthority, kLowSpeed. Applies to kActive; kSim also applies
/// from kNeutralPad (handled in Update).
bool Envelope::ExitCondition(const EnvelopeInputs& in, LatchReason* reason) {
  // The authority timer only runs while its condition holds; it resets whenever the condition is false.
  const bool authority_cond =
      m_params.mode == FlightMode::kClosed && std::fabs(in.roll_rate_dps) > m_params.authority_rate_dps &&
      in.roll_cmd_saturated && ((in.roll_cmd_deg > 0.0F && in.roll_rate_dps < 0.0F) ||
                                (in.roll_cmd_deg < 0.0F && in.roll_rate_dps > 0.0F));
  m_authority_timer_s = authority_cond ? m_authority_timer_s + in.dt_s : 0.0F;

  if (in.sim_running) {
    *reason = LatchReason::kSim;
  } else if (in.imu_faulty) {
    *reason = LatchReason::kImuFault;
  } else if (in.max_abs_gyro_dps >= m_params.gyro_sat_dps) {
    *reason = LatchReason::kGyroSat;
  } else if (in.phase == Phase::kRecovery) {
    *reason = LatchReason::kDrogue;
  } else if (in.tilt_mag_deg > m_params.abort_tilt_deg) {
    *reason = LatchReason::kTilt;
  } else if (m_params.mode == FlightMode::kStep && std::fabs(in.roll_rate_dps) > m_params.step_abort_rate_dps) {
    *reason = LatchReason::kStepRollRate;
  } else if (authority_cond && m_authority_timer_s >= m_params.authority_time_s) {
    *reason = LatchReason::kAuthority;
  } else if (in.phase == Phase::kCoasting && in.v_mps < m_params.v_enable) {
    *reason = LatchReason::kLowSpeed;
  } else {
    return false;
  }
  return true;
}

/// Advances the STEP clock by dt (the first call is t = 0) and returns the doublet command at that time,
/// latching kStepDone once the last doublet has ended.
float Envelope::StepCommand(float dt_s) {
  if (!m_step_started) {
    m_step_started = true;
    m_step_t_s = 0.0F;
  }
  const float t = m_step_t_s;
  m_step_t_s += dt_s;
  const float last_end =
      m_params.step_delay_s +
      static_cast<float>(m_params.step_count > 0 ? m_params.step_count - 1 : 0) * m_params.step_period_s +
      2.0F * m_params.step_s;
  if (m_params.step_count == 0 || t >= last_end) {
    Latch(LatchReason::kStepDone);
    return 0.0F;
  }
  for (std::uint8_t i = 0; i < m_params.step_count; ++i) {
    const float local = t - (m_params.step_delay_s + static_cast<float>(i) * m_params.step_period_s);
    if (local >= 0.0F && local < 2.0F * m_params.step_s) {
      const float sign = (i % 2 == 0) ? 1.0F : -1.0F;
      return (local < m_params.step_s ? sign : -sign) * m_params.step_defl_deg;
    }
  }
  return 0.0F;
}

EnvelopeDecision Envelope::Update(const EnvelopeInputs& in) {
  EnvelopeDecision d;
  d.step_axis = m_params.step_axis;

  if (m_state == EnvState::kNeutralPad) {
    if (in.sim_running) {
      Latch(LatchReason::kSim);
    } else if (in.phase >= Phase::kThrusting && in.v_mps >= m_params.v_enable) {
      m_state = EnvState::kActive;
    }
  }

  if (m_state == EnvState::kActive) {
    LatchReason reason = LatchReason::kNone;
    if (ExitCondition(in, &reason)) {
      Latch(reason);
    }
  }

  if (m_state == EnvState::kActive && m_params.mode == FlightMode::kStep && in.phase == Phase::kCoasting) {
    d.step_cmd_deg = StepCommand(in.dt_s);
  }

  // Test override: only meaningful on the pad; ends on a phase change, a latch or when its time is up.
  if (m_test_running) {
    m_test_elapsed_s += in.dt_s;
    if (m_state != EnvState::kNeutralPad || in.phase != m_test_phase ||
        m_test_elapsed_s >= m_test_duration_s) {
      m_test_running = false;
    }
  }

  d.state = m_state;
  d.reason = m_reason;
  d.source = Source::kTrim;
  if (m_state == EnvState::kActive) {
    if (m_params.mode == FlightMode::kClosed) {
      d.source = Source::kController;
    } else if (m_params.mode == FlightMode::kStep) {
      d.source = Source::kStep;
    }
  }
  if (m_test_running) {
    d.source = Source::kTest;
  }
  return d;
}

}  // namespace canard
