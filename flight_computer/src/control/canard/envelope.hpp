/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

#include "control/canard/canard_types.hpp"

namespace canard {

struct EnvelopeInputs {
  Phase phase = Phase::kCalibrating;
  float v_mps = 0.0F, roll_rate_dps = 0.0F, tilt_mag_deg = 0.0F, max_abs_gyro_dps = 0.0F;
  bool imu_faulty = false, sim_running = false;
  float roll_cmd_deg = 0.0F;
  bool roll_cmd_saturated = false;
  float dt_s = 0.0F;
};

struct EnvelopeDecision {
  enum class Source : std::uint8_t { kTrim, kController, kStep, kTest };
  EnvState state = EnvState::kDisabled;
  LatchReason reason = LatchReason::kNone;
  Source source = Source::kTrim;
  float step_cmd_deg = 0.0F;
  StepAxis step_axis = StepAxis::kRoll;
};

/// Control envelope: pad / active / latched state machine, flight modes, STEP doublets and the pad test
/// override. Time is accumulated from the dt of each Update; there is no wall clock. A latch is permanent
/// until Reset().
class Envelope {
 public:
  void Configure(const CanardParams& params);
  /// Back to kNeutralPad, or kDisabled when the layout is kOff.
  void Reset();
  EnvelopeDecision Update(const EnvelopeInputs& in);
  /// Accepted only in phase kCalibrating/kReady while on the pad. Never accepted with layout kOff.
  bool StartTest(const TestCommand& cmd, Phase phase);
  void CancelTest();

 private:
  bool ExitCondition(const EnvelopeInputs& in, LatchReason* reason);
  void Latch(LatchReason reason);
  float StepCommand(float dt_s);

  CanardParams m_params{};
  EnvState m_state = EnvState::kDisabled;
  LatchReason m_reason = LatchReason::kNone;
  float m_authority_timer_s = 0.0F;
  bool m_step_started = false;
  float m_step_t_s = 0.0F;
  bool m_test_running = false;
  Phase m_test_phase = Phase::kCalibrating;
  float m_test_elapsed_s = 0.0F;
  float m_test_duration_s = 0.0F;
};

}  // namespace canard
