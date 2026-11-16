/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

namespace canard {

enum class Layout : uint8_t { kOff, kRoll2, kFull4 };
enum class FlightMode : uint8_t { kShadow, kStep, kClosed };
enum class StepAxis : uint8_t { kRoll, kT1, kT2 };
/// The glue maps CALIBRATING/READY/THRUSTING/COASTING and DROGUE+ (kRecovery) to these.
enum class Phase : uint8_t { kCalibrating, kReady, kThrusting, kCoasting, kRecovery };
enum class EnvState : uint8_t { kDisabled, kNeutralPad, kActive, kNeutralLatched };
enum class LatchReason : uint8_t {
  kNone,
  kDrogue,
  kLowSpeed,
  kTilt,
  kGyroSat,
  kImuFault,
  kAuthority,
  kStepDone,
  kStepRollRate,
  kSim
};

struct CanardParams {
  Layout layout = Layout::kOff;
  FlightMode mode = FlightMode::kShadow;
  float v_enable = 0.0F, v_ref = 0.0F, tilt_v_min = 0.0F, max_defl_deg = 0.0F, abort_tilt_deg = 0.0F;
  float roll_kp = 0.0F, roll_ki = 0.0F, tilt_kp = 0.0F, tilt_kd = 0.0F;
  StepAxis step_axis = StepAxis::kRoll;
  float step_defl_deg = 0.0F, step_s = 0.0F, step_delay_s = 0.0F, step_period_s = 0.0F;
  uint8_t step_count = 0;
  float authority_rate_dps = 90, authority_time_s = 0.5F, step_abort_rate_dps = 360, gyro_sat_dps = 1900;
  float slew_dps = 300.0F;
};

struct ControlInputs {
  float gyro_board_dps[3] = {};
  float acc_board_mps2[3] = {};
  uint8_t long_axis = 0;
  float long_sign = 1.0F;
  float velocity_mps = 0.0F, altitude_agl_m = 0.0F;
  Phase phase = Phase::kCalibrating;
  bool imu_faulty = false, sim_running = false;
  float dt_s = 0.0F;
};

struct ControlOutputs {
  float defl_deg[4] = {};
  /// Would-be command, logged for SHADOW mode.
  float cmd_deg[4] = {};
  uint8_t n_outputs = 0;
  EnvState state = EnvState::kDisabled;
  LatchReason reason = LatchReason::kNone;
  float roll_rate_dps = 0.0F, tilt_deg[2] = {};
};

struct TestCommand {
  enum class Kind : uint8_t { kNone, kSweep, kLive } kind = Kind::kNone;
  float forced_v_mps = 0.0F;
  float duration_s = 0.0F;
};

}  // namespace canard
