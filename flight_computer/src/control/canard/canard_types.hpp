/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

namespace control::canard {

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
  Layout layout;
  FlightMode mode;
  float v_enable, v_ref, tilt_v_min, max_defl_deg, abort_tilt_deg;
  float roll_kp, roll_ki, tilt_kp, tilt_kd;
  StepAxis step_axis;
  float step_defl_deg, step_s, step_delay_s, step_period_s;
  uint8_t step_count;
  float authority_rate_dps = 180, authority_time_s = 0.5F, step_abort_rate_dps = 360, gyro_sat_dps = 1900;
  float slew_dps;
};

struct ControlInputs {
  float gyro_board_dps[3];
  float acc_board_mps2[3];
  uint8_t long_axis;
  float long_sign;
  float velocity_mps, altitude_agl_m;
  Phase phase;
  bool imu_faulty, sim_running;
  float dt_s;
};

struct ControlOutputs {
  float defl_deg[4];
  /// Would-be command, logged for SHADOW mode.
  float cmd_deg[4];
  uint8_t n_outputs;
  EnvState state;
  LatchReason reason;
  float roll_rate_dps, tilt_deg[2];
};

struct TestCommand {
  enum class Kind : uint8_t { kNone, kSweep, kLive } kind;
  float forced_v_mps;
  float duration_s;
};

}  // namespace control::canard
