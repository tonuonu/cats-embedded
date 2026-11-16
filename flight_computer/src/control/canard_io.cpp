/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "control/canard_io.hpp"

#include <cmath>
#include <limits>

#include "config/cats_config.hpp"
#include "config/globals.hpp"
#include "control/canard/canard_types.hpp"
#include "control/canard/mixer.hpp"
#include "drivers/servo_output.hpp"
#include "flash/recorder.hpp"
#include "util/log.h"

namespace {

constexpr uint8_t kNumOutputs = 4U;
constexpr int16_t kMaxTrimUs = 300;

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables) written once at boot
uint8_t boot_layout = static_cast<uint8_t>(canard::Layout::kOff);

canard::Phase ToPhase(flight_fsm_e fsm) {
  switch (fsm) {
    case READY:
      return canard::Phase::kReady;
    case THRUSTING:
      return canard::Phase::kThrusting;
    case COASTING:
      return canard::Phase::kCoasting;
    case DROGUE:
    case MAIN:
    case TOUCHDOWN:
      return canard::Phase::kRecovery;
    case INVALID:
    case CALIBRATING:
    default:
      return canard::Phase::kCalibrating;
  }
}

/** value * scale rounded to nearest and saturated to T; NaN gives 0. */
template <typename T>
T ScaleSat(float value, float scale) {
  const float x = value * scale;
  if (std::isnan(x)) {
    return 0;
  }
  constexpr auto kLo = static_cast<float>(std::numeric_limits<T>::min());
  constexpr auto kHi = static_cast<float>(std::numeric_limits<T>::max());
  return static_cast<T>(std::lround(std::fmin(kHi, std::fmax(kLo, x))));
}

}  // namespace

void canard_latch_boot_layout() {
  const auto layout = static_cast<canard::Layout>(global_cats_config.canard.layout);
  const bool canard_on = (layout == canard::Layout::kRoll2) || (layout == canard::Layout::kFull4);
  boot_layout = static_cast<uint8_t>(canard_on ? layout : canard::Layout::kOff);
}

uint8_t canard_boot_layout() { return boot_layout; }

bool canard_outputs_claimed(uint8_t servo_index) {
  return boot_layout != static_cast<uint8_t>(canard::Layout::kOff) && servo_index < 2U;
}

int16_t canard_trim_us(uint8_t k) {
  if (k >= kNumOutputs) {
    return 0;
  }
  const int16_t trim = global_cats_config.canard.trim_us[k];
  if (trim > kMaxTrimUs) {
    return kMaxTrimUs;
  }
  if (trim < -kMaxTrimUs) {
    return -kMaxTrimUs;
  }
  return trim;
}

bool canard_reverse(uint8_t k) { return k < kNumOutputs && global_cats_config.canard.reverse[k] != 0U; }

void CanardIo::Init() {
  if (m_mutex == nullptr) {
    const osMutexAttr_t attr = {
        .name = "canard",
        .attr_bits = osMutexPrioInherit,
        .cb_mem = &m_mutex_cb,
        .cb_size = sizeof(m_mutex_cb),
    };
    m_mutex = osMutexNew(&attr);
  }

  const canard_config_t& cfg = global_cats_config.canard;
  canard::CanardParams p{};  // The authority, gyro saturation and slew limits keep their CanardParams defaults.
  p.layout = static_cast<canard::Layout>(canard_boot_layout());
  p.mode = cfg.flight_mode <= static_cast<uint8_t>(canard::FlightMode::kClosed)
               ? static_cast<canard::FlightMode>(cfg.flight_mode)
               : canard::FlightMode::kShadow;
  p.v_enable = static_cast<float>(cfg.v_enable_mps);
  p.v_ref = static_cast<float>(cfg.v_ref_mps);
  p.tilt_v_min = static_cast<float>(cfg.tilt_v_min_mps);
  p.max_defl_deg = static_cast<float>(cfg.max_defl_ddeg) / 10.0F;
  p.abort_tilt_deg = static_cast<float>(cfg.abort_tilt_deg);
  p.roll_kp = static_cast<float>(cfg.roll_kp_milli) / 1000.0F;
  p.roll_ki = static_cast<float>(cfg.roll_ki_milli) / 1000.0F;
  p.tilt_kp = static_cast<float>(cfg.tilt_kp_milli) / 1000.0F;
  p.tilt_kd = static_cast<float>(cfg.tilt_kd_milli) / 1000.0F;
  p.step_axis = cfg.step_axis <= static_cast<uint8_t>(canard::StepAxis::kT2)
                    ? static_cast<canard::StepAxis>(cfg.step_axis)
                    : canard::StepAxis::kRoll;
  p.step_defl_deg = static_cast<float>(cfg.step_defl_ddeg) / 10.0F;
  p.step_s = static_cast<float>(cfg.step_ms) / 1000.0F;
  p.step_delay_s = static_cast<float>(cfg.step_delay_ms) / 1000.0F;
  p.step_period_s = static_cast<float>(cfg.step_period_ms) / 1000.0F;
  p.step_count = cfg.step_count;
  m_control.Configure(p);  // Configure also resets all state.

  m_started = false;
  m_calib_fault = false;
  m_output_fault = false;
  for (uint8_t k = 0; k < canard::NumOutputs(p.layout); ++k) {
    if (global_canard_outputs[k] == nullptr) {
      m_output_fault = true;
      log_error("Canard output %u missing: canards held at trim, IMU fault latch", k + 1U);
    }
  }
}

void CanardIo::Step(uint32_t tick, flight_fsm_e fsm, const SI_data_t& si, float velocity, float height_agl,
                    bool imu_faulty) {
  const calibration_data_t calib = global_flight_stats.calibration_data;
  const bool calibrated = fsm >= READY && calib.angle != 0.0F;
  if (!m_started && !calibrated && fsm < THRUSTING) {
    return;  // Hold the boot pulse and record nothing until the initial calibration is done.
  }

  canard::ControlInputs in;
  in.gyro_board_dps[0] = si.gyro.x;
  in.gyro_board_dps[1] = si.gyro.y;
  in.gyro_board_dps[2] = si.gyro.z;
  in.acc_board_mps2[0] = si.acc.x;
  in.acc_board_mps2[1] = si.acc.y;
  in.acc_board_mps2[2] = si.acc.z;
  in.long_axis = calib.axis;
  in.long_sign = calib.angle < 0.0F ? -1.0F : 1.0F;
  in.velocity_mps = velocity;
  in.altitude_agl_m = height_agl;
  in.phase = ToPhase(fsm);
  in.sim_running = simulation_started;
  in.dt_s = 1.0F / static_cast<float>(CONTROL_SAMPLING_FREQ);

  const bool first_step = !m_started;
  if (m_mutex != nullptr) {
    osMutexAcquire(m_mutex, osWaitForever);
  }
  if (first_step) {
    m_started = true;
    m_calib_fault = !calibrated;
  }
  in.imu_faulty = imu_faulty || m_output_fault || m_calib_fault;
  const canard::ControlOutputs out = m_control.Update(in);
  if (m_mutex != nullptr) {
    osMutexRelease(m_mutex);
  }
  if (first_step && !calibrated) {
    log_error("Canard control: no IMU calibration at liftoff, canards held at trim");
  }

  const canard_config_t& cfg = global_cats_config.canard;
  const float us_per_deg = static_cast<float>(cfg.us_per_deg_deci) / 10.0F;
  for (uint8_t k = 0; k < out.n_outputs && k < kNumOutputs; ++k) {
    driver::ServoOutput* const output = global_canard_outputs[k];
    if (output != nullptr) {
      output->SetPulseUs(
          canard::PulseUs(out.defl_deg[k], cfg.center_us, canard_trim_us(k), canard_reverse(k), us_per_deg));
    }
  }

  control_info_t info{};
  info.roll_rate_ddps = ScaleSat<int16_t>(out.roll_rate_dps, 10.0F);
  info.tilt_cdeg[0] = ScaleSat<int16_t>(out.tilt_deg[0], 100.0F);
  info.tilt_cdeg[1] = ScaleSat<int16_t>(out.tilt_deg[1], 100.0F);
  // Ruling R1: defl_ddeg logs the would-be command cmd_deg (in SHADOW and at trim the controller's mixed command),
  // not the deflection sent to the servos.
  for (uint8_t k = 0; k < kNumOutputs; ++k) {
    info.defl_ddeg[k] = ScaleSat<int8_t>(out.cmd_deg[k], 10.0F);
  }
  info.state = static_cast<uint8_t>(out.state);
  info.reason = static_cast<uint8_t>(out.reason);
  record(tick, CONTROL_INFO, &info);
}

bool CanardIo::StartTest(const canard::TestCommand& cmd) {
  if (m_mutex == nullptr || osMutexAcquire(m_mutex, osWaitForever) != osOK) {
    return false;
  }
  const bool accepted = m_started && !m_output_fault && !m_calib_fault && m_control.StartTest(cmd);
  osMutexRelease(m_mutex);
  return accepted;
}

void CanardIo::CancelTest() {
  if (m_mutex == nullptr || osMutexAcquire(m_mutex, osWaitForever) != osOK) {
    return;
  }
  m_control.CancelTest();
  osMutexRelease(m_mutex);
}
