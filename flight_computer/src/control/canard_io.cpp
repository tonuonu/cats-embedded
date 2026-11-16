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
#include "util/actions.hpp"
#include "util/enum_str_maps.hpp"
#include "util/log.h"

namespace {

constexpr uint8_t kNumOutputs = 4U;
constexpr int16_t kMaxTrimUs = 300;

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables) written once at boot
uint8_t boot_layout = static_cast<uint8_t>(canard::Layout::kOff);
canard_config_t boot_config{};
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

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

void canard_latch_boot_config() {
  boot_config = global_cats_config.canard;
  const auto layout = static_cast<canard::Layout>(boot_config.layout);
  const bool canard_on = (layout == canard::Layout::kRoll2) || (layout == canard::Layout::kFull4);
  boot_layout = static_cast<uint8_t>(canard_on ? layout : canard::Layout::kOff);
}

const canard_config_t& canard_boot_config() { return boot_config; }

uint8_t canard_boot_layout() { return boot_layout; }

bool canard_layout_claims(uint8_t layout, uint8_t servo_index) {
  const bool canard_on =
      layout == static_cast<uint8_t>(canard::Layout::kRoll2) || layout == static_cast<uint8_t>(canard::Layout::kFull4);
  return canard_on && servo_index < 2U;
}

bool canard_outputs_claimed(uint8_t servo_index) { return canard_layout_claims(boot_layout, servo_index); }

static_assert(NUM_EVENTS <= 16U, "canard_claimed_servo_events returns one bit per event in a uint16_t");

uint16_t canard_claimed_servo_events(uint8_t layout) {
  uint16_t events = 0U;
  for (uint8_t e = 0U; e < NUM_EVENTS; ++e) {
    const auto event = static_cast<cats_event_e>(e);
    const uint16_t n = cc_get_num_actions(event);
    for (uint16_t i = 0U; i < n; ++i) {
      config_action_t action{};
      if (!cc_get_action(event, i, &action)) {
        continue;
      }
      const bool servo_one = action.action_idx == ACT_SERVO_ONE && canard_layout_claims(layout, 0U);
      const bool servo_two = action.action_idx == ACT_SERVO_TWO && canard_layout_claims(layout, 1U);
      if (servo_one || servo_two) {
        events = static_cast<uint16_t>(events | (1U << e));
      }
    }
  }
  return events;
}

int16_t canard_trim_us(const canard_config_t& cfg, uint8_t k) {
  if (k >= kNumOutputs) {
    return 0;
  }
  const int16_t trim = cfg.trim_us[k];
  if (trim > kMaxTrimUs) {
    return kMaxTrimUs;
  }
  if (trim < -kMaxTrimUs) {
    return -kMaxTrimUs;
  }
  return trim;
}

uint16_t canard_running_claimed_servo_events() {
  uint16_t events = 0U;
  if (event_action_map == nullptr) {
    return events;
  }
  for (uint8_t e = 0U; e < NUM_EVENTS; ++e) {
    const event_action_map_elem_t& elem = event_action_map[e];
    for (uint8_t i = 0U; i < elem.num_actions && elem.action_list != nullptr; ++i) {
      const action_function_e action = elem.action_list[i].action;
      const bool servo_one = action == ACT_SERVO_ONE && canard_layout_claims(boot_layout, 0U);
      const bool servo_two = action == ACT_SERVO_TWO && canard_layout_claims(boot_layout, 1U);
      if (servo_one || servo_two) {
        events = static_cast<uint16_t>(events | (1U << e));
      }
    }
  }
  return events;
}

bool canard_step_axis_moves_nothing(const canard_config_t& cfg) {
  return cfg.layout == static_cast<uint8_t>(canard::Layout::kRoll2) &&
         (cfg.step_axis == static_cast<uint8_t>(canard::StepAxis::kT1) ||
          cfg.step_axis == static_cast<uint8_t>(canard::StepAxis::kT2));
}

bool canard_reverse(const canard_config_t& cfg, uint8_t k) { return k < kNumOutputs && cfg.reverse[k] != 0U; }

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

  m_cfg = canard_boot_config();
  const canard_config_t& cfg = m_cfg;
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
  m_missing_outputs = 0U;
  for (uint8_t k = 0; k < canard::NumOutputs(p.layout); ++k) {
    if (global_canard_outputs[k] == nullptr) {
      m_missing_outputs = static_cast<uint8_t>(m_missing_outputs | (1U << k));
      log_error("Canard output %u missing: canards held at trim, IMU fault latch", k + 1U);
    }
  }

  // Debug builds only (log_warn compiles out without CATS_DEV); release users see these in `ctrl_test status` and when
  // the CLI starts.
  if (p.layout == canard::Layout::kRoll2 && canard_step_axis_moves_nothing(cfg)) {
    log_warn("Canard: ctrl_step_axis %s with ROLL_2 (roll only): STEP doublets move no canard",
             ctrl_axis_map[static_cast<uint8_t>(p.step_axis)]);
  }

  const uint16_t servo_events = canard_running_claimed_servo_events();
  for (uint8_t e = 0U; e < NUM_EVENTS; ++e) {
    if ((servo_events & (1U << e)) != 0U) {
      log_warn("Event %s: servo action ignored, the canard layout owns SERVO1/SERVO2", event_map[e]);
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
  in.imu_faulty = imu_faulty || m_missing_outputs != 0U || m_calib_fault;
  const canard::ControlOutputs out = m_control.Update(in);
  if (m_mutex != nullptr) {
    osMutexRelease(m_mutex);
  }
  if (first_step && !calibrated) {
    log_error("Canard control: no IMU calibration at liftoff, canards held at trim");
  }

  const float us_per_deg = static_cast<float>(m_cfg.us_per_deg_deci) / 10.0F;
  for (uint8_t k = 0; k < out.n_outputs && k < kNumOutputs; ++k) {
    driver::ServoOutput* const output = global_canard_outputs[k];
    if (output != nullptr) {
      output->SetPulseUs(canard::PulseUs(out.defl_deg[k], m_cfg.center_us, canard_trim_us(m_cfg, k),
                                         canard_reverse(m_cfg, k), us_per_deg));
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
  const bool accepted = m_started && m_missing_outputs == 0U && !m_calib_fault && m_control.StartTest(cmd);
  osMutexRelease(m_mutex);
  return accepted;
}

bool CanardIo::CancelTest() {
  if (m_mutex == nullptr || osMutexAcquire(m_mutex, osWaitForever) != osOK) {
    return false;
  }
  const bool was_running = m_control.CancelTest();
  osMutexRelease(m_mutex);
  return was_running;
}

CanardStatus CanardIo::Status() {
  CanardStatus st;
  if (m_mutex == nullptr || osMutexAcquire(m_mutex, osWaitForever) != osOK) {
    return st;
  }
  st.started = m_started;
  st.n_outputs = canard::NumOutputs(static_cast<canard::Layout>(canard_boot_layout()));
  st.missing_outputs = m_missing_outputs;
  st.pad_sample_seen = m_control.PadSampleSeen();
  st.logged_reason = m_control.LoggedReason();
  st.calib_fault = m_calib_fault;
  st.test_running = m_control.TestRunning();
  st.frame_ok = m_control.FrameCheckOk();
  st.pad_acc_z_mps2 = m_control.PadAccZMps2();
  st.state = m_control.State();
  st.reason = m_control.Reason();
  osMutexRelease(m_mutex);
  return st;
}
