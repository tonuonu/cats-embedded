/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

#include "cmsis_os.h"
#include "control/canard/canard_control.hpp"
#include "util/types.hpp"

/** Snapshot global_cats_config.canard once, right after the config loads (Ruling R22: every canard setting is
 * latched at boot). The layout is stored as OFF unless it is ROLL_2 or FULL_4. The output setup, the boot pulse and
 * CanardIo all use this snapshot, so a later CLI `set ctrl_*` only takes effect after save + reboot.
 */
void canard_latch_boot_config();

/** The canard settings latched at boot; all zero until canard_latch_boot_config() runs. */
const canard_config_t& canard_boot_config();

/** The layout latched at boot (a canard::Layout value); OFF until canard_latch_boot_config() runs. */
uint8_t canard_boot_layout();

/** True when the boot layout owns event servo output servo_index (0 = SERVO1, 1 = SERVO2).
 *
 * ROLL_2 and FULL_4 claim both; OFF claims none, so the event actions keep them.
 */
bool canard_outputs_claimed(uint8_t servo_index);

/** Trim of canard output k (0..3) in cfg, in us, clamped to +-300 us; 0 for k > 3. */
int16_t canard_trim_us(const canard_config_t& cfg, uint8_t k);

/** Reverse flag of canard output k (0..3) in cfg: the configured value != 0; false for k > 3. */
bool canard_reverse(const canard_config_t& cfg, uint8_t k);

/** Firmware side of the canard control module: config to parameters, flight data to ControlInputs, deflections to
 * servo pulses and the CONTROL_INFO record. Step() runs in the StateEstimation task; StartTest() may be called from
 * another task (the CLI) and is serialised against Step() by a mutex.
 *
 * Start of control: Step() only calls CanardControl::Update once the initial calibration is done, i.e. the FSM has
 * reached READY and Preprocessing has stored the long-axis calibration (global_flight_stats.calibration_data.angle is
 * 0 until then, calibrate_imu never stores 0). Until then the outputs hold their boot pulse (centre + trim) and
 * nothing is recorded, so a startup artefact cannot latch the envelope (Ruling R11). Should the FSM leave the pad
 * without that calibration, control starts anyway with an IMU fault, so the envelope latches and the log shows it.
 */
class CanardIo {
 public:
  /** Copies the boot snapshot of the canard settings (canard_boot_config()), converts it to CanardParams, configures
   * the module and checks that every output of the boot layout exists. A missing output is logged once and treated
   * as an IMU fault. Step() only uses this copy, never the live global_cats_config.
   */
  void Init();

  /** One control step at CONTROL_SAMPLING_FREQ: Update, servo pulses from defl_deg, CONTROL_INFO record. */
  void Step(uint32_t tick, flight_fsm_e fsm, const SI_data_t& si, float velocity, float height_agl, bool imu_faulty);

  /** Starts a pad test. Refused before Init, before control has started (see above) and with a missing output. */
  bool StartTest(const canard::TestCommand& cmd);

  /** Ends a running pad test (CLI `ctrl_test stop`): the canards slew back to trim. Serialised against Step(). */
  void CancelTest();

 private:
  canard::CanardControl m_control{};
  /// The canard settings as latched at boot, copied in Init.
  canard_config_t m_cfg{};
  StaticSemaphore_t m_mutex_cb{};
  osMutexId_t m_mutex{nullptr};
  bool m_started = false;
  /// A claimed output is missing (its Configure failed at boot).
  bool m_output_fault = false;
  /// Control started without the long-axis calibration.
  bool m_calib_fault = false;
};
