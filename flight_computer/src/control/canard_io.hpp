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

/** True when layout (a canard::Layout value) owns event servo output servo_index (0 = SERVO1, 1 = SERVO2). */
bool canard_layout_claims(uint8_t layout, uint8_t servo_index);

/** Events whose actions in the CURRENT config (global_cats_config.action_array, what the next boot builds) use
 * ACT_SERVO_ONE/ACT_SERVO_TWO on an output that layout claims: bit e set for event e (cats_event_e). Used for the
 * after-reboot warnings of `set ev_X` / `set ctrl_layout`.
 */
uint16_t canard_claimed_servo_events(uint8_t layout);

/** The same for the RUNNING firmware: the boot layout against the event action map built at boot (event_action_map,
 * which a later `set ev_X` does not change). These actions are ignored now; the boot log (CATS_DEV), the CLI start
 * and `ctrl_test status` warn about them.
 */
uint16_t canard_running_claimed_servo_events();

/** True when cfg has layout ROLL_2 and a T1/T2 STEP axis: ROLL_2 mixes roll only, so STEP doublets move no canard. */
bool canard_step_axis_moves_nothing(const canard_config_t& cfg);

/** Snapshot of the canard glue for the CLI (`ctrl_test status`), as of the last control step. */
struct CanardStatus {
  /// Step() has called CanardControl::Update: either READY with the long-axis calibration, or the FSM left the pad
  /// first, in which case calib_fault is set (control started without the calibration, IMU fault latch).
  bool started = false;
  bool calib_fault = false;
  uint8_t n_outputs = 0;        ///< outputs claimed by the boot layout
  uint8_t missing_outputs = 0;  ///< bit k: claimed output k is missing (Configure failed at boot), IMU fault latch
  bool test_running = false;
  bool pad_sample_seen = false;  ///< false: the frame check never ran (not started, or started off the pad)
  bool frame_ok = false;         ///< frame check (Ruling R24) passes
  float pad_acc_z_mps2 = 0.0F;
  canard::EnvState state = canard::EnvState::kDisabled;  ///< the envelope's own state and reason
  canard::LatchReason reason = canard::LatchReason::kNone;
  /// The reason logged in CONTROL_INFO: reason, or IMU_FAULT while NEUTRAL_PAD with a failed frame check.
  canard::LatchReason logged_reason = canard::LatchReason::kNone;
};

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

  /** True once Step() has called CanardControl::Update (control has started, see above). */
  [[nodiscard]] bool Started() const { return m_started; }

  /** Ends a running pad test (CLI `ctrl_test stop`): the canards slew back to trim. Serialised against Step().
   * Returns whether a test was running.
   */
  bool CancelTest();

  /** Status for `ctrl_test status`, read under the same mutex as Step(). */
  CanardStatus Status();

 private:
  canard::CanardControl m_control{};
  /// The canard settings as latched at boot, copied in Init.
  canard_config_t m_cfg{};
  StaticSemaphore_t m_mutex_cb{};
  osMutexId_t m_mutex{nullptr};
  bool m_started = false;
  /// Bit k: claimed output k is missing (its Configure failed at boot). Any bit set is an output fault.
  uint8_t m_missing_outputs = 0U;
  /// Control started without the long-axis calibration.
  bool m_calib_fault = false;
};
