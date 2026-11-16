/// Copyright (C) 2020, 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later
///
/// Additional notice:
/// This file was adapted from Betaflight (https://github.com/betaflight/betaflight),
/// released under GNU General Public License v3.0.

#include "cli/settings.hpp"
#include "cli/cli.hpp"
#include "util/enum_str_maps.hpp"

#include "control/canard/canard_types.hpp"
#include "control/canard_io.hpp"

#include <cstddef>
#include <cstdint>

/* The lookup tables in enum_str_maps.hpp are indexed by the canard enum values. */
static_assert(static_cast<size_t>(canard::Layout::kOff) == 0 && static_cast<size_t>(canard::Layout::kRoll2) == 1 &&
              static_cast<size_t>(canard::Layout::kFull4) == 2);
static_assert(static_cast<size_t>(canard::FlightMode::kShadow) == 0 &&
              static_cast<size_t>(canard::FlightMode::kStep) == 1 &&
              static_cast<size_t>(canard::FlightMode::kClosed) == 2);
static_assert(static_cast<size_t>(canard::StepAxis::kRoll) == 0 && static_cast<size_t>(canard::StepAxis::kT1) == 1 &&
              static_cast<size_t>(canard::StepAxis::kT2) == 2);

/* ctrl_max_defl upper limit, 0.1 deg: the mechanical stops are at +-11 deg. CONTROL_INFO logs deflections as int8 in
 * 0.1 deg, so the limit must stay within its +-12.7 deg. */
constexpr uint8_t kCtrlMaxDeflDdeg = 110;
static_assert(kCtrlMaxDeflDdeg <= INT8_MAX, "CONTROL_INFO logs the deflection as int8 in 0.1 deg");

#undef LOOKUP_TABLE_ENTRY

/* Every canard setting is latched at boot (control/canard_io.hpp, Ruling R22). */
static void ctrl_reboot_note(const cli_value_t *var) {
  cli_print_linefeed();
  cli_printf("%s takes effect after save + reboot", var->name);
}

/* One line per event in `events` (bit e = cats_event_e e) whose SERVO_ONE/SERVO_TWO action layout ignores. running:
 * the running firmware's boot-built event map against the boot layout (ignored now); otherwise the current config
 * against the configured layout (ignored after reboot). Warn only: the actions are kept. */
static void warn_claimed_servo_actions(uint16_t events, uint8_t layout, bool running) {
  for (uint8_t e = 0U; e < NUM_EVENTS; ++e) {
    if ((events & (1U << e)) == 0U) {
      continue;
    }
    cli_print_linefeed();
    if (running) {
      cli_printf(
          "warning: running firmware: event %s has a SERVO_ONE/SERVO_TWO action (boot-time event map); the boot "
          "ctrl_layout %s owns SERVO1/SERVO2, so it is ignored now",
          event_map[e], ctrl_layout_map[layout]);
    } else {
      cli_printf(
          "warning: after reboot: event %s has a SERVO_ONE/SERVO_TWO action (current config); ctrl_layout %s "
          "owns SERVO1/SERVO2, so it will be ignored",
          event_map[e], ctrl_layout_map[layout]);
    }
  }
}

/* ROLL_2 mixes roll only: a STEP doublet on T1/T2 would move no canard (warn only, the setting is kept). */
static void warn_step_axis(const canard_config_t &cfg, const char *when) {
  if (canard_step_axis_moves_nothing(cfg)) {
    cli_print_linefeed();
    cli_printf("warning: ctrl_step_axis %s with ctrl_layout ROLL_2 (roll only)%s: STEP doublets move no canard",
               ctrl_axis_map[cfg.step_axis], when);
  }
}

void cli_print_canard_boot_warnings() {
  warn_claimed_servo_actions(canard_running_claimed_servo_events(), canard_boot_layout(), true);
  warn_step_axis(canard_boot_config(), " in the running firmware");
}

/* set ev_X: event X only, against the layout configured for the next boot; the running layout is mentioned when it
 * differs. */
static void ev_servo_check(const cli_value_t *var) {
  const size_t first = offsetof(cats_config_t, action_array);
  const size_t index = (var->member_offset - first) / sizeof(global_cats_config.action_array[0]);
  if (index >= NUM_EVENTS) {
    return;
  }
  const auto bit = static_cast<uint16_t>(1U << index);
  const uint8_t configured = global_cats_config.canard.layout;
  const uint8_t running = canard_boot_layout();
  if ((canard_claimed_servo_events(configured) & bit) != 0U) {
    warn_claimed_servo_actions(bit, configured, false);
    if (running != configured) {
      cli_print_linefeed();
      cli_printf("(running now: ctrl_layout %s)", ctrl_layout_map[running]);
    }
  } else if ((canard_running_claimed_servo_events() & bit) != 0U) {
    cli_print_linefeed();
    // The configured layout does not claim the outputs here (else the branch above), so only the running state is
    // described: whether the action exists after reboot depends on what is saved.
    cli_printf(
        "note: the running firmware ignores event %s's SERVO_ONE/SERVO_TWO action (boot-time event map; the "
        "boot ctrl_layout %s owns SERVO1/SERVO2); the configured ctrl_layout does not claim them after reboot",
        event_map[index], ctrl_layout_map[running]);
  }
}

/* set ctrl_layout: reboot note, then the event servo actions the new layout would claim and the STEP axis. */
static void ctrl_layout_cb(const cli_value_t *var) {
  ctrl_reboot_note(var);
  const uint8_t configured = global_cats_config.canard.layout;
  warn_claimed_servo_actions(canard_claimed_servo_events(configured), configured, false);
  warn_step_axis(global_cats_config.canard, " after reboot");
}

/* set ctrl_step_axis: reboot note and the STEP axis check. */
static void ctrl_step_axis_cb(const cli_value_t *var) {
  ctrl_reboot_note(var);
  warn_step_axis(global_cats_config.canard, " after reboot");
}

const cli_value_t value_table[] = {
    // Control
    {"acc_threshold",
     VAR_UINT16,
     {.minmax_unsigned = {30, 80}},
     offsetof(cats_config_t, control_settings.liftoff_acc_threshold)},
    {"main_altitude",
     VAR_UINT16,
     {.minmax_unsigned = {10, 65535}},
     offsetof(cats_config_t, control_settings.main_altitude)},

    // Timers
    {"timer1_start",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_EVENTS}},
     offsetof(cats_config_t, timers[0].start_event)},
    {"timer1_trigger",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_EVENTS}},
     offsetof(cats_config_t, timers[0].trigger_event)},
    {"timer1_duration", VAR_UINT32, {.u32_max = 1200000}, offsetof(cats_config_t, timers[0].duration)},
    {"timer2_start",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_EVENTS}},
     offsetof(cats_config_t, timers[1].start_event)},
    {"timer2_trigger",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_EVENTS}},
     offsetof(cats_config_t, timers[1].trigger_event)},
    {"timer2_duration", VAR_UINT32, {.u32_max = 1200000}, offsetof(cats_config_t, timers[1].duration)},
    {"timer3_start",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_EVENTS}},
     offsetof(cats_config_t, timers[2].start_event)},
    {"timer3_trigger",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_EVENTS}},
     offsetof(cats_config_t, timers[2].trigger_event)},
    {"timer3_duration", VAR_UINT32, {.u32_max = 1200000}, offsetof(cats_config_t, timers[2].duration)},
    {"timer4_start",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_EVENTS}},
     offsetof(cats_config_t, timers[3].start_event)},
    {"timer4_trigger",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_EVENTS}},
     offsetof(cats_config_t, timers[3].trigger_event)},
    {"timer4_duration", VAR_UINT32, {.u32_max = 1200000}, offsetof(cats_config_t, timers[3].duration)},

    // Events
    {"ev_calibrate",
     VAR_INT16 | MODE_ARRAY,
     {.array = {.length = 16}},
     offsetof(cats_config_t, action_array[EV_CALIBRATE]),
     ev_servo_check},
    {"ev_ready",
     VAR_INT16 | MODE_ARRAY,
     {.array = {.length = 16}},
     offsetof(cats_config_t, action_array[EV_READY]),
     ev_servo_check},
    {"ev_liftoff",
     VAR_INT16 | MODE_ARRAY,
     {.array = {.length = 16}},
     offsetof(cats_config_t, action_array[EV_LIFTOFF]),
     ev_servo_check},
    {"ev_burnout",
     VAR_INT16 | MODE_ARRAY,
     {.array = {.length = 16}},
     offsetof(cats_config_t, action_array[EV_MAX_V]),
     ev_servo_check},
    {"ev_apogee",
     VAR_INT16 | MODE_ARRAY,
     {.array = {.length = 16}},
     offsetof(cats_config_t, action_array[EV_APOGEE]),
     ev_servo_check},
    {"ev_main_deployment",
     VAR_INT16 | MODE_ARRAY,
     {.array = {.length = 16}},
     offsetof(cats_config_t, action_array[EV_MAIN_DEPLOYMENT]),
     ev_servo_check},
    {"ev_touchdown",
     VAR_INT16 | MODE_ARRAY,
     {.array = {.length = 16}},
     offsetof(cats_config_t, action_array[EV_TOUCHDOWN]),
     ev_servo_check},
    {"ev_custom1",
     VAR_INT16 | MODE_ARRAY,
     {.array = {.length = 16}},
     offsetof(cats_config_t, action_array[EV_CUSTOM_1]),
     ev_servo_check},
    {"ev_custom2",
     VAR_INT16 | MODE_ARRAY,
     {.array = {.length = 16}},
     offsetof(cats_config_t, action_array[EV_CUSTOM_2]),
     ev_servo_check},

    // Servo position
    {"servo1_init_pos", VAR_INT16, {.minmax_unsigned = {0, 1000}}, offsetof(cats_config_t, initial_servo_position[0])},
    {"servo2_init_pos", VAR_INT16, {.minmax_unsigned = {0, 1000}}, offsetof(cats_config_t, initial_servo_position[1])},

    {"tele_link_phrase",
     VAR_UINT8 | MODE_STRING,
     {.string = {kMinConnPhraseChars, kMaxConnPhraseChars}},
     offsetof(cats_config_t, telemetry_settings.link_phrase)},
    {"tele_test_phrase",
     VAR_UINT8 | MODE_STRING,
     {.string = {kMinConnPhraseChars, kMaxConnPhraseChars}},
     offsetof(cats_config_t, telemetry_settings.test_phrase)},
    {"tele_power_level",
     VAR_UINT8,
     {.minmax_unsigned = {16, 30}},
     offsetof(cats_config_t, telemetry_settings.power_level)},
    {"tele_enable",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_POWER}},
     offsetof(cats_config_t, telemetry_settings.enable_telemetry)},
    {"tele_adaptive_power",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_POWER}},
     offsetof(cats_config_t, telemetry_settings.adaptive_power)},

    // Canard control
    {"ctrl_layout",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_CTRL_LAYOUT}},
     offsetof(cats_config_t, canard.layout),
     ctrl_layout_cb},
    {"ctrl_flight_mode",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_CTRL_MODE}},
     offsetof(cats_config_t, canard.flight_mode),
     ctrl_reboot_note},
    {"ctrl_v_enable",
     VAR_UINT16,
     {.minmax_unsigned = {10, 100}},
     offsetof(cats_config_t, canard.v_enable_mps),
     ctrl_reboot_note},
    {"ctrl_v_ref",
     VAR_UINT16,
     {.minmax_unsigned = {20, 300}},
     offsetof(cats_config_t, canard.v_ref_mps),
     ctrl_reboot_note},
    {"ctrl_tilt_v_min",
     VAR_UINT16,
     {.minmax_unsigned = {20, 300}},
     offsetof(cats_config_t, canard.tilt_v_min_mps),
     ctrl_reboot_note},
    {"ctrl_max_defl",
     VAR_UINT8,
     {.minmax_unsigned = {0, kCtrlMaxDeflDdeg}},
     offsetof(cats_config_t, canard.max_defl_ddeg),
     ctrl_reboot_note},
    {"ctrl_abort_tilt",
     VAR_UINT8,
     {.minmax_unsigned = {5, 60}},
     offsetof(cats_config_t, canard.abort_tilt_deg),
     ctrl_reboot_note},
    {"ctrl_roll_kp",
     VAR_UINT16,
     {.minmax_unsigned = {0, 65535}},
     offsetof(cats_config_t, canard.roll_kp_milli),
     ctrl_reboot_note},
    {"ctrl_roll_ki",
     VAR_UINT16,
     {.minmax_unsigned = {0, 65535}},
     offsetof(cats_config_t, canard.roll_ki_milli),
     ctrl_reboot_note},
    {"ctrl_tilt_kp",
     VAR_UINT16,
     {.minmax_unsigned = {0, 65535}},
     offsetof(cats_config_t, canard.tilt_kp_milli),
     ctrl_reboot_note},
    {"ctrl_tilt_kd",
     VAR_UINT16,
     {.minmax_unsigned = {0, 65535}},
     offsetof(cats_config_t, canard.tilt_kd_milli),
     ctrl_reboot_note},
    {"ctrl_step_axis",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_CTRL_AXIS}},
     offsetof(cats_config_t, canard.step_axis),
     ctrl_step_axis_cb},
    {"ctrl_step_defl",
     VAR_UINT8,
     {.minmax_unsigned = {0, 100}},
     offsetof(cats_config_t, canard.step_defl_ddeg),
     ctrl_reboot_note},
    {"ctrl_step_ms",
     VAR_UINT16,
     {.minmax_unsigned = {50, 2000}},
     offsetof(cats_config_t, canard.step_ms),
     ctrl_reboot_note},
    {"ctrl_step_delay_ms",
     VAR_UINT16,
     {.minmax_unsigned = {0, 10000}},
     offsetof(cats_config_t, canard.step_delay_ms),
     ctrl_reboot_note},
    {"ctrl_step_count",
     VAR_UINT8,
     {.minmax_unsigned = {1, 10}},
     offsetof(cats_config_t, canard.step_count),
     ctrl_reboot_note},
    {"ctrl_step_period_ms",
     VAR_UINT16,
     {.minmax_unsigned = {200, 10000}},
     offsetof(cats_config_t, canard.step_period_ms),
     ctrl_reboot_note},
    {"ctrl_us_per_deg",
     VAR_UINT16,
     {.minmax_unsigned = {10, 1000}},
     offsetof(cats_config_t, canard.us_per_deg_deci),
     ctrl_reboot_note},
    {"ctrl_center_us",
     VAR_UINT16,
     {.minmax_unsigned = {1000, 2000}},
     offsetof(cats_config_t, canard.center_us),
     ctrl_reboot_note},
    {"ctrl_servo_hz",
     VAR_UINT16,
     {.minmax_unsigned = {50, 333}},
     offsetof(cats_config_t, canard.servo_hz),
     ctrl_reboot_note},
    {"ctrl_trim",
     VAR_INT16 | MODE_ARRAY,
     {.array = {.length = 4}},
     offsetof(cats_config_t, canard.trim_us),
     ctrl_reboot_note},
    {"ctrl_reverse",
     VAR_UINT8 | MODE_ARRAY,
     {.array = {.length = 4}},
     offsetof(cats_config_t, canard.reverse),
     ctrl_reboot_note},

    {"buzzer_volume", VAR_UINT8, {.minmax_unsigned = {0, 100}}, offsetof(cats_config_t, buzzer_volume)},
    {"battery_type", VAR_UINT8 | MODE_LOOKUP, {.lookup = {TABLE_BATTERY}}, offsetof(cats_config_t, battery_type)},

    {"rec_elements", VAR_UINT32, {.u32_max = UINT32_MAX}, offsetof(cats_config_t, rec_mask)},
    {"rec_speed", VAR_UINT8 | MODE_LOOKUP, {.lookup = {TABLE_SPEEDS}}, offsetof(cats_config_t, rec_speed_idx)},
    {"test_mode", VAR_UINT8 | MODE_LOOKUP, {.lookup = {TABLE_POWER}}, offsetof(cats_config_t, enable_testing_mode)},
};

const uint16_t value_table_entry_count = ARRAYLEN(value_table);

void *get_cats_config_member_ptr(const cats_config_t *cfg, const cli_value_t *var) {
  // NOLINTNEXTLINE(google-readability-casting)
  return ((uint8_t *)cfg) + var->member_offset;
}

void print_cats_config(const char *cmd_name, const cats_config_t *cfg, bool print_limits) {
  const char *prefix = "";
  if (strcmp(cmd_name, "dump") == 0) {
    prefix = "set ";
  }

  // NOLINTNEXTLINE(modernize-loop-convert)
  for (uint32_t i = 0; i < value_table_entry_count; i++) {
    const cli_value_t *val = &value_table[i];
    cli_printf("%s%s = ", prefix, value_table[i].name);
    cli_print_var(cmd_name, cfg, val, print_limits);
    cli_print_linefeed();
  }

  /* Print out the 'set_by_user' flag when requested by commands other than 'dump'. */
  if (strcmp(cmd_name, "dump") != 0) {
    cli_printf("set_by_user: %s", cfg->is_set_by_user ? "TRUE" : "FALSE");
  }
}
