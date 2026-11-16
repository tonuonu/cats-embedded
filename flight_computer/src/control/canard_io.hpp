/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

/** Snapshot global_cats_config.canard.layout once, right after the config loads. Any value other than ROLL_2 or
 * FULL_4 is stored as OFF. The output setup is decided from this snapshot, so a later CLI `set ctrl_layout` only takes
 * effect after save + reboot.
 */
void canard_latch_boot_layout();

/** The layout latched at boot (a canard::Layout value); OFF until canard_latch_boot_layout() runs. */
uint8_t canard_boot_layout();

/** True when the boot layout owns event servo output servo_index (0 = SERVO1, 1 = SERVO2).
 *
 * ROLL_2 and FULL_4 claim both; OFF claims none, so the event actions keep them.
 */
bool canard_outputs_claimed(uint8_t servo_index);

/** Trim of canard output k (0..3) in us, clamped to +-300 us; 0 for k > 3. */
int16_t canard_trim_us(uint8_t k);

/** Reverse flag of canard output k (0..3): the configured value != 0; false for k > 3. */
bool canard_reverse(uint8_t k);
