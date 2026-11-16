/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

/** True when the canard layout owns event servo output servo_index (0 = SERVO1, 1 = SERVO2).
 *
 * ROLL_2 and FULL_4 claim both; OFF (and any unknown layout value) claims none, so the event actions keep them.
 */
bool canard_outputs_claimed(uint8_t servo_index);
