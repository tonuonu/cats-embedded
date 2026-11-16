/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

namespace canard {

/// Maps a board-frame vector to the body frame (x = t1, y = t2, z = roll axis).
/// long_axis is the board axis along the rocket, long_sign its direction (+1 or -1).
/// The mapping is a proper rotation for either sign, so handedness is preserved.
void BoardToBody(const float board[3], uint8_t long_axis, float long_sign, float body[3]);

}  // namespace canard
