/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "control/canard/frame.hpp"

namespace canard {

void BoardToBody(const float board[3], uint8_t long_axis, float long_sign, float body[3]) {
  const uint8_t a = long_axis % 3U;
  body[2] = long_sign * board[a];
  body[0] = board[(a + 1U) % 3U];
  body[1] = long_sign * board[(a + 2U) % 3U];
}

}  // namespace canard
