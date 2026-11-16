/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "control/canard_io.hpp"

#include "config/cats_config.hpp"
#include "control/canard/canard_types.hpp"

bool canard_outputs_claimed(uint8_t servo_index) {
  const auto layout = static_cast<canard::Layout>(global_cats_config.canard.layout);
  const bool canard_on = (layout == canard::Layout::kRoll2) || (layout == canard::Layout::kFull4);
  return canard_on && servo_index < 2U;
}
