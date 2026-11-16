/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "control/canard_io.hpp"

#include "config/cats_config.hpp"
#include "control/canard/canard_types.hpp"

namespace {

constexpr uint8_t kNumOutputs = 4U;
constexpr int16_t kMaxTrimUs = 300;

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables) written once at boot
uint8_t boot_layout = static_cast<uint8_t>(canard::Layout::kOff);

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
