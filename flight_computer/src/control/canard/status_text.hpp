/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "control/canard/canard_types.hpp"

namespace canard {

/// What `ctrl_test status` knows when it prints the CONTROL_INFO ("logged") reason.
struct LoggedReasonView {
  bool started = false;          ///< control has started (nothing is recorded before)
  bool pad_sample_seen = false;  ///< the frame check ran at least once
  bool frame_ok = false;
  bool on_pad = false;  ///< the last control step was in CALIBRATING/READY
  LatchReason envelope_reason = LatchReason::kNone;
  LatchReason logged_reason = LatchReason::kNone;
};

/// The note after the logged reason: why it differs from the envelope's reason, keyed on the actual cause. Empty when
/// both agree. Never promises a latch off the pad, where no activation may follow.
const char* LoggedReasonNote(const LoggedReasonView& v);

}  // namespace canard
