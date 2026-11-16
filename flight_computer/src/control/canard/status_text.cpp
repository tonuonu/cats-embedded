/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "control/canard/status_text.hpp"

namespace canard {

const char* LoggedReasonNote(const LoggedReasonView& v) {
  if (!v.started) {
    return " (control not started, nothing recorded)";
  }
  if (v.logged_reason == v.envelope_reason) {
    return "";
  }
  if (!v.pad_sample_seen) {
    return " (no pad samples)";
  }
  if (!v.frame_ok) {
    return v.on_pad ? " (frame check fails; latches at activation)" : " (frame check failed)";
  }
  return "";
}

}  // namespace canard
