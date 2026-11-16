/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

namespace canard {

/// `ctrl_test live` duration when none is given, and the longest one the CLI allows, in s.
constexpr float kLiveTestDefaultS = 20.0F;
constexpr float kMaxTestS = 120.0F;

struct LiveTestArgs {
  bool ok = false;  ///< false: print the usage, start nothing
  float speed_mps = 0.0F;
  float duration_s = 0.0F;
  bool capped = false;  ///< the given duration was above kMaxTestS and was cut to it
};

/// Parses the arguments of `ctrl_test live`: "<speed_mps> [seconds]". The speed must be finite and > 0, the optional
/// duration finite and > 0 (strtof also accepts "inf" and "nan"; both are refused), with nothing after it. A duration
/// above kMaxTestS is capped to it. nullptr or an empty string is refused.
LiveTestArgs ParseLiveTestArgs(const char* text);

}  // namespace canard
