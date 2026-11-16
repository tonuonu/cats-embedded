/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

namespace canard {

/// `ctrl_test live` duration when none is given, and the longest one the CLI allows, in s.
constexpr float kLiveTestDefaultS = 20.0F;
constexpr float kMaxTestS = 120.0F;

struct LiveTestArgs {
  /// Why the arguments were refused: kUsage for a malformed token (checked first, so "1e39x" is kUsage), a literal
  /// inf/nan or a value <= 0 (an underflow such as 1e-50 reads as 0, so kUsage); k*Range when the number overflows a
  /// float (e.g. 1e39 or -1e39). errno is not used.
  enum class Error : uint8_t { kNone, kUsage, kSpeedRange, kDurationRange };
  bool ok = false;  ///< false: print the error, start nothing
  Error error = Error::kUsage;
  float speed_mps = 0.0F;
  float duration_s = 0.0F;
  bool capped = false;  ///< the given duration was above kMaxTestS and was cut to it
};

/// Parses the arguments of `ctrl_test live`: "<speed_mps> [seconds]". The speed must be finite and > 0, the optional
/// duration finite and > 0 (strtof also accepts "inf" and "nan"; both are refused), with nothing after it. A duration
/// above kMaxTestS is capped to it. nullptr or an empty string is refused. A number that overflows a float is refused
/// with k*Range, so it is not mistaken for the cap.
LiveTestArgs ParseLiveTestArgs(const char* text);

}  // namespace canard
