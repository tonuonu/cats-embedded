/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "control/canard/test_args.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>

namespace canard {

namespace {
bool FinitePositive(float x) { return std::isfinite(x) && x > 0.0F; }
}  // namespace

LiveTestArgs ParseLiveTestArgs(const char* text) {
  LiveTestArgs a;
  if (text == nullptr) {
    return a;
  }
  char* end = nullptr;
  errno = 0;
  a.speed_mps = std::strtof(text, &end);
  if (end != text && errno == ERANGE) {
    a.error = LiveTestArgs::Error::kSpeedRange;
    return a;
  }
  if (end == text || !FinitePositive(a.speed_mps)) {
    return a;
  }
  a.duration_s = kLiveTestDefaultS;
  if (*end != '\0') {
    const char* const start = end;
    char* end2 = nullptr;
    errno = 0;
    a.duration_s = std::strtof(start, &end2);
    if (end2 != start && errno == ERANGE) {
      a.error = LiveTestArgs::Error::kDurationRange;
      return a;
    }
    if (end2 == start || !FinitePositive(a.duration_s) || *end2 != '\0') {
      return a;
    }
  }
  if (a.duration_s > kMaxTestS) {
    a.duration_s = kMaxTestS;
    a.capped = true;
  }
  a.ok = true;
  a.error = LiveTestArgs::Error::kNone;
  return a;
}

}  // namespace canard
