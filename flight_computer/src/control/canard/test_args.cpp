/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "control/canard/test_args.hpp"

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
  a.speed_mps = std::strtof(text, &end);
  if (end == text || !FinitePositive(a.speed_mps)) {
    return a;
  }
  a.duration_s = kLiveTestDefaultS;
  if (*end != '\0') {
    const char* const start = end;
    char* end2 = nullptr;
    a.duration_s = std::strtof(start, &end2);
    if (end2 == start || !FinitePositive(a.duration_s) || *end2 != '\0') {
      return a;
    }
  }
  if (a.duration_s > kMaxTestS) {
    a.duration_s = kMaxTestS;
    a.capped = true;
  }
  a.ok = true;
  return a;
}

}  // namespace canard
