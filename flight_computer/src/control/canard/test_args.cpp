/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "control/canard/test_args.hpp"

#include <cmath>
#include <cstdlib>

namespace canard {

namespace {

bool FinitePositive(float x) { return std::isfinite(x) && x > 0.0F; }

/// strtof overflow without errno (errno is shared between tasks here: no configUSE_NEWLIB_REENTRANT). Overflow returns
/// +-HUGE_VALF (inf); a literal "inf"/"nan" token is told apart by its letter n, which no numeric token contains.
bool Overflowed(float value, const char* begin, const char* end) {
  if (!std::isinf(value)) {
    return false;
  }
  for (const char* c = begin; c != end; ++c) {
    if (*c == 'n' || *c == 'N') {
      return false;
    }
  }
  return true;
}

}  // namespace

LiveTestArgs ParseLiveTestArgs(const char* text) {
  LiveTestArgs a;
  if (text == nullptr) {
    return a;
  }
  // Syntax first: each token must parse fully, so trailing garbage is a usage error even when the number would
  // overflow. Then range (out of range), then value (> 0, finite) checks.
  char* end = nullptr;
  a.speed_mps = std::strtof(text, &end);
  if (end == text) {
    return a;
  }
  const char* const speed_end = end;
  bool has_duration = false;
  const char* duration_begin = nullptr;
  char* duration_end = nullptr;
  if (*end != '\0') {
    duration_begin = end;
    a.duration_s = std::strtof(duration_begin, &duration_end);
    if (duration_end == duration_begin || *duration_end != '\0') {
      return a;
    }
    has_duration = true;
  }
  if (Overflowed(a.speed_mps, text, speed_end)) {
    a.error = LiveTestArgs::Error::kSpeedRange;
    return a;
  }
  if (!FinitePositive(a.speed_mps)) {
    return a;
  }
  if (has_duration) {
    if (Overflowed(a.duration_s, duration_begin, duration_end)) {
      a.error = LiveTestArgs::Error::kDurationRange;
      return a;
    }
    if (!FinitePositive(a.duration_s)) {
      return a;
    }
  } else {
    a.duration_s = kLiveTestDefaultS;
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
