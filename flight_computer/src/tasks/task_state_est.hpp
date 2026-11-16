/// Copyright (C) 2020, 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "control/canard_io.hpp"
#include "control/kalman_filter.hpp"
#include "control/orientation_filter.hpp"
#include "task.hpp"
#include "task_preprocessing.hpp"
#include "tasks/task_preprocessing.hpp"
#include "util/error_handler.hpp"
#include "util/log.h"
#include "util/types.hpp"

#include <atomic>

namespace task {

class StateEstimation;
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
extern StateEstimation* global_state_estimation;
/// The canard glue of the running StateEstimation; nullptr while the boot layout is OFF (set in Run, after Init).
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
extern CanardIo* global_canard_io;

/* Stack size in words (uint32_t) */
inline constexpr uint32_t kStateEstStackWords = 512;

class StateEstimation final : public Task<StateEstimation, kStateEstStackWords> {
 public:
  explicit StateEstimation(const Preprocessing& task_preprocessing)
      : m_task_preprocessing{task_preprocessing},
        m_filter{.t_sampl = 1.0F / static_cast<float>(CONTROL_SAMPLING_FREQ)},
        m_orientation_filter{} {
    global_state_estimation = this;
  }
  [[nodiscard]] estimation_output_t GetEstimationOutput() const noexcept;

 private:
  [[noreturn]] void Run() noexcept override;

  void GetEstimationInputData();

  const Preprocessing& m_task_preprocessing;

  /* Initialize State Estimation */
  kalman_filter_t m_filter;
  orientation_filter_t m_orientation_filter;

  /* Canard control, stepped after every Kalman step while the boot layout is not OFF */
  CanardIo m_canard{};
};

}  // namespace task
