/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "task.hpp"

#include "sensors/lsm6dso32.hpp"
#include "sensors/ms5607.hpp"
#include "util/log.h"
#include "util/types.hpp"

namespace task {

class SensorRead final : public Task<SensorRead, 512> {
 public:
  SensorRead() = default;
  explicit SensorRead(sensor::Lsm6dso32* imu, sensor::Ms5607* barometer) : m_imu(imu), m_barometer(barometer) {}

  [[nodiscard]] baro_data_t GetBaro(uint8_t index) const noexcept;
  [[nodiscard]] imu_data_t GetImu(uint8_t index) const noexcept;

 private:
  [[noreturn]] void Run() noexcept override;

  /** Read all available samples from IMU FIFO and record them
   *
   * @param base_tick_count Current RTOS tick for timestamp derivation
   */
  void ReadImuFifo(uint32_t base_tick_count) noexcept;

  enum class BaroReadoutType {
    kReadBaroTemperature = 1,
    kReadBaroPressure = 2,
  };

  sensor::Lsm6dso32* m_imu{nullptr};
  sensor::Ms5607* m_barometer{nullptr};

  imu_data_t m_imu_data[NUM_IMU]{};
  baro_data_t m_baro_data[NUM_BARO]{};
  BaroReadoutType m_current_readout{BaroReadoutType::kReadBaroTemperature};

  /// FIFO enabled flag
  bool m_fifo_enabled{false};
  /// Sample counter for timestamp derivation within a batch read
  uint32_t m_fifo_sample_counter{0};
};

}  // namespace task
