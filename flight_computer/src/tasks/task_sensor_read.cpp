/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "tasks/task_sensor_read.hpp"
#include "cmsis_os.h"
#include "config/globals.hpp"
#include "flash/recorder.hpp"

#include "sensors/ms5607.hpp"
#include "util/log.h"
#include "util/task_util.hpp"

/** Private Function Declarations **/

namespace task {

baro_data_t SensorRead::GetBaro(uint8_t index) const noexcept { return m_baro_data[index]; }

imu_data_t SensorRead::GetImu(uint8_t index) const noexcept { return m_imu_data[index]; }

void SensorRead::ReadImuFifo(uint32_t base_tick_count) noexcept {
  if (!m_fifo_enabled || m_imu == nullptr) {
    return;
  }

  // Check for FIFO overrun (data loss)
  if (m_imu->IsFifoOverrun()) {
    log_error("IMU FIFO overrun detected");
  }

  // Get number of available samples
  uint16_t sample_count = m_imu->GetFifoSampleCount();

  // Track samples for this batch for timestamp derivation
  uint32_t accel_sample_idx = 0;
  uint32_t gyro_sample_idx = 0;

  // Read all available FIFO samples
  while (sample_count > 0) {
    sensor::FifoSample sample{};
    if (!m_imu->ReadFifoSample(sample)) {
      break;
    }

    // Process based on tag
    if (sample.tag == static_cast<uint8_t>(sensor::FifoTag::kAccelerometer)) {
      // Store accelerometer data
      m_imu_data[0].acc.x = sample.x;
      m_imu_data[0].acc.y = sample.y;
      m_imu_data[0].acc.z = sample.z;

      // Derive timestamp: base + sample_index * 9615µs (converted to ms ticks)
      // Note: RTOS tick is 1ms, so we calculate ms offset
      uint32_t derived_tick = base_tick_count - (sample_count * sensor::Lsm6dso32::kSamplePeriodUs / 1000U);

      // Record with derived timestamp
      record(derived_tick, add_id_to_record_type(IMU, 0), &(m_imu_data[0]));
      accel_sample_idx++;

    } else if (sample.tag == static_cast<uint8_t>(sensor::FifoTag::kGyroscope)) {
      // Store gyroscope data
      m_imu_data[0].gyro.x = sample.x;
      m_imu_data[0].gyro.y = sample.y;
      m_imu_data[0].gyro.z = sample.z;
      gyro_sample_idx++;
    }

    sample_count--;
  }

  m_fifo_sample_counter += accel_sample_idx;
}

/** Exported Function Definitions **/

/**
 * @brief Function implementing the task_sens_read thread.
 * @param argument: Not used
 * @retval None
 */
[[noreturn]] void SensorRead::Run() noexcept {
  /* Initialize IMU data variables */
  m_barometer->Prepare(sensor::Ms5607::Request::kTemperature);
  osDelay(5);

  /* Configure IMU FIFO for hardware-precise 104Hz sampling */
  if (m_imu != nullptr && imu_initialized[0]) {
    m_fifo_enabled = m_imu->ConfigureFifo(1);  // Watermark = 1 sample
    if (m_fifo_enabled) {
      log_info("IMU FIFO enabled for 104Hz sampling");
    } else {
      log_error("Failed to configure IMU FIFO, falling back to direct reads");
    }
  }

  uint32_t tick_count = osKernelGetTickCount();
  /* This task is sampled with 2 times the control sampling frequency to maximize speed of the barometer. In one
   * timestep the Baro pressure is read out and then the Baro Temperature. The other sensors are only read out one in
   * two times. */
  constexpr uint32_t tick_update = sysGetTickFreq() / static_cast<uint32_t>(2 * CONTROL_SAMPLING_FREQ);
  while (true) {
    // Readout the baro register
    m_barometer->Read();

    // Always read IMU FIFO when enabled (every 5ms iteration)
    // FIFO buffers samples at 104Hz, we read them out here
    if (m_fifo_enabled) {
      ReadImuFifo(tick_count);
    }

    // Prepare new readout for the baro
    if (m_current_readout == SensorRead::BaroReadoutType::kReadBaroPressure) {
      m_barometer->Prepare(sensor::Ms5607::Request::kPressure);
      m_current_readout = SensorRead::BaroReadoutType::kReadBaroTemperature;
    } else {
      m_barometer->Prepare(sensor::Ms5607::Request::kTemperature);
      m_current_readout = SensorRead::BaroReadoutType::kReadBaroPressure;
      /* For Simulator */
      if (simulation_started) {
        for (int i = 0; i < NUM_BARO; i++) {
          m_baro_data[i].pressure = global_baro_sim[i].pressure;
        }
      } else {
        m_barometer->GetMeasurement(m_baro_data[0].pressure, m_baro_data[0].temperature);
      }

      /* Save Barometric Data */
      for (int i = 0; i < NUM_BARO; i++) {
        record(tick_count, add_id_to_record_type(BARO, i), &(m_baro_data[0]));
      }

      /* Read and Save IMU Data - fallback for non-FIFO mode or simulation */
      if (!m_fifo_enabled) {
        for (int i = 0; i < NUM_IMU; i++) {
          if (simulation_started) {
            m_imu_data[i].acc = global_imu_sim[i].acc;
          } else {
            if (imu_initialized[i]) {
              // NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)
              m_imu->ReadGyroRaw(reinterpret_cast<int16_t*>(&m_imu_data[i].gyro));
              m_imu->ReadAccelRaw(reinterpret_cast<int16_t*>(&m_imu_data[i].acc));
              // NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)
            }
          }
          record(tick_count, add_id_to_record_type(IMU, i), &(m_imu_data[i]));
        }
      }
    }

    tick_count += tick_update;
    osDelayUntil(tick_count);
  }
}

}  // namespace task
