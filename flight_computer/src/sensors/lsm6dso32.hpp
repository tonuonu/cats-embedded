/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "drivers/gpio.hpp"
#include "drivers/spi.hpp"
#include "util/log.h"

#include <cstdint>
#include <cstring>
#include <vector>

namespace sensor {

/// FIFO sample data structure
struct FifoSample {
  int16_t x;
  int16_t y;
  int16_t z;
  uint8_t tag;  ///< 0x01 = gyro, 0x02 = accel
};

/// FIFO tag values for sample identification
enum class FifoTag : uint8_t {
  kGyroscope = 0x01,
  kAccelerometer = 0x02,
};

class Lsm6dso32 {
 public:
  /// Sample period in microseconds at 104Hz ODR (1/104 * 1e6)
  static constexpr uint32_t kSamplePeriodUs = 9615;

  /** Constructor
   *
   * @param spi reference the the SPI interface @injected
   * @param cs reference the the chip select output pin @injected
   */
  Lsm6dso32(driver::Spi& spi, driver::OutputPin& cs) : m_spi{spi}, m_cs{cs} { m_cs.SetHigh(); }

  /** Initialize the sensor
   *
   * @return true on success
   */
  [[nodiscard]] bool Init() {
    uint8_t temp = 0U;
    // First check the WHO AM I register to determine if sensor responds
    ReadRegister(static_cast<uint8_t>(Register::kWhoAmI), &temp, 1U);
    if (temp != static_cast<uint8_t>(0x6CU)) {
      return false;
    }

    // Configure Accelerometer
    temp = static_cast<uint8_t>(ImuOdr::kOdr104Hz) | static_cast<uint8_t>(AccelerometerFs::kFs32G);
    WriteRegister(static_cast<uint8_t>(Register::kCtrl1Xl), &temp, 1U);

    // Configure Gyroscope
    temp = static_cast<uint8_t>(ImuOdr::kOdr104Hz) | static_cast<uint8_t>(GyroscopeFs::kFs2000Dps);
    WriteRegister(static_cast<uint8_t>(Register::kCtrl2G), &temp, 1U);

    return true;
  }

  /** Configure FIFO for continuous mode sampling
   *
   * Enables hardware-precise 104Hz sampling with FIFO buffering.
   * Both accelerometer and gyroscope data are batched into FIFO.
   *
   * @param watermark Number of samples before watermark flag is set (1-511)
   * @return true on success
   */
  [[nodiscard]] bool ConfigureFifo(uint16_t watermark = 1) {
    uint8_t temp = 0U;

    // Clamp watermark to valid range (1-511, 9 bits)
    if (watermark == 0) {
      watermark = 1;
    }
    if (watermark > 511) {
      watermark = 511;
    }

    // FIFO_CTRL1: Watermark threshold bits [7:0]
    temp = static_cast<uint8_t>(watermark & 0xFFU);
    WriteRegister(static_cast<uint8_t>(Register::kFifoCtrl1), &temp, 1U);

    // FIFO_CTRL2: Watermark threshold bit [8] (bit 0), STOP_ON_WTM disabled
    temp = static_cast<uint8_t>((watermark >> 8U) & 0x01U);
    WriteRegister(static_cast<uint8_t>(Register::kFifoCtrl2), &temp, 1U);

    // FIFO_CTRL3: Batch data rates - 104Hz for both accel and gyro
    // BDR_GY[3:0] in bits [7:4], BDR_XL[3:0] in bits [3:0]
    // 0100b = 104Hz for both
    temp = (static_cast<uint8_t>(FifoBdr::kBdr104Hz) << 4U) | static_cast<uint8_t>(FifoBdr::kBdr104Hz);
    WriteRegister(static_cast<uint8_t>(Register::kFifoCtrl3), &temp, 1U);

    // FIFO_CTRL4: FIFO mode = Continuous (110b = 0x06)
    temp = static_cast<uint8_t>(FifoMode::kContinuous);
    WriteRegister(static_cast<uint8_t>(Register::kFifoCtrl4), &temp, 1U);

    return true;
  }

  /** Disable FIFO and return to bypass mode */
  void DisableFifo() {
    uint8_t temp = static_cast<uint8_t>(FifoMode::kBypass);
    WriteRegister(static_cast<uint8_t>(Register::kFifoCtrl4), &temp, 1U);
  }

  /** Get number of unread samples in FIFO
   *
   * @return Number of 7-byte words (tag + 6 data bytes) available in FIFO
   */
  [[nodiscard]] uint16_t GetFifoSampleCount() {
    uint8_t status[2] = {0U, 0U};
    ReadRegister(static_cast<uint8_t>(Register::kFifoStatus1), status, 2U);
    // DIFF_FIFO[9:0]: bits [7:0] in STATUS1, bits [9:8] in STATUS2[1:0]
    return (static_cast<uint16_t>(status[1] & 0x03U) << 8U) | static_cast<uint16_t>(status[0]);
  }

  /** Check if FIFO overrun has occurred
   *
   * @return true if FIFO has overrun (data loss)
   */
  [[nodiscard]] bool IsFifoOverrun() {
    uint8_t status2 = 0U;
    ReadRegister(static_cast<uint8_t>(Register::kFifoStatus2), &status2, 1U);
    // FIFO_OVR_IA is bit 6
    return (status2 & 0x40U) != 0U;
  }

  /** Read one sample from FIFO
   *
   * @param sample Output structure to store the sample
   * @return true if sample was read successfully
   */
  [[nodiscard]] bool ReadFifoSample(FifoSample& sample) {
    uint8_t data[7] = {0U};

    // Read tag byte and 6 data bytes
    ReadRegister(static_cast<uint8_t>(Register::kFifoDataOutTag), data, 7U);

    // Extract tag from bits [7:3] of tag byte
    sample.tag = (data[0] >> 3U) & 0x1FU;

    // Extract XYZ data (little-endian)
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    sample.x = *reinterpret_cast<int16_t*>(&data[1]);
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    sample.y = *reinterpret_cast<int16_t*>(&data[3]);
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    sample.z = *reinterpret_cast<int16_t*>(&data[5]);

    return true;
  }

  /** Read raw gyroscope data from the sensor (direct register read, bypasses FIFO)
   *
   * @param data pointer to write the raw data to, needs to be of size 3!
   */
  void ReadGyroRaw(int16_t* data) {
    // The bit representation inside the imu is exactly the same order as the int16 data register, therefore we want to
    // reinterpret cast the data
    // NOLINTNEXTLINE
    ReadRegister(static_cast<uint8_t>(Register::kOutXLG), reinterpret_cast<uint8_t*>(data), 6U);
  }

  /** Read raw accelerometer data from the sensor (direct register read, bypasses FIFO)
   *
   * @param data pointer to write the raw data to, needs to be of size 3!
   */
  void ReadAccelRaw(int16_t* data) {
    // The bit representation inside the imu is exactly the same order as the int16 data register, therefore we want to
    // reinterpret cast the data
    // NOLINTNEXTLINE
    ReadRegister(static_cast<uint8_t>(Register::kOutXLA), reinterpret_cast<uint8_t*>(data), 6U);
  }

 private:
  /** Read data from a register of the IMU
   *
   * @param reg register to read
   * @param data pointer to data to store
   * @param length length of read data
   */
  void ReadRegister(const uint8_t reg, uint8_t* const data, const size_t length) {
    // Set the read flag of the register
    uint8_t read_reg = reg | static_cast<uint8_t>(0x80U);
    // Read from the sensor
    m_cs.SetLow();
    m_spi.Transfer(&read_reg, 1U);
    m_spi.Receive(data, length);
    m_cs.SetHigh();
  }

  /** Write to a data register of the IMU
   *
   * @param reg register to write
   * @param data data to write
   * @param length length of write data
   */
  void WriteRegister(const uint8_t reg, const uint8_t* data, const size_t length) {
    // Concatenate the register and the data
    // NOLINTNEXTLINE(cppcoreguidelines-init-variables) linter is confused
    std::vector<uint8_t> concatenated{};
    concatenated.push_back(reg);
    concatenated.insert(concatenated.end(), data, data + length);
    // Transfer the data
    m_cs.SetLow();
    m_spi.Transfer(concatenated.data(), length + 1U);
    m_cs.SetHigh();
  }

  /// Scoped sensor register enum
  enum class Register : uint8_t {
    kFifoCtrl1 = 0x07,
    kFifoCtrl2 = 0x08,
    kFifoCtrl3 = 0x09,
    kFifoCtrl4 = 0x0A,
    kWhoAmI = 0x0F,
    kCtrl1Xl = 0x10,
    kCtrl2G = 0x11,
    kOutXLG = 0x22,
    kOutXLA = 0x28,
    kFifoStatus1 = 0x3A,
    kFifoStatus2 = 0x3B,
    kFifoDataOutTag = 0x78,
  };

  /// Scoped sensor output data rate enum
  enum class ImuOdr : uint8_t {
    kOdr1Hz6 = 0xB0,
    kOdr12Hz5 = 0x10,
    kOdr26Hz = 0x20,
    kOdr52Hz = 0x30,
    kOdr104Hz = 0x40,
    kOdr208Hz = 0x50,
    kOdr416Hz = 0x60,
    kOdr833Hz = 0x70,
    kOdr1kHz66 = 0x80,
    kOdr3kHz33 = 0x90,
    kOdr6kHz66 = 0xA0,
  };

  /// FIFO batch data rate enum
  enum class FifoBdr : uint8_t {
    kBdrNotBatched = 0x00,
    kBdr12Hz5 = 0x01,
    kBdr26Hz = 0x02,
    kBdr52Hz = 0x03,
    kBdr104Hz = 0x04,
    kBdr208Hz = 0x05,
    kBdr416Hz = 0x06,
    kBdr833Hz = 0x07,
  };

  /// FIFO mode enum
  enum class FifoMode : uint8_t {
    kBypass = 0x00,
    kFifoMode = 0x01,
    kContinuousToFifo = 0x03,
    kBypassToContinuous = 0x04,
    kContinuous = 0x06,
    kBypassToFifo = 0x07,
  };

  /// Scoped accelerometer full scale enum
  enum class AccelerometerFs : uint8_t {
    kFs4G = 0x00,
    kFs8G = 0x08,
    kFs16G = 0x0C,
    kFs32G = 0x04,
  };

  /// Scoped gyroscope full scale enum
  enum class GyroscopeFs : uint8_t {
    kFs250Dps = 0x00,
    kFs500Dps = 0x04,
    kFs1000Dps = 0x08,
    kFs2000Dps = 0x0C,
  };

  /// Reference to the spi interface
  driver::Spi& m_spi;
  /// Reference to the chip select pin
  driver::OutputPin& m_cs;
};

}  // namespace sensor
