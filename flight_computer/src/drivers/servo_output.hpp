/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "target.hpp"

namespace driver {

/** Servo output that writes the pulse width straight into the timer compare register.
 *
 * One timer tick is 1 us, so the compare value is the pulse width in microseconds. Unlike Pwm/Servo, a pulse update
 * does not reconfigure or restart the channel, so it can be called every control loop. The channel compare preload
 * stays enabled, so a new pulse takes effect at the next frame without glitches.
 *
 * Two outputs share one timer: Configure() re-initialises the whole timer, so configure every output on a timer
 * before starting any of them.
 */
class ServoOutput {
 public:
  /** Constructor
   *
   * @param timer Reference to the HAL timer @injected
   * @param channel Timer channel (TIM_CHANNEL_x)
   */
  ServoOutput(TIM_HandleTypeDef& timer, uint32_t channel) : m_timer(timer), m_channel(channel) {}

  /** Set the timer to 1 us per tick and the frame to 1 / frame_hz, and set the channel up as PWM mode 1.
   *
   * @param frame_hz servo frame rate in Hz, 50 - 333
   * @return false if frame_hz is out of range or the timer clock is not a whole number of MHz
   */
  bool Configure(uint16_t frame_hz);

  /** Start the pulse output. The first frame already carries the last SetPulseUs() value. */
  void Start();

  /** Set the pulse width; takes effect at the next frame.
   *
   * @param us pulse width in microseconds, must be shorter than the frame
   */
  void SetPulseUs(uint16_t us) { __HAL_TIM_SET_COMPARE(&m_timer, m_channel, us); }

 private:
  /// Reference to the timer
  TIM_HandleTypeDef& m_timer;
  /// Timer channel
  uint32_t m_channel;
};

}  // namespace driver

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
/// Canard servo outputs 0..3; nullptr when the layout does not use the output.
extern driver::ServoOutput* global_canard_outputs[4];
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)
