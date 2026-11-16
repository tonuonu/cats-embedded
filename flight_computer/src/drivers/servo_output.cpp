/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "drivers/servo_output.hpp"

namespace {

constexpr uint32_t kTickHz = 1'000'000U;
constexpr uint16_t kMinFrameHz = 50U;
constexpr uint16_t kMaxFrameHz = 333U;

/** Actual counter clock of a timer: its APB clock, doubled when that APB prescaler is not 1.
 *
 * @return clock in Hz, 0 if the timer clock prescaler (TIMPRE) is set, which this firmware never does
 */
uint32_t TimerClockHz(const TIM_TypeDef* instance) {
  if ((RCC->DCKCFGR & RCC_DCKCFGR_TIMPRE) != 0U) {
    return 0U;
  }
  // TIM1, TIM9, TIM10 and TIM11 are on APB2, the others on APB1.
  const bool apb2 = (instance == TIM1) || (instance == TIM9) || (instance == TIM10) || (instance == TIM11);
  const uint32_t pclk = apb2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
  const uint32_t ppre = RCC->CFGR & (apb2 ? RCC_CFGR_PPRE2 : RCC_CFGR_PPRE1);
  return (ppre == 0U) ? pclk : 2U * pclk;
}

}  // namespace

namespace driver {

bool ServoOutput::Configure(uint16_t frame_hz) {
  if (frame_hz < kMinFrameHz || frame_hz > kMaxFrameHz) {
    return false;
  }

  const uint32_t timer_hz = TimerClockHz(m_timer.Instance);
  if (timer_hz == 0U || timer_hz % kTickHz != 0U) {
    return false;
  }

  // 1 tick = 1 us. 50 Hz -> ARR 19999, 333 Hz -> ARR 3002; both fit a 16-bit timer.
  m_timer.Init.Prescaler = (timer_hz / kTickHz) - 1U;
  m_timer.Init.Period = (kTickHz / frame_hz) - 1U;
  m_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
  m_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  m_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&m_timer) != HAL_OK) {
    return false;
  }

  TIM_OC_InitTypeDef config_oc = {};
  config_oc.OCMode = TIM_OCMODE_PWM1;
  config_oc.Pulse = 0U;
  config_oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  config_oc.OCFastMode = TIM_OCFAST_DISABLE;
  return HAL_TIM_PWM_ConfigChannel(&m_timer, &config_oc, m_channel) == HAL_OK;
}

void ServoOutput::Start() {
  // The compare register is preloaded: while the counter is still stopped, load the pending SetPulseUs() value now
  // so the first frame already has it. Once the timer runs, an update event would cut a sibling channel's frame.
  if ((m_timer.Instance->CR1 & TIM_CR1_CEN) == 0U) {
    HAL_TIM_GenerateEvent(&m_timer, TIM_EVENTSOURCE_UPDATE);
  }
  HAL_TIM_PWM_Start(&m_timer, m_channel);
}

}  // namespace driver
