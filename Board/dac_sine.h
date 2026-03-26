/**
 * @file    dac_sine.h
 * @brief   DAC sine wave generator for STM32F4DISCOVERY (STM32F407VGT6)
 *
 * Outputs a sine wave on PA4 (DAC1_OUT) using TIMx -> DAC -> DMA.
 * The CPU is completely free after initialisation; DMA refills the DAC
 * autonomously in circular mode.
 *
 * Supported frequency range : 1 Hz – 100 kHz
 * DAC resolution             : 12-bit (4096 levels)
 * Output pin                 : PA4  (DAC Channel 1, no AF needed – analog mode)
 * Timer                      : Selectable via DAC_SINE_TIMER (default: TIM6)
 * DMA                        : DMA1 Stream 5 Channel 7 → DAC1 DHR12R1
 *
 * Timer selection – set DAC_SINE_TIMER in your Makefile / project defines,
 * or edit the default below.  All three options are on APB1 (84 MHz clock).
 *
 *   DAC_SINE_TIMER  Timer   TSEL[2:0]   Notes
 *   ─────────────── ─────── ──────────  ─────────────────────────────────────
 *        6          TIM6      000       Basic timer, purpose-built for DAC ← default
 *        7          TIM7      010       Basic timer, identical to TIM6
 *        4          TIM4      101       General-purpose; has 4 capture/compare channels
 *
 * All three are 16-bit timers.  A PSC + ARR pair is computed automatically
 * so the full 1 Hz – 100 kHz range is covered without overflow.
 *
 * Assumed clocks (standard STM32F4DISCOVERY CubeMX / SystemInit config):
 *   HSE      = 8 MHz  (on-board oscillator)
 *   SYSCLK   = 168 MHz  (via PLL)
 *   AHB      = 168 MHz
 *   APB1     = 42 MHz   → APB1 timer clock = 84 MHz  (×2 multiplier active)
 *   APB2     = 84 MHz
 *
 * Output quality notes:
 *   The number of samples per period is chosen automatically so that the
 *   DAC update rate never exceeds 1 MHz (DAC settling time ~1 µs).
 *   At 100 kHz this yields 10 samples/period; a simple RC low-pass filter
 *   (e.g. 1 kΩ + 1.5 nF, fc ≈ 100 kHz) on PA4 will remove DAC staircase
 *   harmonics and give a much cleaner sine shape at high frequencies.
 */

#ifndef DAC_SINE_H
#define DAC_SINE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialise all peripherals and start the sine wave output.
 *
 * Call once before any other function.  Configures GPIO, DMA, DAC, and
 * the selected timer.  Does NOT reconfigure the system clock – call
 * SystemInit() or your CubeMX HAL_Init() / SystemClock_Config() first.
 *
 * @param  freq_hz  Desired output frequency in Hz (clamped to 1 – 100 000).
 */
void DAC_Sine_Init(uint32_t freq_hz);

/**
 * @brief  Change the output frequency while running.
 *
 * The update is glitch-minimised: the timer and DMA are stopped for the
 * shortest possible window while the new sample buffer and timer period
 * are loaded, then restarted.
 *
 * @param  freq_hz  New frequency in Hz (clamped to 1 – 100 000).
 */
void DAC_Sine_SetFreq(uint32_t freq_hz);

/**
 * @brief  Stop the sine wave output (timer and DMA disabled, DAC held at 0 V).
 */
void DAC_Sine_Stop(void);

/**
 * @brief  Resume after DAC_Sine_Stop() using the last configured frequency.
 */
void DAC_Sine_Start(void);

#ifdef __cplusplus
}
#endif

#endif /* DAC_SINE_H */
