// hal_timebase_tim5.c — HAL timebase redirected to TIM5.
//
// Overrides the __weak HAL_GetTick() and HAL_IncTick() supplied by
// stm32f4xx_hal.c so that all HAL timing reads the free-running TIM5
// hardware counter rather than a SysTick-driven software counter.
//
// Two-phase behaviour:
//   Early init  (before init_clocks() enables TIM5):
//     HAL_GetTick() falls back to uwTick, which SysTick still increments
//     via HAL_IncTick().  This keeps HAL_ETH_Init() and other peripheral
//     initialisers working correctly while TIM5 is not yet running.
//
//   Normal operation (after init_clocks() enables TIM5 and kills SysTick):
//     LL_TIM_IsEnabledCounter(TIM5) returns true, so HAL_GetTick() reads
//     the hardware counter and converts 100 µs ticks to milliseconds.
//     HAL_IncTick() continues to maintain uwTick for the rare case where
//     this override is bypassed, but SysTick no longer fires so it is
//     effectively a no-op.
//
// TIM5 resolution: 100 µs per tick (ONE_SECOND = 10 000 ticks/s).
// HAL_GetTick() contract: returns milliseconds → divide by 10.

#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_tim.h"

#define TIM5_TICKS_PER_MS   10U     // ONE_SECOND (10 000) / 1000

uint32_t HAL_GetTick(void)
{
    if (LL_TIM_IsEnabledCounter(TIM5))
        return (uint32_t)(LL_TIM_GetCounter(TIM5) / TIM5_TICKS_PER_MS);

    return uwTick;   // fallback: SysTick-driven during early init
}

void HAL_IncTick(void)
{
    uwTick += (uint32_t)uwTickFreq;   // keep fallback counter valid
}
