// button.c — B1 user button (PA0) — start/stop accelerometer toggle.
//
// Hardware:
//   PA0 is shorted to VDD (3.3 V) when B1 is pressed; an on-board resistor
//   holds it low when released.  MX_GPIO_Init() configures PA0 as input /
//   no-pull and routes EXTI line 0 to PORTA.  button_init() promotes EXTI0
//   from EVENT to IT mode.
//
// State machine — one flag, one function:
//
//   IDLE (btn_active = false, EXTI0 armed):
//     Rising edge → EXTI0_IRQHandler → button_exti0_isr():
//       • HAL_NVIC_DisableIRQ(EXTI0_IRQn)  — stops all further edges now
//       • btn_active = true
//       • later(button_service)             — enter event loop
//     → DEBOUNCING
//
//   DEBOUNCING (btn_active = true, EXTI0 disarmed):
//     button_service() pattern:
//       if (btn_active) {
//           after(msec(500), button_service);   // reschedule
//           if (PA0 == 0) {                      // button released?
//               btn_active = false;              // stop the machine
//               toggle accel;
//               clear pending flags;
//               re-enable EXTI0;                 // → IDLE
//           }
//       }
//     While the button is held the function keeps rescheduling itself every
//     500 ms without acting.  Once PA0 reads low the toggle fires and EXTI0
//     is re-armed in the same event.  The one already-scheduled call that
//     follows will find btn_active = false and return immediately.
//
// EXTI0 conflict (production LIS3DSH only):
//   accel_start() re-routes EXTI0 to PE0 (MEMS INT1); accel_stop() re-routes
//   back to PA0.  EXTI0_IRQHandler dispatches by reading SYSCFG EXTICR1.

#include "button.h"
#include "accel.h"
#include "tea.h"
#include "main.h"                       // HAL, CMSIS, EXTI0_IRQn
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_gpio.h"

static bool btn_active = false;
static void button_service(void);

void button_init(void)
{
    // EXTI0 is already mapped to PA0 by MX_GPIO_Init(); promote to IT mode.
    LL_EXTI_InitTypeDef exti = {0};
    exti.Line_0_31   = LL_EXTI_LINE_0;
    exti.LineCommand = ENABLE;
    exti.Mode        = LL_EXTI_MODE_IT;
    exti.Trigger     = LL_EXTI_TRIGGER_RISING;
    LL_EXTI_Init(&exti);

    HAL_NVIC_SetPriority(EXTI0_IRQn, 6, 0);   // lower priority than MEMS (5)
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    namedAction(button_service);
}

// Called from EXTI0_IRQHandler (ISR context) when EXTI0 is routed to PA0.
// Does the minimum: disable EXTI, set flag, hand off to event loop.
void button_exti0_isr(void)
{
    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
    btn_active = true;
    later(button_service);
}

// Event-loop function.  Reschedules itself while the button is held;
// acts and re-arms as soon as PA0 returns to zero.
static void button_service(void)
{
    if (!btn_active) return;

    after(msec(500), button_service);              // reschedule

    if (!(LL_GPIO_ReadInputPort(GPIOA) & LL_GPIO_PIN_0)) {   // PA0 low = released
        btn_active = false;                        // stop the machine
        if (accel_is_running()) accel_stop();
        else                    accel_start();
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);   // clear edge that fired while armed
        HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);     // clear NVIC latch
        HAL_NVIC_EnableIRQ(EXTI0_IRQn);           // re-arm → IDLE
    }
}
