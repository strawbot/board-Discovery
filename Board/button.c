// button.c — B1 user button (PA0) — start/stop accelerometer toggle.
//
// Hardware:
//   PA0 is shorted to VDD (3.3 V) when B1 is pressed; an on-board resistor
//   holds it low when released.  MX_GPIO_Init() already configures PA0 as
//   input / no-pull and routes EXTI line 0 to PORTA via SYSCFG EXTICR1.
//   MX_GPIO_Init() leaves EXTI0 in EVENT mode; button_init() promotes it to
//   IT mode to enable the interrupt.
//
// Interrupt path:
//   Rising edge on PA0 → EXTI0_IRQHandler → button_exti0_isr() →
//   timestamp debounce → later(button_toggle) → event loop.
//
// Debounce:
//   button_exti0_isr() records the tick of the first rising edge and ignores
//   any further edges within BUTTON_DEBOUNCE_MS.  Because EXTI is configured
//   for rising edge only, falling-edge bounce is invisible.  With a 50 ms
//   window, the button action fires immediately on the first press and further
//   edges within the bounce window are discarded.
//
// EXTI0 conflict (production LIS3DSH only):
//   EXTI line 0 can only be connected to one GPIO port at a time.  When the
//   production LIS3DSH chip (WHO_AM_I=0x3F) is in use, accel_start() re-routes
//   EXTI0 to PE0 (MEMS INT1) after disabling EXTI0_IRQn for the button.
//   accel_stop() re-routes EXTI0 back to PA0 and re-enables EXTI0_IRQn.
//   The EXTI0_IRQHandler dispatches based on the current SYSCFG EXTICR1
//   routing, so no stale handler is ever called.
//   On the early-silicon board (WHO_AM_I=0x01) EXTI0 is never re-routed and
//   the button works throughout.

#include "button.h"
#include "accel.h"
#include "tea.h"
#include "main.h"                       // HAL, CMSIS, EXTI0_IRQn
#include "stm32f4xx_ll_exti.h"

#define BUTTON_DEBOUNCE_MS  50u

static void button_toggle(void);

void button_init(void)
{
    // EXTI0 is already mapped to PA0 via SYSCFG EXTICR1 by MX_GPIO_Init().
    // Change EXTI0 from EVENT to IT mode; keep rising-edge trigger.
    LL_EXTI_InitTypeDef exti = {0};
    exti.Line_0_31   = LL_EXTI_LINE_0;
    exti.LineCommand = ENABLE;
    exti.Mode        = LL_EXTI_MODE_IT;
    exti.Trigger     = LL_EXTI_TRIGGER_RISING;
    LL_EXTI_Init(&exti);

    HAL_NVIC_SetPriority(EXTI0_IRQn, 6, 0);   // lower priority than MEMS (5)
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    namedAction(button_toggle);
}

// Called from EXTI0_IRQHandler (ISR context) when EXTI0 is routed to PA0.
// later() is ISR-safe; all real work runs in the event loop via button_toggle.
void button_exti0_isr(void)
{
    static uint32_t last_ms = 0u;
    uint32_t now = HAL_GetTick();
    if (now - last_ms < BUTTON_DEBOUNCE_MS) return;
    last_ms = now;
    later(button_toggle);
}

static void button_toggle(void)
{
    if (accel_is_running())
        accel_stop();
    else
        accel_start();
}
