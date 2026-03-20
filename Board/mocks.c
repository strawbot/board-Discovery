#include "ttypes.h"
#include "tea.h"
#include "cli.h"
#include "byteq.h"
#include "timeout.h"
#include "printers.h"

void system_failure(Long reason) { while (true); }

static Long dropped_chars = 0;

void show_cli() { print("\nDropped chars: "), printDec(dropped_chars); }

void output() {
    Timeout box;
    setTimeout(secs(1), &box);
    while(fullbq(emitq))
        if (checkTimeout(&box))
            pullbq(emitq);
        else
            action_slice();
}

// HAL_RCC_GetHCLKFreq — stub to satisfy stm32f4xx_hal_pcd.c linker reference.
// HAL PCD is in the build (CubeMX generated MX_USB_OTG_FS_PCD_Init) but its
// IRQ handler is bypassed at runtime (OTG_FS_IRQHandler calls tud_int_handler
// instead).  The HAL RCC module is not compiled because the project uses LL
// for all clock configuration; this stub provides the required symbol.
uint32_t HAL_RCC_GetHCLKFreq(void) {
    return 168000000U;   // 168 MHz — must match SystemClock_Config HCLK
}
