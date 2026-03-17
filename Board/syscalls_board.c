// syscalls_board.c — redirect newlib stdout/stderr to USART6
//
// Any call to printf(), puts(), or fprintf(stdout/stderr, ...) in any
// library (LwIP, HAL, etc.) eventually calls _write().  By providing
// this implementation we catch all of them and route to USART6 directly
// via LL — safe even during early init before the tea.c action loop runs.
//
// CubeMX may generate a syscalls.c with a weak _write stub; this strong
// definition takes precedence.  If there is a link conflict, mark the
// CubeMX version as excluded from build or remove its _write body.

#include <sys/unistd.h>
#include "stm32f4xx_ll_usart.h"

int _write(int file, char *ptr, int len) {
    (void)file;     // treat all file descriptors as stdout
    for (int i = 0; i < len; i++) {
        while (!LL_USART_IsActiveFlag_TXE(USART6)) { }
        LL_USART_TransmitData8(USART6, (uint8_t)ptr[i]);
    }
    return len;
}
