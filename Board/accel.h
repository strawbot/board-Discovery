#ifndef ACCEL_H
#define ACCEL_H

#include <stdbool.h>

// ── Chip identification ───────────────────────────────────────────────────────
//
// Both accelerometers share the same SPI pins and CS (PE3) on all Discovery
// board revisions.  WHO_AM_I (register 0x0F) distinguishes them:
//   0x3F → LIS3DSH   (16-bit output, FIFO, post-~2012 boards)
//   0x3B → LIS302DL  ( 8-bit output, no FIFO path, older boards)

#define REG_WHO_AM_I  0x0Fu

typedef enum {
    ACCEL_NONE    = 0,
    ACCEL_LIS3DSH,
    ACCEL_LIS302DL,
} accel_type_t;

// accel_init — probe WHO_AM_I, identify chip, apply ODR and range settings.
// Uses HAL SPI1 (must be initialised by MX_SPI1_Init() first) with the
// PHY-isolate / PA7-switch mechanism so it is safe to call after MX_LWIP_Init().
// Returns true if a supported chip is found.
void accel_init(void);

// accel_start — arm the sample source appropriate for the detected chip.
//   LIS3DSH:  configures the hardware FIFO and enables the EXTI0 interrupt
//             on PE0 (MEMS_INT1); no Tea timer is used.
//   LIS302DL: schedules the first sample ISR via in().
// Call only after accel_init() returns true.
void accel_start(void);

// accel_stop — suspend sampling without resetting chip state.
//   LIS3DSH:  disables EXTI0_IRQn; the FIFO continues filling silently.
//   LIS302DL: clears the accel_running flag; the next ISR invocation will
//             not reschedule itself.
// accel_start() may be called again to resume.
void accel_stop(void);

// accel_int1_isr — ISR trampoline for LIS3DSH INT1 (EXTI0 on PE0).
// Must be called from EXTI0_IRQHandler after the EXTI flag is cleared.
// Defers all work to the event loop via later(); safe to call from interrupt.
void accel_int1_isr(void);

// show_acc — print accelerometer status to the CLI terminal.
// Reports chip type, sampling mode, WHO_AM_I, pitch, roll, sample count,
// and tap count.
void show_acc(void);

#endif // ACCEL_H
