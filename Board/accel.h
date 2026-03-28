#ifndef ACCEL_H
#define ACCEL_H

#include <stdbool.h>

// ── Chip identification ───────────────────────────────────────────────────────
//
// Both accelerometers share the same SPI pins and CS (PE3) on all Discovery
// board revisions.  WHO_AM_I (register 0x0F) distinguishes them:
//   0x3F → LIS3DSH          (16-bit, FIFO, standard production silicon)
//   0x01 → LIS3DSH (early)  (16-bit, FIFO, pre-production / engineering sample;
//                             same register map as 0x3F, different silicon ID)
//   0x3B → LIS302DL         ( 8-bit, no FIFO, oldest boards)

#define REG_WHO_AM_I  0x0Fu

typedef enum {
    ACCEL_NONE    = 0,
    ACCEL_LIS3DSH,
    ACCEL_LIS302DL,
} accel_type_t;

// accel_init — probe WHO_AM_I, identify chip, apply ODR and range settings.
// Uses LL SPI1 (must be initialised by MX_SPI1_Init() first) with the
// PHY-isolate / PA7-switch mechanism so it is safe to call after MX_LWIP_Init().
// Detected chip type is stored internally; query with show_acc() or accel_type.
void accel_init(void);

// accel_start — write FIFO config registers and schedule the first accel_poll().
// Call only after accel_init().  Does nothing if no LIS3DSH was detected.
void accel_start(void);

// accel_stop — suspend sampling without resetting chip state.
// Clears the running flag; accel_poll() sees it on its next fire and stops.
// accel_start() may be called again to resume.
void accel_stop(void);

// accel_is_running — true between a successful accel_start() and accel_stop().
// Safe to call from any event-loop context.
bool accel_is_running(void);

// show_acc — print accelerometer status to the CLI terminal.
// Reports chip type, sampling mode, WHO_AM_I, pitch, roll, sample count,
// and tap count.
void show_acc(void);

// show_regs — read and decode every LIS3DSH control/status register over SPI
// and print a human-readable summary.  Safe to call while accel is running.
void show_regs(void);

#endif // ACCEL_H
