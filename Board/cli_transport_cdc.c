// cli_transport_cdc.c — USB CDC transport for TimbreOS CLI
//
// Input routing:
//   Before each keyIn() call, EmitEvent is pointed at cdc_emit so output
//   is automatically directed back to the USB CDC serial port.
//   keyEcho controls character echo — disabled by default for CDC since
//   most terminal emulators handle local echo themselves.
//
// TinyUSB drives the USB stack via usb_action in the tea.c action queue.
// cdc_rx_action is posted by the TinyUSB CDC RX callback and drains the
// CDC RX FIFO into keyIn().
//
// TX backpressure:
//   cdc_emit checks tud_cdc_write_available() before pulling from emitq so
//   no byte is ever lost.  When the TinyUSB TX FIFO is full, cdc_emit queues
//   cdc_emit_action exactly once (guarded by emit_retry_pending) and returns.
//   usb_action runs first (it is always ahead in the queue), calls tud_task()
//   which flushes the FIFO to USB, then cdc_emit_action resumes the drain.
//   Without the gate a long response would push one later() per EmitEvent
//   fire, overflowing the action queue and triggering system_failure.

#include <stdint.h>
#include <stdbool.h>

#include "tea.h"
#include "cli.h"
#include "byteq.h"
#include "printers.h"

#include "tusb.h"

// ── Forward declarations ──────────────────────────────────────────────────────

void cdc_rx_action(void);
void cdc_emit_action(void);     // non-static — must be addressable by later()

// ── TX retry gate ─────────────────────────────────────────────────────────────
//
// Cleared by cdc_emit_action when it runs.  Prevents cdc_emit from pushing
// more than one pending retry into the action queue regardless of how many
// times EmitEvent fires while the TinyUSB TX FIFO is backed up.

static bool emit_retry_pending = false;

// ── EmitEvent target — drains emitq to USB CDC TX ────────────────────────────

static void cdc_emit(void) {
    while (qbq(emitq)) {
        if (tud_cdc_write_available() == 0) {
            // TX FIFO full — flush what is staged and yield.
            // Queue a single retry; usb_action will run first and drain
            // the FIFO to USB before cdc_emit_action resumes here.
            tud_cdc_write_flush();
            if (!emit_retry_pending) {
                emit_retry_pending = true;
                later(cdc_emit_action);
            }
            return;
        }
        uint8_t ch = pullbq(emitq);
        tud_cdc_write(&ch, 1);
    }
    tud_cdc_write_flush();
}

// ── cdc_emit_action — scheduled by cdc_emit when TX FIFO was full ─────────────

void cdc_emit_action(void) {
    emit_retry_pending = false;
    cdc_emit();
}

// ── cdc_rx_action — posted by TinyUSB CDC RX callback ────────────────────────

void cdc_rx_action(void) {
    if (!tud_cdc_connected()) return;
    if (!tud_cdc_available()) return;

    // Point output back to CDC before feeding any characters.
    when(EmitEvent, cdc_emit);
    autoEchoOff();          // terminal emulator handles local echo on CDC

    uint8_t buf[64];
    uint32_t count = tud_cdc_read(buf, sizeof(buf));

    for (uint32_t i = 0; i < count; i++) {
        keyIn((char)buf[i]);
    }

    // If more data arrived while we were processing, requeue.
    if (tud_cdc_available()) {
        later(cdc_rx_action);
    }
}

// ── TinyUSB CDC callbacks — called from tud_task() inside usb_action ─────────

// Posted when RX data is available.
void tud_cdc_rx_cb(uint8_t itf) {
    (void)itf;
    later(cdc_rx_action);
}

// TX FIFO drained to USB — if we have more to send, resume the drain.
void tud_cdc_tx_complete_cb(uint8_t itf) {
    (void)itf;
    if (qbq(emitq) && !emit_retry_pending) {
        emit_retry_pending = true;
        later(cdc_emit_action);
    }
}

// Connection state change — print prompt on connect.
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void)itf; (void)rts;
    if (dtr) {
        // Terminal connected — send prompt via emitq.
        when(EmitEvent, cdc_emit);
        dotPrompt();
    }
}
