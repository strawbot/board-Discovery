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

#include <stdint.h>
#include <stdbool.h>

#include "tea.h"
#include "cli.h"
#include "byteq.h"
#include "printers.h"

#include "tusb.h"

// ── Forward declaration ───────────────────────────────────────────────────────

void cdc_rx_action(void);

// ── EmitEvent target — drains emitq to USB CDC TX ────────────────────────────

static void cdc_emit(void) {
    while (qbq(emitq)) {
        uint8_t ch = pullbq(emitq);
        // tud_cdc_write returns bytes written — retry if buffer full.
        while (tud_cdc_write(&ch, 1) == 0) {
            // CDC TX buffer full — flush what we have and yield to usb_action.
            tud_cdc_write_flush();
            later(cdc_rx_action);   // requeue ourselves; usb_action will run first
            return;                 // remaining emitq bytes sent on next pass
        }
    }
    tud_cdc_write_flush();
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

// TX complete — nothing needed; cdc_emit drives all TX.
void tud_cdc_tx_complete_cb(uint8_t itf) {
    (void)itf;
}

// Connection state change — print prompt on connect.
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void)itf; (void)rts;
    if (dtr) {
        // Terminal connected — send prompt via emitq.
        when(EmitEvent, cdc_emit);
        print("\r\nTimbreOS> ");
    }
}
