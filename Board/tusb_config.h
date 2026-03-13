#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

// ── Board / controller ────────────────────────────────────────────────────────

#define CFG_TUSB_MCU                OPT_MCU_STM32F4
#define CFG_TUSB_OS                 OPT_OS_NONE         // no RTOS — tea.c handles scheduling
#define CFG_TUSB_DEBUG              0

// Use OTG_FS (PA11/PA12, mini-B connector on Discovery board).
// Change to OPT_MODE_HIGH_SPEED if using OTG_HS with ULPI.
#define CFG_TUSB_RHPORT0_MODE       (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

// ── Memory ────────────────────────────────────────────────────────────────────
// All buffers are statically allocated — no heap.

#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN          __attribute__((aligned(4)))

// ── Device stack ──────────────────────────────────────────────────────────────

#define CFG_TUD_ENDPOINT0_SIZE      64

// ── Class drivers — enable only what is needed ───────────────────────────────

#define CFG_TUD_CDC                 1       // CDC ACM for CLI serial port
#define CFG_TUD_MSC                 0
#define CFG_TUD_HID                 0
#define CFG_TUD_MIDI                0
#define CFG_TUD_VENDOR              0
#define CFG_TUD_AUDIO               0
#define CFG_TUD_VIDEO               0
#define CFG_TUD_USBTMC              0
#define CFG_TUD_DFU_RUNTIME         0
#define CFG_TUD_DFU                 0
#define CFG_TUD_BTH                 0
#define CFG_TUD_ECM_RNDIS           0
#define CFG_TUD_NCM                 0

// ── CDC configuration ─────────────────────────────────────────────────────────

#define CFG_TUD_CDC_RX_BUFSIZE      256     // RX FIFO — characters from host
#define CFG_TUD_CDC_TX_BUFSIZE      256     // TX FIFO — characters to host
#define CFG_TUD_CDC_EP_BUFSIZE      64      // USB packet size

#endif // TUSB_CONFIG_H
