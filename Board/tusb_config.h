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
#define CFG_TUD_NCM                 1       // CDC NCM for USB network (192.168.7.1/24)

// ── CDC configuration ─────────────────────────────────────────────────────────

#define CFG_TUD_CDC_RX_BUFSIZE      256     // RX FIFO — characters from host
#define CFG_TUD_CDC_TX_BUFSIZE      256     // TX FIFO — characters to host
#define CFG_TUD_CDC_EP_BUFSIZE      64      // USB packet size

// ── NCM configuration ─────────────────────────────────────────────────────────
// NTB (Network Transfer Block) buffers — keep small to fit in STM32F407 SRAM.
// 2 KB is enough for one full 1514-byte Ethernet frame plus NTB header overhead.

#define CFG_TUD_NCM_IN_NTB_MAX_SIZE     2048
#define CFG_TUD_NCM_OUT_NTB_MAX_SIZE    2048

#endif // TUSB_CONFIG_H
