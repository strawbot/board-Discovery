// usb_init.c — TinyUSB task pump and CDC descriptors for TimbreOS
//
// Hardware prerequisites (configured in CubeMX, NOT here):
//   USB_OTG_FS  Device Only, Full Speed, no internal VBUS sensing
//   → CubeMX generates MX_USB_OTG_FS_USB_Init() which configures:
//       PA11 OTG_FS_DM / PA12 OTG_FS_DP  (AF10, very high speed, no pull)
//       RCC AHB2 OTG_FS clock enable
//       NVIC OTG_FS_IRQn at appropriate priority
//   → CubeMX sets PLLQ=7 in SystemClock_Config for the 48 MHz USB clock
//     (8 MHz HSE / PLLM=8 × PLLN=336 / PLLQ=7 = 48 MHz)
//   → CubeMX calls MX_USB_OTG_FS_USB_Init() in main() before USER CODE BEGIN 2
//
// This file's sole job:
//   cdc_transport_init() — call tusb_init() then start usb_action in the queue
//   usb_action()         — call tud_task() on every pass, then requeue itself
//
// USB interrupt (stm32f4xx_it.c):
//   OTG_FS_IRQHandler → tud_int_handler(0)
//
// CLI I/O (cli_transport_cdc.c):
//   tud_cdc_rx_cb       → later(cdc_rx_action)   feeds keyIn()
//   tud_cdc_line_state_cb → prints prompt on connect
//   cdc_emit            → EmitEvent target drains emitq to CDC TX

#include <stdint.h>
#include <string.h>

#include "tea.h"
#include "printers.h"

#include "tusb.h"
#include "usb_init.h"

// ── usb_action — TinyUSB task pump in the tea.c action queue ─────────────────
//
// Self-rescheduling: always present in the queue so tud_task() is serviced
// on every pass.  USB interrupts call tud_int_handler(0) to poke the DWC2
// driver; tud_task() drains the resulting events and fires the cdc_*_cb
// callbacks defined in cli_transport_cdc.c.

void usb_action(void) {
    tud_task();
    later(usb_action);
}

// ── cdc_transport_init — call once from main() after MX_USB_OTG_FS_USB_Init ─
//
// By the time this is called, CubeMX has already configured GPIO and clocks.
// All we do here is start the TinyUSB stack and kick the action-queue pump.

void cdc_transport_init(void) {
    tusb_init();
    later(usb_action);
    namedAction(usb_action);
    print("USB: CDC init done\r\n");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  TinyUSB descriptor callbacks
//  All three are mandatory — called by TinyUSB during USB enumeration.
// ═══════════════════════════════════════════════════════════════════════════════

// ── String descriptor indices ─────────────────────────────────────────────────

enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_CDC,
};

// ── Device descriptor ─────────────────────────────────────────────────────────

static const tusb_desc_device_t device_desc = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    // TUSB_CLASS_MISC / MISC_SUBCLASS_COMMON / MISC_PROTOCOL_IAD allows Windows
    // to load a single composite driver for both CDC interfaces.
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = 0xCafe,       // TinyUSB development VID
    .idProduct          = 0x4001,
    .bcdDevice          = 0x0100,

    .iManufacturer      = STRID_MANUFACTURER,
    .iProduct           = STRID_PRODUCT,
    .iSerialNumber      = STRID_SERIAL,

    .bNumConfigurations = 0x01,
};

uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&device_desc;
}

// ── Configuration descriptor ──────────────────────────────────────────────────
//
// CDC ACM with two interfaces:
//   Interface 0 — CDC Communication  (notification EP 0x81, interrupt, 8 B)
//   Interface 1 — CDC Data           (bulk OUT 0x02 / bulk IN 0x82, 64 B)

#define EP_CDC_NOTIF        0x81
#define EP_CDC_OUT          0x02
#define EP_CDC_IN           0x82

#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

static const uint8_t config_desc[] = {
    // 1 configuration, 2 interfaces, no string, no remote wakeup, 100 mA
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    // CDC ACM: starting interface 0, string STRID_CDC
    TUD_CDC_DESCRIPTOR(0, STRID_CDC, EP_CDC_NOTIF, 8, EP_CDC_OUT, EP_CDC_IN, 64),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return config_desc;
}

// ── String descriptors ────────────────────────────────────────────────────────

static const char *const string_desc_arr[] = {
    (const char[]){ 0x09, 0x04 },   // 0: language — English (0x0409)
    "TimbreWorks",                   // 1: manufacturer
    "ActiveRobot",                   // 2: product
    "000001",                        // 3: serial number
    "TimbreOS CDC",                  // 4: CDC interface
};

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;

    static uint16_t desc_str[32];
    uint8_t chr_count;

    if (index == 0) {
        memcpy(&desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if (index >= (uint8_t)(sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))) {
            return NULL;
        }
        const char *str = string_desc_arr[index];
        chr_count = (uint8_t)strlen(str);
        if (chr_count > 31) chr_count = 31;
        for (uint8_t i = 0; i < chr_count; i++) {
            desc_str[1 + i] = (uint16_t)str[i];    // ASCII → UTF-16LE
        }
    }

    desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2u * chr_count + 2u));
    return desc_str;
}
