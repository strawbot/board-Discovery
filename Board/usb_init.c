// usb_init.c — TinyUSB task pump and composite CDC+NCM descriptors for TimbreOS
//
// Hardware prerequisites (configured in CubeMX, NOT here):
//   USB_OTG_FS  Device Only, Full Speed, no internal VBUS sensing
//   → CubeMX generates MX_USB_OTG_FS_USB_Init() which configures:
//       PA11 OTG_FS_DM / PA12 OTG_FS_DP  (AF10, very high speed, no pull)
//       RCC AHB2 OTG_FS clock enable
//       NVIC OTG_FS_IRQn at appropriate priority
//   → CubeMX sets PLLQ=7 in SystemClock_Config for the 48 MHz USB clock
//     (8 MHz HSE / PLLM=8 × PLLN=336 / PLLQ=7 = 48 MHz)
//
// Composite device — two functions:
//   CDC ACM  (interfaces 0+1): CLI serial port
//   CDC NCM  (interfaces 2+3): USB Ethernet → LwIP HTTP + Telnet
//
// Endpoint assignment (STM32F407 OTG-FS has 3 non-control IN endpoints):
//   EP1 OUT 0x01 / IN 0x81 — CDC bulk data
//   EP2 IN  0x82            — NCM interrupt notification
//   EP3 OUT 0x03 / IN 0x83 — NCM bulk data
//
// The CDC communication interface carries NO notification endpoint.
// Removing it frees EP2 IN for NCM's required notification endpoint.
// TinyUSB's CDC ACM driver treats the notification EP as optional; data
// transfer (bulk IN/OUT) works normally without it.
//
// USB interrupt (stm32f4xx_it.c):
//   OTG_FS_IRQHandler → tud_int_handler(0)
//
// CLI I/O (cli_transport_cdc.c):
//   tud_cdc_rx_cb         → later(cdc_rx_action)   feeds keyIn()
//   tud_cdc_line_state_cb → prints prompt on connect
//   cdc_emit              → EmitEvent target drains emitq to CDC TX
//
// Network I/O (usb_net.c):
//   tud_network_recv_cb   → passes frame to LwIP
//   tud_network_xmit_cb   → copies LwIP pbuf into TinyUSB NCM buffer

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
// driver; tud_task() drains the resulting events and fires the CDC and NCM
// callbacks defined in cli_transport_cdc.c and usb_net.c.

void usb_action(void) {
    tud_task();
    later(usb_action);
}

// ── cdc_transport_init — call once from main() after MX_USB_OTG_FS_USB_Init ─

void cdc_transport_init(void) {
    tusb_init();
    later(usb_action);
    namedAction(usb_action);
    print("USB: CDC+NCM init done\r\n");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  TinyUSB descriptor callbacks
// ═══════════════════════════════════════════════════════════════════════════════

// ── String descriptor indices ─────────────────────────────────────────────────

enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_CDC,
    STRID_NCM_MAC,      // MAC address as 12-char uppercase hex (ECM functional desc)
    STRID_NCM_IF,       // NCM interface name
};

// ── Endpoint addresses ────────────────────────────────────────────────────────

#define EP_CDC_OUT      0x01    // EP1 OUT — CDC bulk data
#define EP_CDC_IN       0x81    // EP1 IN  — CDC bulk data
#define EP_NCM_NOTIF    0x82    // EP2 IN  — NCM interrupt notification
#define EP_NCM_OUT      0x03    // EP3 OUT — NCM bulk data
#define EP_NCM_IN       0x83    // EP3 IN  — NCM bulk data

// ── Device descriptor ─────────────────────────────────────────────────────────

static const tusb_desc_device_t device_desc = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    // MISC/COMMON/IAD allows Windows to load a single composite driver
    // for both CDC functions without ambiguity.
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = 0xCafe,
    .idProduct          = 0x4002,   // distinct PID from CDC-only build
    .bcdDevice          = 0x0100,

    .iManufacturer      = STRID_MANUFACTURER,
    .iProduct           = STRID_PRODUCT,
    .iSerialNumber      = STRID_SERIAL,

    .bNumConfigurations = 0x01,
};

uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&device_desc;
}

// ── Hand-crafted CDC ACM descriptor without notification endpoint ─────────────
//
// Standard TUD_CDC_DESCRIPTOR includes a 7-byte notification EP descriptor
// (interrupt IN) which costs one of the three available IN endpoints on the
// STM32F407 OTG-FS.  Removing it frees that endpoint for NCM's notification.
//
// Layout: IAD(8) + CommIface(9) + Header(5) + CallMgmt(5) + ACM(4) + Union(5)
//         + DataIface(9) + EPout(7) + EPin(7)  =  59 bytes
//
// bNumEndpoints in the Communication Interface is 0 (was 1 with notification).

#define CDC_NO_NOTIF_DESC_LEN   59

#define TUD_CDC_NO_NOTIF_DESCRIPTOR(_itfnum, _desc_stridx, _epout, _epin, _epsize) \
  /* Interface Association Descriptor */                                             \
  8, TUSB_DESC_INTERFACE_ASSOCIATION,                                                \
    _itfnum, 2,                                                                      \
    TUSB_CLASS_CDC, CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL,                       \
    CDC_COMM_PROTOCOL_NONE, 0,                                                       \
  /* CDC Communication Interface — 0 endpoints (notification EP omitted) */         \
  9, TUSB_DESC_INTERFACE,                                                            \
    _itfnum, 0, 0,                                                                   \
    TUSB_CLASS_CDC, CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL,                       \
    CDC_COMM_PROTOCOL_NONE, _desc_stridx,                                            \
  /* CDC Header Functional Descriptor */                                             \
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_HEADER, U16_TO_U8S_LE(0x0120),          \
  /* CDC Call Management Functional Descriptor */                                    \
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_CALL_MANAGEMENT,                         \
    0x00, (uint8_t)((_itfnum) + 1),                                                  \
  /* CDC ACM Functional Descriptor — support Set/Get Line Coding, Set Control */    \
  4, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_ABSTRACT_CONTROL_MANAGEMENT, 0x02,      \
  /* CDC Union Functional Descriptor */                                              \
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_UNION,                                   \
    _itfnum, (uint8_t)((_itfnum) + 1),                                               \
  /* CDC Data Interface */                                                            \
  9, TUSB_DESC_INTERFACE,                                                            \
    (uint8_t)((_itfnum) + 1), 0, 2,                                                  \
    TUSB_CLASS_CDC_DATA, 0x00, 0x00, 0x00,                                           \
  /* Bulk OUT endpoint */                                                             \
  7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0,        \
  /* Bulk IN endpoint */                                                              \
  7, TUSB_DESC_ENDPOINT, _epin,  TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0

// ── Configuration descriptor ──────────────────────────────────────────────────
//
// 4 interfaces:
//   0  CDC Communication  (no notification EP)
//   1  CDC Data           (EP1 OUT / EP1 IN, bulk, 64 B)
//   2  NCM Communication  (EP2 IN, interrupt, 64 B, notification)
//   3  NCM Data           (alt-0: no EPs; alt-1: EP3 OUT / EP3 IN, bulk, 64 B)

#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + CDC_NO_NOTIF_DESC_LEN + TUD_CDC_NCM_DESC_LEN)

static const uint8_t config_desc[] = {
    // 1 config, 4 interfaces, no string, 100 mA
    TUD_CONFIG_DESCRIPTOR(1, 4, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    // CDC ACM: interfaces 0+1, no notification EP
    TUD_CDC_NO_NOTIF_DESCRIPTOR(0, STRID_CDC, EP_CDC_OUT, EP_CDC_IN, 64),

    // CDC NCM: interfaces 2+3, notification EP2 IN, data EP3 OUT/IN, MTU 1514
    // Macro signature: (itfnum, iface_stridx, mac_stridx, ep_notif, ep_notif_size,
    //                   epout, epin, epsize, maxsegmentsize)
    TUD_CDC_NCM_DESCRIPTOR(2, STRID_NCM_IF, STRID_NCM_MAC, EP_NCM_NOTIF, 64,
                           EP_NCM_OUT, EP_NCM_IN, 64, 1514),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return config_desc;
}

// ── String descriptors ────────────────────────────────────────────────────────
//
// STRID_NCM_MAC must be exactly 12 uppercase hex ASCII characters —
// the ECM Ethernet Functional Descriptor points to this string to convey
// the device MAC address to the host.  Must match usb_net.c usb_mac[].

static const char *const string_desc_arr[] = {
    (const char[]){ 0x09, 0x04 },   // 0: language — English (0x0409)
    "TimbreWorks",                   // 1: manufacturer
    "ActiveRobot",                   // 2: product
    "000001",                        // 3: serial number
    "TimbreOS CDC",                  // 4: CDC interface
    "020284000001",                  // 5: NCM MAC  02:02:84:00:00:01
    "TimbreOS Net",                  // 6: NCM interface
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
