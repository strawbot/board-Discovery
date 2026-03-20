// usb_net.c — TinyUSB CDC-NCM callbacks + LwIP netif driver
//
// Bridges the TinyUSB NCM network driver to a bare-metal LwIP netif.
// The device presents itself as a USB Ethernet adapter with:
//   IP  192.168.7.1 / 24
//   MAC 02:02:84:00:00:01
//
// The host side gets a RNDIS/NCM Ethernet interface; assign it any address
// on 192.168.7.0/24 (e.g. 192.168.7.2) and the HTTP and Telnet servers are
// reachable at 192.168.7.1:80 and 192.168.7.1:23.
//
// Internet routing via USB
// ────────────────────────
// The USB gateway is set to 192.168.7.2 (the first address assigned by the
// on-board DHCP server to the USB host).  If the host has IP forwarding
// enabled (e.g. macOS Internet Sharing, Linux ip_forward=1), LwIP will route
// internet-bound packets — including NTP/DNS — through the USB host.
// If the host does not forward, those packets are silently dropped and the
// Ethernet interface (when connected) handles internet traffic instead.
//
// TinyUSB NCM callbacks (called from tud_task() inside usb_action):
//   tud_network_init_cb()    — USB host reset / re-enumeration
//   tud_network_recv_cb()    — frame received from host → LwIP input
//   tud_network_xmit_cb()    — copy pending LwIP frame into TinyUSB buffer
//
// LwIP netif:
//   usb_netif_linkoutput()   — LwIP output → TinyUSB xmit
//
// TX flow:
//   LwIP → usb_netif_linkoutput() → tud_network_can_xmit() ?
//           yes: tud_network_xmit(pbuf, 0) → tud_network_xmit_cb() copies pbuf
//           no:  pbuf_ref, save in pending_tx, later(usb_net_tx_action)
//   usb_net_tx_action() retries until tud_network_can_xmit() returns true.

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "tea.h"
#include "printers.h"

#include "tusb.h"
#include "usb_net.h"
#include "usb_dhcpd.h"
#include "http_server.h"
#include "telnet_server.h"

#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include "lwip/etharp.h"
#include "netif/ethernet.h"

// ── Static configuration ──────────────────────────────────────────────────────

#define USB_IP0    192
#define USB_IP1    168
#define USB_IP2      7
#define USB_IP3      1

#define USB_GW0    192
#define USB_GW1    168
#define USB_GW2      7
#define USB_GW3      2  // host address assigned by usb_dhcpd; forwards internet if enabled

#define USB_MASK0  255
#define USB_MASK1  255
#define USB_MASK2  255
#define USB_MASK3    0

// The USB NCM descriptor advertises "020284000001" as iMACAddress.
// macOS (and some other hosts) assign that string as the HOST interface MAC
// (en17 etc.).  The board's netif MAC must be DIFFERENT — if both sides share
// the same MAC, every Ethernet frame from the board looks like a loopback to
// the host and is silently discarded at the Ethernet layer, breaking DHCP and
// all other traffic.
//
// Convention:  descriptor MAC (020284000001) → host/en17
//              netif MAC      (020284000002) → board
static const uint8_t usb_mac[6] = { 0x02, 0x02, 0x84, 0x00, 0x00, 0x02 };

// ── Global netif ──────────────────────────────────────────────────────────────

struct netif usb_netif;

// ── Pending TX slot ───────────────────────────────────────────────────────────
//
// If the TinyUSB NCM TX buffer is busy when LwIP wants to send, we hold a
// single pbuf reference here and retry from usb_net_tx_action.
//
// Priority rule: DHCP responses (OFFERs, ACKs) never occupy this slot.
// If the slot is taken, or the new frame is a DHCP response, the new frame
// is dropped silently.  ARP replies and TCP segments take priority because
// they cannot be recovered without a full connection restart, whereas DHCP
// loss only triggers a new DISCOVER cycle.

static struct pbuf *pending_tx = NULL;

// ── Forward declarations ──────────────────────────────────────────────────────

static void usb_net_tx_action(void);

// ── DHCP frame classifier ─────────────────────────────────────────────────────
//
// Returns non-zero if p is a DHCP response (IPv4 UDP, source port 67).
//
// DHCP responses are low-priority: the host retries DISCOVER automatically,
// so it is safe to drop an OFFER or ACK that cannot be sent immediately.
// We must NOT allow DHCP to occupy the pending_tx slot, because that would
// prevent higher-priority ARP replies and TCP segments from being queued,
// causing connection failures.
//
// Frame layout assumed (LwIP raw, no VLAN tags):
//   [0..13]  Ethernet header (dst MAC, src MAC, EtherType)
//   [14..33] IPv4 header (IHL=5, 20 bytes — LwIP never adds IP options)
//   [34..35] UDP source port
//
static int pbuf_is_dhcp(struct pbuf *p) {
    if (p->tot_len < 36) return 0;          // too short to contain UDP src port

    uint8_t hdr[36];
    pbuf_copy_partial(p, hdr, sizeof(hdr), 0);

    if (hdr[12] != 0x08 || hdr[13] != 0x00) return 0;  // not IPv4
    if (hdr[23] != 0x11)                    return 0;   // not UDP
    uint16_t sport = ((uint16_t)hdr[34] << 8) | hdr[35];
    return sport == 67;                                  // DHCP server port
}

// ── tud_network_init_cb — USB host reset or re-enumeration ───────────────────

void tud_network_init_cb(void) {
    // Our LwIP netif stays registered regardless of USB host state.
    // Nothing to reset here.
}

// ── tud_network_recv_cb — Ethernet frame received from USB host ───────────────
//
// Called from tud_task() with a pointer into the TinyUSB RX buffer.
// We must call tud_network_recv_renew() before returning to release the
// buffer for the next incoming frame.
//
// The pbuf is allocated from the PBUF_POOL and handed to LwIP via the
// netif input function (ethernet_input in NO_SYS=1 builds).

bool tud_network_recv_cb(const uint8_t *src, uint16_t size) {
    struct pbuf *p = pbuf_alloc(PBUF_RAW, size, PBUF_POOL);
    if (p) {
        pbuf_take(p, src, size);
        if (usb_netif.input(p, &usb_netif) != ERR_OK) {
            pbuf_free(p);
        }
    }
    tud_network_recv_renew();
    return true;
}

// ── tud_network_xmit_cb — copy outgoing frame into TinyUSB NCM buffer ────────
//
// Called by TinyUSB after tud_network_xmit() is invoked.  ref is the pbuf
// pointer we passed; we copy the pbuf chain into the flat dst buffer and
// return the total byte count.

uint16_t tud_network_xmit_cb(uint8_t *dst, void *ref, uint16_t arg) {
    struct pbuf *p = (struct pbuf *)ref;
    (void)arg;

    uint16_t len = 0;
    for (struct pbuf *q = p; q != NULL; q = q->next) {
        memcpy(dst + len, q->payload, q->len);
        len += q->len;
    }
    return len;
}

// ── usb_net_tx_action — scheduled retry when NCM TX buffer was busy ───────────

static void usb_net_tx_action(void) {
    if (!pending_tx) return;
    if (!tud_ready()) {
        // USB not connected — discard the pending frame.
        pbuf_free(pending_tx);
        pending_tx = NULL;
        return;
    }
    if (tud_network_can_xmit(pending_tx->tot_len)) {
        tud_network_xmit(pending_tx, 0);
        pbuf_free(pending_tx);
        pending_tx = NULL;
    } else {
        later(usb_net_tx_action);
    }
}

// ── usb_netif_linkoutput — LwIP calls this to transmit a frame ───────────────

static err_t usb_netif_linkoutput(struct netif *netif, struct pbuf *p) {
    (void)netif;

    if (!tud_ready()) return ERR_OK;   // USB not connected — drop silently

    if (tud_network_can_xmit(p->tot_len)) {
        // TX buffer free — send immediately.
        tud_network_xmit(p, 0);
    } else if (!pending_tx && !pbuf_is_dhcp(p)) {
        // TX buffer busy, nothing queued yet, and this is not a DHCP frame.
        // Hold it for deferred transmission.
        //
        // DHCP responses (OFFERs, ACKs) are deliberately excluded from the
        // pending slot: the client retries DISCOVER automatically, so a
        // dropped response is harmless.  Allowing DHCP to occupy pending_tx
        // would starve ARP replies and TCP segments (SYN-ACKs, HTTP
        // responses), because those arrive shortly after and find the slot
        // taken, leading to connection timeouts.
        pbuf_ref(p);
        pending_tx = p;
        later(usb_net_tx_action);
    }
    // Frames that fall through here are dropped silently:
    //   • pending_tx already occupied (any frame type), or
    //   • DHCP response that couldn't be sent immediately.
    // TCP handles loss via retransmit; DHCP handles loss via DISCOVER retry.
    return ERR_OK;
}

// ── usb_netif_lwip_init — LwIP netif init callback ───────────────────────────

static err_t usb_netif_lwip_init(struct netif *netif) {
    netif->hwaddr_len = ETH_HWADDR_LEN;
    memcpy(netif->hwaddr, usb_mac, ETH_HWADDR_LEN);
    netif->mtu        = 1514;
    netif->name[0]    = 'u';
    netif->name[1]    = 's';
    netif->output     = etharp_output;
    netif->linkoutput = usb_netif_linkoutput;
    netif->flags      = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;
    return ERR_OK;
}

// ── usb_netif_init — public entry point, call from network_init() ─────────────

void usb_netif_init(void) {
    ip4_addr_t ip, gw, mask;
    IP4_ADDR(&ip,   USB_IP0,   USB_IP1,   USB_IP2,   USB_IP3);
    IP4_ADDR(&gw,   USB_GW0,   USB_GW1,   USB_GW2,   USB_GW3);
    IP4_ADDR(&mask, USB_MASK0, USB_MASK1, USB_MASK2, USB_MASK3);

    netif_add(&usb_netif, &ip, &mask, &gw, NULL, usb_netif_lwip_init, ethernet_input);
    netif_set_up(&usb_netif);

    print("USB-NET: 192.168.7.1/24\r\n");

    // Start DHCP server so the host adapter self-configures to 192.168.7.2.
    usb_dhcpd_init();

    // Start HTTP and Telnet servers if not already running.
    // http_server_start() and telnet_server_start() are idempotent — they
    // both guard against double-start, so calling them here is safe even if
    // the Ethernet link_callback already started them.
    http_server_start();
    telnet_server_start();
}
