// usb_dhcpd.c — minimal DHCP server for the USB NCM point-to-point link
//
// Hands out one fixed lease to whoever is on the other end of the USB cable:
//   Client:  192.168.7.2 / 255.255.255.0
//   Lease:   24 hours
//
// No router (option 3) is sent.  This is a point-to-point USB link; the host
// only needs a subnet route for 192.168.7.0/24 on its USB adapter.  Sending a
// gateway address causes some OS implementations (macOS in particular) to clone
// a host route for 192.168.7.1 via whichever interface wins the ARP race,
// which can end up on the wrong adapter when multiple NICs are present.
//
// Flow:
//   Host boots USB adapter → DHCP DISCOVER (broadcast)
//   We reply with DHCP OFFER (192.168.7.2)
//   Host accepts → DHCP REQUEST (broadcast)
//   We reply with DHCP ACK
//   Host configures adapter — HTTP/Telnet now reachable at 192.168.7.1
//
// Only DISCOVER and REQUEST are handled; all other message types are
// ignored, which is correct for a single-client point-to-point link.
// RENEW (unicast REQUEST with ciaddr set) also matches and gets an ACK.

#include <string.h>
#include <stdint.h>

#include "printers.h"

#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"
#include "lwip/err.h"

#include "usb_net.h"
#include "usb_dhcpd.h"

// ── Ports ─────────────────────────────────────────────────────────────────────

#define DHCP_SERVER_PORT    67
#define DHCP_CLIENT_PORT    68

// ── DHCP op codes ─────────────────────────────────────────────────────────────

#define BOOTREQUEST     1
#define BOOTREPLY       2

// ── DHCP message types (option 53 value) ─────────────────────────────────────

#define DHCPDISCOVER    1
#define DHCPOFFER       2
#define DHCPREQUEST     3
#define DHCPACK         5

// ── Magic cookie (RFC 2131) ───────────────────────────────────────────────────

#define DHCP_MAGIC      0x63825363UL

// ── Fixed addresses (host byte order — LWIP_MAKEU32 is host order) ────────────

#define SERVER_IP    LWIP_MAKEU32(192, 168, 7, 1)
#define CLIENT_IP    LWIP_MAKEU32(192, 168, 7, 2)
#define SUBNET_MASK  LWIP_MAKEU32(255, 255, 255, 0)
#define LEASE_SECS   86400UL    // 24 hours

// ── DHCP fixed-length header (236 bytes, RFC 2131 §2) ─────────────────────────

typedef struct __attribute__((packed)) {
    uint8_t  op;            // message op: BOOTREQUEST / BOOTREPLY
    uint8_t  htype;         // hardware type: 1 = Ethernet
    uint8_t  hlen;          // hardware address length: 6
    uint8_t  hops;          // relay agent hops: 0
    uint32_t xid;           // transaction ID (echoed from client)
    uint16_t secs;          // seconds since client started: 0
    uint16_t flags;         // flags (echoed from client)
    uint32_t ciaddr;        // client IP (0 for DISCOVER)
    uint32_t yiaddr;        // offered / acknowledged IP
    uint32_t siaddr;        // next-server IP (us)
    uint32_t giaddr;        // relay agent IP: 0
    uint8_t  chaddr[16];    // client hardware address (echoed)
    uint8_t  sname[64];     // server host name: zeroed
    uint8_t  file[128];     // boot file name: zeroed
    uint32_t magic;         // magic cookie 0x63825363
} dhcp_msg_t;

// ── Options block size ────────────────────────────────────────────────────────
// opt 53: type(1)+len(1)+val(1)           =  3
// opt 54: type(1)+len(1)+4-byte IP        =  6
// opt 51: type(1)+len(1)+4-byte time      =  6
// opt  1: type(1)+len(1)+4-byte mask      =  6
// opt255: end                             =  1
//                                  total = 22
// Note: option 3 (router/gateway) is intentionally omitted — see file header.

#define OPT_LEN     22

// ── Helper: write a 4-byte value in network (big-endian) byte order ───────────

static inline uint8_t *put32(uint8_t *p, uint32_t v) {
    *p++ = (v >> 24) & 0xFF;
    *p++ = (v >> 16) & 0xFF;
    *p++ = (v >>  8) & 0xFF;
    *p++ = (v      ) & 0xFF;
    return p;
}

// ── UDP receive callback ──────────────────────────────────────────────────────

static void dhcpd_recv(void *arg, struct udp_pcb *pcb,
                       struct pbuf *p, const ip_addr_t *src, u16_t port)
{
    (void)arg; (void)src; (void)port;

    // Flatten the inbound packet: fixed header + up to 64 bytes of options.
    uint8_t in[sizeof(dhcp_msg_t) + 64];
    if (p->tot_len < sizeof(dhcp_msg_t)) { pbuf_free(p); return; }
    uint16_t n = p->tot_len < (uint16_t)sizeof(in)
                 ? p->tot_len : (uint16_t)sizeof(in);
    pbuf_copy_partial(p, in, n, 0);
    pbuf_free(p);

    dhcp_msg_t *req = (dhcp_msg_t *)in;
    if (req->op    != BOOTREQUEST)             return;
    if (lwip_ntohl(req->magic) != DHCP_MAGIC)  return;

    // Scan options for message type (option 53).
    uint8_t msg_type = 0;
    uint8_t *o   = in + sizeof(dhcp_msg_t);
    uint8_t *end = in + n;
    while (o < end && *o != 255) {
        if (*o == 0) { o++; continue; }                // pad byte
        if (o + 1 >= end) break;
        if (*o == 53 && o + 2 < end) msg_type = o[2];
        o += 2 + o[1];                                  // skip to next option
    }

    if (msg_type != DHCPDISCOVER && msg_type != DHCPREQUEST) return;

    // ── Build reply ───────────────────────────────────────────────────────────

    struct pbuf *rp = pbuf_alloc(PBUF_TRANSPORT,
                                 sizeof(dhcp_msg_t) + OPT_LEN, PBUF_RAM);
    if (!rp) return;

    dhcp_msg_t *rep = (dhcp_msg_t *)rp->payload;
    memset(rep, 0, sizeof(dhcp_msg_t));

    rep->op     = BOOTREPLY;
    rep->htype  = 1;                                // Ethernet
    rep->hlen   = 6;
    rep->xid    = req->xid;                         // echo transaction ID
    rep->flags  = req->flags;                       // echo broadcast flag
    rep->yiaddr = lwip_htonl(CLIENT_IP);            // address we're offering
    rep->siaddr = 0;                                // next-server: not used
    memcpy(rep->chaddr, req->chaddr, 16);           // echo client hardware addr
    rep->magic  = lwip_htonl(DHCP_MAGIC);

    // Options
    uint8_t *op = (uint8_t *)(rep + 1);
    uint8_t reply_type = (msg_type == DHCPDISCOVER) ? DHCPOFFER : DHCPACK;

    *op++ = 53; *op++ = 1; *op++ = reply_type;         // DHCP message type
    *op++ = 54; *op++ = 4; op = put32(op, SERVER_IP);  // server identifier
    *op++ = 51; *op++ = 4; op = put32(op, LEASE_SECS); // IP address lease time
    *op++ =  1; *op++ = 4; op = put32(op, SUBNET_MASK);// subnet mask
    // option 3 (router) intentionally omitted — see file header
    *op++ = 255;                                        // end

    // Broadcast the reply on the USB netif.
    ip_addr_t bcast;
    IP4_ADDR(&bcast, 255, 255, 255, 255);
    udp_sendto_if(pcb, rp, &bcast, DHCP_CLIENT_PORT, &usb_netif);
    pbuf_free(rp);
}

// ── usb_dhcpd_init — bind port 67 and register callback ──────────────────────

void usb_dhcpd_init(void) {
    struct udp_pcb *pcb = udp_new();
    if (!pcb) {
        print("USB-DHCPD: pcb alloc failed\r\n");
        return;
    }

    if (udp_bind(pcb, IP_ADDR_ANY, DHCP_SERVER_PORT) != ERR_OK) {
        udp_remove(pcb);
        print("USB-DHCPD: bind failed\r\n");
        return;
    }

    udp_recv(pcb, dhcpd_recv, NULL);
    print("USB-DHCPD: 192.168.7.2 -> host\r\n");
}
