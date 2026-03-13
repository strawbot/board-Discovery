// network_init.c — Board-specific network bring-up for Discovery
//
// Responsibilities:
//   - Initialise LwIP and register the Ethernet netif
//   - DHCP with timed fallback to static IP
//   - Bring up HTTP (port 80) and Telnet (port 23) servers
//   - Wire the LwIP timeout into the tea.c delta timer
//   - Handle link up / link down via netif status callback
//
// TimbreOS (tea.h, cli.h) is a sibling source directory.
// All OS primitives come from tea.h; no FreeRTOS, no heap.

#include "tea.h"
#include "cli.h"
#include "printers.h"
#include "ttypes.h"
#include "project_defs.h"

#include "ethernetif.h"
#include "http_server.h"
#include "telnet_server.h"

#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "lwip/ip_addr.h"
#include "lwip/dhcp.h"
#include "lwip/sys.h"
#include "netif/etharp.h"

// ── sys_now — required by LwIP timeouts.c ─────────────────────────────────────
// LwIP calls this for every timer operation.  We have no SysTick and no OS;
// get_ticks() from tea.c returns the delta-timer millisecond count.

u32_t sys_now(void) {
    return (u32_t)get_ticks();
}

// ── IP configuration ──────────────────────────────────────────────────────────

// DHCP timeout — if no lease acquired within this many ms, fall back to static.
#define DHCP_TIMEOUT_MS         15000

// Fallback static address — used if DHCP fails.
#define STATIC_IP0    192
#define STATIC_IP1    168
#define STATIC_IP2    100
#define STATIC_IP3     24

#define STATIC_GW0    192
#define STATIC_GW1    168
#define STATIC_GW2    100
#define STATIC_GW3      1

#define STATIC_MASK0  255
#define STATIC_MASK1  255
#define STATIC_MASK2  255
#define STATIC_MASK3    0

// MAC address — set to match your board or OTP fuses.
static uint8_t mac_addr[6] = { 0x02, 0x00, 0x00, 0x00, 0x00, 0x01 };

// ── DHCP state ────────────────────────────────────────────────────────────────

typedef enum {
    DHCP_STATE_OFF,         // link down, DHCP not running
    DHCP_STATE_TRYING,      // DHCP in progress, waiting for lease
    DHCP_STATE_BOUND,       // lease acquired
    DHCP_STATE_FALLBACK,    // DHCP timed out, using static IP
} dhcp_state_t;

static volatile dhcp_state_t dhcp_state = DHCP_STATE_OFF;

// ── Forward declarations ──────────────────────────────────────────────────────

static void servers_start(void);
static void servers_stop(void);
static void apply_static_ip(void);
static void kick_lwip_timer(void);

// ── lwip_timeout_action ───────────────────────────────────────────────────────
// Runs in the action queue — services all active LwIP timers then reschedules
// itself exactly when LwIP next needs attention.  No systick polling needed.

void lwip_timeout_action(void) {
    sys_check_timeouts();
    kick_lwip_timer();
}

// ── dhcp_timeout_action ───────────────────────────────────────────────────────
// Fires DHCP_TIMEOUT_MS after link-up if DHCP has not yet provided a lease.
// Falls back to the static address and brings the servers up.

void dhcp_timeout_action(void) {
    if (dhcp_state != DHCP_STATE_TRYING) {
        return;     // DHCP succeeded or link went down before timeout
    }

    print("NET: DHCP timeout — using static IP\r\n");

    dhcp_stop(&gnetif);
    dhcp_state = DHCP_STATE_FALLBACK;

    apply_static_ip();
    servers_start();
}

// ── dhcp_check_action ─────────────────────────────────────────────────────────
// Polled every second while DHCP is trying — detects lease acquisition without
// relying on a LwIP DHCP-bound callback (which does not exist in raw API).

void dhcp_check_action(void) {
    if (dhcp_state != DHCP_STATE_TRYING) {
        return;
    }

    struct dhcp *d = netif_dhcp_data(&gnetif);
    if (d && d->state == DHCP_STATE_BOUND) {
        dhcp_state = DHCP_STATE_BOUND;
        print("NET: DHCP bound — IP: ");
        print(ip4addr_ntoa(netif_ip4_addr(&gnetif)));
        print("\r\n");
        netif_set_up(&gnetif);
        servers_start();
    } else {
        // Still waiting — check again in 1 s.
        after(1000, dhcp_check_action);
    }
}

// ── netif link status callback ────────────────────────────────────────────────
// Called by LwIP when netif_set_link_up / netif_set_link_down is invoked.
// Runs in the action queue context (via eth_link_action → netif_set_link_*).

static void link_callback(struct netif *netif) {
    if (netif_is_link_up(netif)) {
        print("NET: link up — starting DHCP\r\n");

        dhcp_state = DHCP_STATE_TRYING;

        // Start DHCP — address will be zero until lease is acquired.
        dhcp_start(netif);

        // Schedule fallback in case DHCP never responds.
        after(DHCP_TIMEOUT_MS, dhcp_timeout_action);

        // Start polling for DHCP lease acquisition every second.
        after(1000, dhcp_check_action);

        // Kick the LwIP timeout chain now that we have a link.
        kick_lwip_timer();

    } else {
        print("NET: link down\r\n");

        dhcp_state = DHCP_STATE_OFF;
        dhcp_stop(netif);
        netif_set_down(netif);

        servers_stop();
    }
}

// ── Internal helpers ──────────────────────────────────────────────────────────

static void apply_static_ip(void) {
    ip4_addr_t ip, gw, mask;
    IP4_ADDR(&ip,   STATIC_IP0,   STATIC_IP1,   STATIC_IP2,   STATIC_IP3);
    IP4_ADDR(&gw,   STATIC_GW0,   STATIC_GW1,   STATIC_GW2,   STATIC_GW3);
    IP4_ADDR(&mask, STATIC_MASK0, STATIC_MASK1, STATIC_MASK2, STATIC_MASK3);
    netif_set_addr(&gnetif, &ip, &mask, &gw);
    netif_set_up(&gnetif);
    print("NET: static IP: ");
    print(ip4addr_ntoa(&ip));
    print("\r\n");
}

static void servers_start(void) {
    http_server_start();
    telnet_server_start();
}

static void servers_stop(void) {
    http_server_stop();
    telnet_server_stop();
}

static void kick_lwip_timer(void) {
    Long next = (Long)sys_timeouts_sleeptime();
    if (next != (Long)SYS_TIMEOUTS_SLEEPTIME_INFINITE) {
        after(next, lwip_timeout_action);
    }
}

// ── network_init ──────────────────────────────────────────────────────────────
// Call once from main after init_clocks() and before the action loop.

void network_init(void) {
    // ── LwIP core ─────────────────────────────────────────────────────────────
    lwip_init();

    // ── Ethernet netif ────────────────────────────────────────────────────────
    // Start with zeroed addresses — DHCP will fill them in.
    // If DHCP times out, dhcp_timeout_action() applies the static fallback.
    ip4_addr_t zero;
    IP4_ADDR(&zero, 0, 0, 0, 0);

    for (int i = 0; i < 6; i++) {
        gnetif.hwaddr[i] = mac_addr[i];
    }
    gnetif.hwaddr_len = 6;

    netif_add(&gnetif, &zero, &zero, &zero,
              NULL,               // state — not used
              ethernetif_init,    // init callback — sets up HAL ETH + DMA
              ethernet_input);    // input — standard LwIP Ethernet input

    netif_set_default(&gnetif);

    // Link callback fires on cable plug/unplug via eth_link_action().
    netif_set_link_callback(&gnetif, link_callback);

    // ── Servers ───────────────────────────────────────────────────────────────
    // Initialise fixed PCB pools and listener state now.
    // _start() is called from link_callback once an IP address is confirmed.
    http_server_init();
    telnet_server_init();

    print("NET: init done\r\n");
}
