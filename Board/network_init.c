// network_init.c — Board-specific network bring-up for Discovery
//
// Responsibilities:
//   - Initialise LwIP and register the Ethernet netif
//   - DHCP with timed fallback to static IP
//   - Bring up HTTP (port 80) and Telnet (port 23) servers
//   - Wire the LwIP timeout into the tea.c delta timer
//   - Handle link up / link down via netif status callback
//   - Start SNTP time sync when internet-capable interface comes up
//
// TimbreOS (tea.h, cli.h) is a sibling source directory.
// All OS primitives come from tea.h; no FreeRTOS, no heap.

#include "tea.h"
#include "cli.h"
#include "printers.h"
#include "ttypes.h"
#include "project_defs.h"

#include "ethernetif.h"
#include "http_server.h"           // now lives in Robot/net/http/ (added to include path)
#include "http_streams.h"          // Discovery SSE push functions
#include "telnet_server.h"
#include "usb_net.h"
#include "ntp_sync.h"           // now lives in Robot/net/ntp/ (added to include path)
#include "network_init.h"

// ── NTP → HTTP bridge ─────────────────────────────────────────────────────────
// The Robot NTP module no longer calls http_server directly (see Robot/README.md
// for the layering rule).  Boards that want status-stream pushes on every sync
// register a small adapter with ntp_on_sync() during network_init().

static void ntp_push_http_status(uint32_t utc_seconds)
{
    (void)utc_seconds;        // http_status_push pulls the value itself
    http_status_push();
}

#include "tusb.h"           // tud_ready() — USB host connection state

#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "lwip/ip_addr.h"
#include "lwip/dhcp.h"
#include "lwip/sys.h"
#include "netif/etharp.h"
#include "netif/ethernet.h"

// ── sys_now — required by LwIP timeouts.c ─────────────────────────────────────
// LwIP calls this for every timer operation.  We have no SysTick and no OS;
// getTime() from tea.c returns the delta-timer millisecond count.

u32_t sys_now(void) {
    return (u32_t)getTime();
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

// MAC address is set by MX_LWIP_Init() in LWIP/App/lwip.c (CubeMX-generated).

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

    print("NET: DHCP timeout — keeping static IP\r\n");

    dhcp_stop(&gnetif);
    dhcp_state = DHCP_STATE_FALLBACK;
    // apply_static_ip() and servers_start() were already called on link-up.
}

// ── dhcp_check_action ─────────────────────────────────────────────────────────
// Polled every second while DHCP is trying — detects lease acquisition without
// relying on a LwIP DHCP-bound callback (which does not exist in raw API).

void dhcp_check_action(void) {
    if (dhcp_state != DHCP_STATE_TRYING) {
        return;
    }

    if (dhcp_supplied_address(&gnetif)) {
        dhcp_state = DHCP_STATE_BOUND;
        print("NET: DHCP bound — IP: ");
        print(ip4addr_ntoa(netif_ip4_addr(&gnetif)));
        print("\r\n");
        // netif and servers already up from link_callback; IP was updated by DHCP.
        // Kick SNTP immediately: DHCP has now configured a DNS server, so the
        // earlier DNS lookup for pool.ntp.org (which may have failed while DHCP
        // was still negotiating) will now succeed.
        ntp_sync_kick();
        // Push updated Ethernet status (now shows real IP + "DHCP").
        http_status_push();
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
        // Use static IP directly — no DHCP.
        // dhcp_start() zeros the netif address; setting a static IP while DHCP
        // is active triggers LwIP's dhcp_ipv4_addr_changed hook which resets
        // DHCP back to INIT and re-zeros the address.  Skipping DHCP avoids
        // that fight entirely.  Re-enable when a DHCP server is available.
        print("NET: link up\r\n");
        dhcp_state = DHCP_STATE_FALLBACK;
        // Neutralise any DHCP started by MX_LWIP_Init() before we arrived.
        // dhcp_stop() sets state=OFF so the fine/coarse timers leave the
        // netif address alone.  Must be called before apply_static_ip() so
        // that netif_set_addr() does not trigger dhcp_ipv4_addr_changed()
        // with DHCP in a live state, which would set the address back to 0.
        // dhcp_stop(&gnetif);
        netif_set_up(&gnetif);
        apply_static_ip();
        servers_start();
        kick_lwip_timer();
        // Assert Ethernet as the default route now that its link is up.
        network_update_default_route();
        // Restart SNTP unconditionally via kick (stop + reinit) rather than
        // the conditional ntp_sync_start().  This ensures the SNTP PCB is
        // always fresh on link-up, including after eth_recovery_action() which
        // resets the MAC DMA under any open UDP sockets.  A stale PCB would
        // silently fail: sntp_enabled() returns true so ntp_sync_start() is a
        // no-op, but the underlying socket can no longer receive responses.
        ntp_sync_kick();
        // Push link-up status to any connected status SSE client.
        http_status_push();

    } else {
        print("NET: link down\r\n");
        dhcp_state = DHCP_STATE_OFF;
        netif_set_down(netif);
        servers_stop();
        // Stop SNTP so its UDP PCB is closed cleanly before the MAC is reset.
        // If eth_recovery_action() triggered this link-down, the MAC DMA is
        // about to be torn down; leaving the PCB open causes ntp_sync_start()
        // to be a no-op on the next link-up (sntp_enabled() still true) and
        // the refreshed socket never receives responses.  ntp_sync_kick() on
        // link-up will restart it with a new PCB.
        ntp_sync_stop();
        // Fall back to USB as the default route if a host is connected.
        // This lets NTP reach the internet via USB host IP forwarding while
        // Ethernet is absent.  If USB is also unavailable, the default stays
        // on Ethernet and NTP will simply retry when connectivity returns.
        network_update_default_route();
        if (tud_ready()) {
            // USB host can forward internet traffic — keep SNTP alive.
            ntp_sync_start();
        }
        // Push link-down status to any connected status SSE client.
        http_status_push();
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

// ── network_update_default_route ──────────────────────────────────────────────
// Sets the lwIP default netif — the interface used for all internet-bound
// traffic (NTP, DNS, anything not matching a directly-connected subnet).
//
// Priority:
//   1. Ethernet link up               → always preferred (lower latency,
//                                       no dependency on host IP forwarding)
//   2. Ethernet down + USB host ready → fall back to USB; the host must have
//                                       IP forwarding enabled for this to reach
//                                       the internet (macOS Internet Sharing,
//                                       Linux ip_forward=1, etc.)
//   3. Neither available              → no change; NTP will fail gracefully
//                                       and retry on its schedule.
//
// Call this after any change that may affect which interface can reach the
// internet: Ethernet link-up, link-down, and USB host enumeration.

void network_update_default_route(void)
{
    if (netif_is_link_up(&gnetif)) {
        netif_set_default(&gnetif);
    } else if (tud_ready()) {
        netif_set_default(&usb_netif);
    }
    // else: no usable route — leave current default unchanged.
}

// ── eth_status ────────────────────────────────────────────────────────────────
// Called by http_server.c when building the status JSON for the SSE stream.

const char *eth_status(void)
{
    static char buf[40];

    if (!netif_is_link_up(&gnetif)) {
        return "link down";
    }

    const ip4_addr_t *ip = netif_ip4_addr(&gnetif);
    const char *mode;
    switch (dhcp_state) {
        case DHCP_STATE_BOUND:    mode = "DHCP";    break;
        case DHCP_STATE_TRYING:   mode = "DHCP..."; break;
        case DHCP_STATE_FALLBACK: mode = "static";  break;
        default:                  mode = "up";      break;
    }

    // ip4addr_ntoa() uses a single internal static buffer — copy before reuse.
    snprintf(buf, sizeof(buf), "%s (%s)", ip4addr_ntoa(ip), mode);
    return buf;
}

void lwip_assert_handler(const char *msg) {
    print("LWIP assert: ");
    print(msg);
    printCr();
}

// ── network_init ──────────────────────────────────────────────────────────────
// Call once from main AFTER MX_LWIP_Init() has run.
// MX_LWIP_Init() owns lwip_init(), netif_add(), and hwaddr setup.
// This function adds board-specific callbacks and starts services on top.

void network_init(void) {
    netif_set_default(&gnetif);

    // CubeMX may generate MX_LWIP_Init() with NO_SYS=0, which sets
    // gnetif.input = tcpip_input (the RTOS mailbox version).  Without a
    // tcpip thread running, that function silently queues every received
    // frame and never processes it.  For bare-metal operation we need
    // ethernet_input called directly.  Setting it here is safe and idempotent:
    // if NO_SYS=1 is already in effect, tcpip_input IS ethernet_input and
    // this write is a no-op.
    gnetif.input = ethernet_input;

    // Link callback fires when eth_link_action() calls netif_set_link_up/down.
    netif_set_link_callback(&gnetif, link_callback);

    // Start the 1-second BSR polling loop for LAN8720 link detection.
    later(eth_link_action);
    namedAction(eth_link_action);

    // Initialise fixed PCB pools — _start() called from link_callback
    // once an IP address is confirmed.
    http_server_init();
    http_streams_init();    // register Discovery routes + SSE channels
    telnet_server_init();
    namedAction(lwip_timeout_action);

    // USB network interface — static 192.168.7.1/24, always up.
    // HTTP and Telnet are reachable at 192.168.7.1 once the host assigns
    // itself an address on the 192.168.7.0/24 subnet.
    usb_netif_init();

    // Initialise the Robot NTP module and wire the board-specific listener
    // that forwards each sync to the HTTP status-stream.  Must come before
    // ntp_sync_start() so the first sync fan-out reaches the observer.
    ntp_sync_init();
    ntp_on_sync(ntp_push_http_status);

    // Start SNTP now so USB-only operation can sync if the USB host forwards
    // internet traffic.  ntp_sync_start() is idempotent — the Ethernet
    // link_callback calls it too, so whichever fires first wins.
    ntp_sync_start();

    // Unconditionally kick the LwIP timer chain here.
    //
    // Without this, lwip_timeout_action only starts when the Ethernet PHY
    // signals link-up (eth_link_action → netif_set_link_up → link_callback →
    // kick_lwip_timer).  If Ethernet is absent — e.g. USB-only operation —
    // sys_check_timeouts() never runs.  The immediate consequence is that
    // LwIP's TCP delayed-ACK timer never fires: the board queues ACKs that
    // are never sent, causing the remote side to retransmit endlessly and
    // curl / browser connections to hang until timeout.
    kick_lwip_timer();

    print("NET: init done\r\n");
}
