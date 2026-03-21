// ntp_sync.c — SNTP time synchronisation for bare-metal lwIP
//
// Architecture
// ─────────────
// lwIP's sntp.c handles all UDP socket management, RFC-compliant retry/back-
// off, and the 1-hour poll interval.  This file provides:
//
//   1. ntp_set_utc_seconds()  — the SNTP_SET_SYSTEM_TIME callback that
//                               records each sync result.
//   2. A simple RAM-based wall clock updated by interpolating getTime()
//                               (TimbreOS uptime, ms resolution).
//   3. ntp_sync_start/stop/kick — lifecycle wrappers called by network_init.
//
// NTP servers
// ────────────
// With SNTP_SERVER_DNS=1 (lwipopts.h) we can use hostnames.  Two servers
// are configured so the SNTP client has a fallback:
//
//   slot 0 : pool.ntp.org       (anycast pool, globally distributed)
//   slot 1 : time.cloudflare.com (reliable single-operator server)
//
// Interface routing
// ─────────────────
// lwIP chooses the output interface based on its routing table.  Ethernet
// (with a DHCP or static gateway) gives the device a default route to the
// internet.  The USB interface gateway is set to 192.168.7.2 (the address
// given to the USB host by the on-board DHCP server), so if the USB host
// has IP forwarding enabled, NTP packets can also reach the internet via
// USB.  whichever route resolves first will carry the SNTP traffic.

#include "ntp_sync.h"
#include "http_server.h"   // http_status_push()
#include "printers.h"
#include "tea.h"

#include "lwip/apps/sntp.h"
#include "lwip/ip_addr.h"

// ── State ─────────────────────────────────────────────────────────────────────

static volatile uint32_t utc_at_sync   = 0;  // UTC epoch at last sync
static volatile uint32_t uptime_at_sync = 0;  // getTime() ms at last sync
static volatile bool     synced         = false;

// ── SNTP callback ─────────────────────────────────────────────────────────────
//
// Called via the SNTP_SET_SYSTEM_TIME(sec) macro in lwipopts.h whenever
// sntp.c receives a valid NTP response.

void ntp_set_utc_seconds(uint32_t sec)
{
    utc_at_sync    = sec;
    uptime_at_sync = (uint32_t)getTime();
    synced         = true;

    // Print human-readable confirmation.  No printf available on this target;
    // format the epoch as decimal manually.
    print("NTP: synced, UTC=");
    char buf[12];
    uint32_t n = sec;
    int i = 10;
    buf[11] = '\0';
    if (n == 0) {
        buf[i] = '0';
    } else {
        while (n > 0 && i >= 0) {
            buf[i--] = (char)('0' + (n % 10));
            n /= 10;
        }
        i++;
    }
    print(&buf[i]);
    print("\r\n");

    // Push updated UTC time and sync state to any connected status SSE client.
    http_status_push();
}

// ── Public API ────────────────────────────────────────────────────────────────

void ntp_sync_start(void)
{
    if (sntp_enabled()) {
        return;  // already running — ntp_sync_kick() if you need a fresh burst
    }

    print("NTP: starting SNTP client (pool.ntp.org / time.cloudflare.com)\r\n");

    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_setservername(1, "time.cloudflare.com");
    sntp_init();
}

void ntp_sync_stop(void)
{
    if (!sntp_enabled()) {
        return;
    }
    sntp_stop();
    print("NTP: SNTP stopped\r\n");
}

void ntp_sync_kick(void)
{
    // Restart the SNTP client so it sends a fresh request immediately rather
    // than waiting for the next scheduled poll.  This is useful after DHCP
    // binds and sets up DNS — without a kick the first request may have
    // already failed with SNTP_STARTUP_DELAY expired but DNS not ready.
    if (sntp_enabled()) {
        sntp_stop();
    }
    print("NTP: kick — requesting immediate sync\r\n");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_setservername(1, "time.cloudflare.com");
    sntp_init();
}

uint32_t ntp_get_utc(void)
{
    if (!synced) {
        return 0;
    }
    // Interpolate: elapsed ms since last sync / 1000 gives whole seconds.
    uint32_t elapsed_ms = (uint32_t)getTime() - uptime_at_sync;
    return utc_at_sync + elapsed_ms / 1000u;
}

bool ntp_is_synced(void)
{
    return synced;
}
