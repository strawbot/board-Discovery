// ntp_sync.h — SNTP time synchronisation for bare-metal lwIP
//
// Wraps lwIP's built-in SNTP client.  On every successful NTP response,
// sntp.c calls SNTP_SET_SYSTEM_TIME (defined in lwipopts.h), which in turn
// calls ntp_set_utc_seconds() here.  The UTC epoch is held in RAM and
// interpolated using the TimbreOS uptime counter (getTime()).
//
// Typical usage from network_init.c:
//   - Call ntp_sync_start() when an internet-capable interface comes up.
//   - Call ntp_sync_stop() when all internet-capable interfaces go down.
//   - Read current UTC time with ntp_get_utc().
//
// The SNTP client uses DNS to reach "pool.ntp.org" and "time.cloudflare.com".
// It retries every 15 s until a response is received, then polls once per
// hour per the RFC.  Both Ethernet (with a gateway) and USB (when the host
// forwards internet traffic) are handled automatically by lwIP routing.

#ifndef NTP_SYNC_H
#define NTP_SYNC_H

#include <stdint.h>
#include <stdbool.h>

// ntp_sync_start — begin SNTP polling.
// Safe to call multiple times; no-op if SNTP is already running.
// Call from network_init or link_callback when internet access is available.
void ntp_sync_start(void);

// ntp_sync_stop — stop SNTP polling.
// Call when all internet-capable interfaces go down.
void ntp_sync_stop(void);

// ntp_sync_kick — force an immediate re-sync attempt.
// Useful after DHCP binds and the DNS server is newly configured.
// If SNTP is not yet running, starts it.
void ntp_sync_kick(void);

// ntp_get_utc — returns seconds since the Unix epoch (UTC).
// Interpolates using getTime() so sub-second accuracy improves over time.
// Returns 0 if no successful sync has occurred yet.
uint32_t ntp_get_utc(void);

// ntp_is_synced — returns true after the first successful NTP response.
bool ntp_is_synced(void);

// ntp_set_utc_seconds — called via the SNTP_SET_SYSTEM_TIME macro in
// lwipopts.h.  Not intended for direct use by application code.
void ntp_set_utc_seconds(uint32_t sec);

#endif // NTP_SYNC_H
