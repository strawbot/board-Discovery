#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H

#include <stdint.h>
#include <stdbool.h>

// ── Public API ────────────────────────────────────────────────────────────────

// Call once from network_init() — allocates no memory, sets up pool.
void http_server_init(void);

// Call from link_callback when IP is confirmed — opens TCP listener port 80.
void http_server_start(void);

// Call from link_callback on link down — closes listener and all connections.
void http_server_stop(void);

// Send an SSE comment ping to keep the stream connection alive through proxies.
// Scheduled automatically on SSE connect via after(); reschedules itself.
void http_sse_keepalive(void);

// Fill *active and *idle with current connection counts for show_http().
void http_server_stats(uint8_t *active, uint8_t *idle);

// Push the current device status to any connected /status_stream SSE client.
// No-op if no client is listening.  Call from any module on a state change
// (link up/down, DHCP bind, NTP sync, USB connect) to trigger an immediate
// update without the browser having to poll.
void http_status_push(void);

// Push accelerometer orientation and tap events to any connected
// /accel_stream SSE client.  No-op if no client is listening.
// pitch10 and roll10 are angles in tenths of a degree (e.g. 123 = 12.3°).
// tap is true for one call immediately after a tap is detected, then false.
void http_accel_push(int16_t pitch10, int16_t roll10, bool tap);

#endif // HTTP_SERVER_H
