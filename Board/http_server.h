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

#endif // HTTP_SERVER_H
