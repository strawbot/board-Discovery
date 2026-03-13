#ifndef TELNET_SERVER_H
#define TELNET_SERVER_H

// ── Public API ────────────────────────────────────────────────────────────────

// Call once from network_init() — sets up connection pool, no memory allocated.
void telnet_server_init(void);

// Call from link_callback when IP is confirmed — opens TCP listener port 23.
void telnet_server_start(void);

// Call from link_callback on link down — closes listener and all connections.
void telnet_server_stop(void);

// Fill *active and *idle with current connection counts for show_telnet().
void telnet_server_stats(uint8_t *active, uint8_t *idle);

#endif // TELNET_SERVER_H
