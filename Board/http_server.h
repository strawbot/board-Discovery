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
// gx1000 / gy1000 / gz1000 are normalised gravity components × 1000
// (e.g. 1000 = 1.0 g).  Y is up when the board is flat on a desk.
// tap is true for one call immediately after a tap is detected, then false.
void http_accel_push(int16_t gx1000, int16_t gy1000, int16_t gz1000, bool tap);

// Push a running-state change to any connected /accel_stream SSE client.
// Sends {"run":1} or {"run":0} so the browser can update the board colour
// immediately rather than waiting for the 2-second data-flow timeout.
void http_accel_state(bool running);

// Feed one (x,y,z) sample in g-units into the live-stream burst accumulator.
// Every GRAPH_LIVE_BURST (10) calls the accumulator flushes one SSE frame
// {"live":[x0,y0,z0,...]} to any connected /graph_stream client.
// Call from accel_batch_process() for every sample at the chip ODR.
// No-op (with accumulator reset) when no client is connected.
void http_graph_live_feed(float x, float y, float z);

// Push one 100-sample chunk of float g-unit data to any connected /graph_stream
// SSE client.  ch_idx: 0=X, 1=Y, 2=Z.  start: sample offset within the
// 1000-sample capture.  data: pointer to slice.  count: samples in slice.
// No-op if no client is connected.
void http_graph_push_chunk(uint8_t ch_idx, uint16_t start,
                           const float *data, uint16_t count);

// Send {"done":1} to the /graph_stream SSE client after the last chunk,
// signalling the browser to render all three channels.  No-op if no client.
void http_graph_done(void);

#endif // HTTP_SERVER_H
