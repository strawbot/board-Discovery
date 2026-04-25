#ifndef HTTP_STREAMS_H
#define HTTP_STREAMS_H

#include <stdint.h>
#include <stdbool.h>

// ── Discovery-specific HTTP SSE streams ───────────────────────────────────────
//
// This header declares the board-side push functions for the Discovery's
// application SSE streams.  The generic HTTP engine lives in Robot/net/http/.
//
// Call http_streams_init() from network_init() after http_server_init() to
// register the Discovery route table and SSE channels with the engine.

// Register Discovery routes and SSE channels with the Robot HTTP engine.
// Must be called after http_server_init() and before http_server_start().
void http_streams_init(void);

// Push the current device status to the /status_stream SSE client.
// No-op if no client is listening.  Call on any state change (link up/down,
// DHCP bind, NTP sync, USB connect) to trigger an immediate browser update.
void http_status_push(void);

// Push accelerometer orientation and tap events to /accel_stream.
// gx1000/gy1000/gz1000 are gravity components × 1000.  tap is true for
// one call immediately after a tap is detected.
void http_accel_push(int16_t gx1000, int16_t gy1000, int16_t gz1000, bool tap);

// Push a running-state change to /accel_stream.  Sends {"run":1} or {"run":0}.
void http_accel_state(bool running);

// Feed one (x,y,z) g-unit sample into the /graph_stream live-burst
// accumulator.  Flushes one SSE frame every GRAPH_LIVE_BURST (10) calls.
void http_graph_live_feed(float x, float y, float z);

// Push one 100-sample capture chunk to /graph_stream.
// ch_idx: 0=X, 1=Y, 2=Z.  start: sample offset.  data/count: float g-values.
void http_graph_push_chunk(uint8_t ch_idx, uint16_t start,
                           const float *data, uint16_t count);

// Send {"done":1} to /graph_stream after the last capture chunk.
void http_graph_done(void);

// Push one muscle wire live sample to /mw_stream.
// vsupply_v: supply volts.  r_wire_ohm: wire resistance.  pwm_pct: duty 0–100.
void http_mw_live_feed(float vsupply_v, float r_wire_ohm, float pwm_pct);

#endif // HTTP_STREAMS_H
