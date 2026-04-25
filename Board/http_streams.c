// http_streams.c — Discovery-specific HTTP SSE streams
//
// Provides the board-side half of the HTTP server:
//   - Route table for Discovery's endpoints
//   - SSE channel instances (status, accel, graph, mw)
//   - Push functions called by other board modules
//   - http_streams_init() wires everything into the Robot HTTP engine
//
// TCP mechanics, connection lifecycle, and the terminal stream (/term_stream,
// /term_in) are handled entirely by Robot/net/http/http_server.c.

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "tea.h"
#include "printers.h"

#include "http_server.h"        // Robot engine API + types
#include "http_streams.h"
#include "http_content.h"       // resp_index (Discovery HTML)
#include "accel.h"              // accel_set_graph_raw()
#include "ntp_sync.h"           // ntp_get_utc(), ntp_is_synced()
#include "network_init.h"       // eth_status()
#include "usb_net.h"            // usb_netif

#include "lwip/netif.h"         // netif_ip4_addr()
#include "lwip/ip4_addr.h"      // ip4addr_ntoa()
#include "tusb.h"               // tud_connected(), tud_ready()

// ── SSE channel instances ─────────────────────────────────────────────────────

static http_sse_chan_t status_chan;
static http_sse_chan_t accel_chan;
static http_sse_chan_t graph_chan;
static http_sse_chan_t mw_chan;

// ── UTC epoch formatter ───────────────────────────────────────────────────────
// Self-contained: no gmtime(), no libc dependency.
// Handles 1970-01-01 through 2105-12-31.

static void epoch_to_utc_str(uint32_t epoch, char *buf, int bufsize)
{
    uint32_t s   = epoch % 60; epoch /= 60;
    uint32_t min = epoch % 60; epoch /= 60;
    uint32_t h   = epoch % 24; epoch /= 24;

    uint32_t days = epoch;
    uint32_t year = 1970;
    for (;;) {
        bool leap = (year % 4 == 0) && ((year % 100 != 0) || (year % 400 == 0));
        uint32_t diy = leap ? 366u : 365u;
        if (days < diy) break;
        days -= diy;
        year++;
    }

    static const uint8_t dim[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    bool leap = (year % 4 == 0) && ((year % 100 != 0) || (year % 400 == 0));
    uint32_t month = 0;
    for (month = 0; month < 12; month++) {
        uint32_t d = dim[month] + (uint32_t)(month == 1 && leap);
        if (days < d) break;
        days -= d;
    }

    snprintf(buf, (size_t)bufsize, "%04lu-%02lu-%02lu %02lu:%02lu:%02lu UTC",
             (unsigned long)year,
             (unsigned long)(month + 1u),
             (unsigned long)(days  + 1u),
             (unsigned long)h,
             (unsigned long)min,
             (unsigned long)s);
}

// ── Status JSON builder ───────────────────────────────────────────────────────

static int build_status_json(char *buf, int bufsize)
{
    char utc_str[32];
    uint32_t utc_epoch = ntp_get_utc();
    if (ntp_is_synced()) {
        epoch_to_utc_str(utc_epoch, utc_str, (int)sizeof(utc_str));
    } else {
        snprintf(utc_str, sizeof(utc_str), "not synced");
    }

    const char *eth = eth_status();

    char usb_ip[16];
    snprintf(usb_ip, sizeof(usb_ip), "%s", ip4addr_ntoa(netif_ip4_addr(&usb_netif)));
    char usb_str[40];
    if (tud_ready()) {
        snprintf(usb_str, sizeof(usb_str), "%s (host connected)", usb_ip);
    } else if (tud_connected()) {
        snprintf(usb_str, sizeof(usb_str), "%s (enumerating)", usb_ip);
    } else {
        snprintf(usb_str, sizeof(usb_str), "%s (no host)", usb_ip);
    }

    return snprintf(buf, (size_t)bufsize,
        "{"
        "\"uptime_s\":%lu,"
        "\"utc_time\":\"%s\","
        "\"utc_epoch\":%lu,"
        "\"ethernet\":\"%s\","
        "\"usb\":\"%s\""
        "}",
        HAL_GetTick() / 1000U,
        utc_str,
        (unsigned long)utc_epoch,
        eth,
        usb_str);
}

// ── fmt_g — float to 4 d.p. without float printf ─────────────────────────────

static int fmt_g(char *dst, int dsz, float v)
{
    int32_t  iv  = (int32_t)(v * 10000.0f + (v >= 0.0f ? 0.5f : -0.5f));
    int       neg = (iv < 0);
    uint32_t  uv  = (uint32_t)(neg ? -iv : iv);
    uint32_t  whl = uv / 10000u;
    uint32_t  frc = uv % 10000u;
    return snprintf(dst, (size_t)dsz,
                    neg ? "-%lu.%04lu" : "%lu.%04lu",
                    (unsigned long)whl, (unsigned long)frc);
}

// ── /status_stream push ───────────────────────────────────────────────────────

void http_status_push(void)
{
    char json[512];
    int  jlen = build_status_json(json, (int)sizeof(json));

    char frame[540];
    int  flen = snprintf(frame, sizeof(frame), "data: ");
    memcpy(frame + flen, json, (size_t)jlen);
    flen += jlen;
    frame[flen++] = '\n';
    frame[flen++] = '\n';

    http_sse_push(&status_chan, frame, (uint16_t)flen);
}

// 10-second heartbeat — keeps uptime and utc_epoch fresh in the browser.
void http_status_heartbeat(void)
{
    if (!status_chan.pcb) return;
    http_status_push();
    after(secs(10), http_status_heartbeat);
}

// ── /accel_stream pushes ──────────────────────────────────────────────────────

void http_accel_push(int16_t gx1000, int16_t gy1000, int16_t gz1000, bool tap)
{
    char frame[80];
    int n = snprintf(frame, sizeof(frame),
        "data: {\"x\":%d,\"y\":%d,\"z\":%d,\"t\":%d}\n\n",
        (int)gx1000, (int)gy1000, (int)gz1000, tap ? 1 : 0);
    http_sse_push(&accel_chan, frame, (uint16_t)n);
}

void http_accel_state(bool running)
{
    char frame[32];
    int n = snprintf(frame, sizeof(frame),
        "data: {\"run\":%d}\n\n", running ? 1 : 0);
    http_sse_push(&accel_chan, frame, (uint16_t)n);
}

// ── /graph_stream pushes ──────────────────────────────────────────────────────

#define GRAPH_LIVE_BURST 10u

static float   live_buf[3][GRAPH_LIVE_BURST];
static uint8_t live_fill = 0;

void http_graph_live_feed(float x, float y, float z)
{
    if (!graph_chan.pcb) {
        live_fill = 0;
        return;
    }

    live_buf[0][live_fill] = x;
    live_buf[1][live_fill] = y;
    live_buf[2][live_fill] = z;
    if (++live_fill < GRAPH_LIVE_BURST) return;
    live_fill = 0;

    char frame[320];
    int pos = snprintf(frame, sizeof(frame), "data: {\"live\":[");
    for (uint8_t i = 0; i < GRAPH_LIVE_BURST; i++) {
        if (i) { frame[pos++] = ','; }
        pos += fmt_g(frame + pos, (int)sizeof(frame) - pos, live_buf[0][i]);
        frame[pos++] = ',';
        pos += fmt_g(frame + pos, (int)sizeof(frame) - pos, live_buf[1][i]);
        frame[pos++] = ',';
        pos += fmt_g(frame + pos, (int)sizeof(frame) - pos, live_buf[2][i]);
    }
    pos += snprintf(frame + pos, (int)sizeof(frame) - pos, "]}\n\n");
    http_sse_push(&graph_chan, frame, (uint16_t)pos);
}

void http_graph_push_chunk(uint8_t ch_idx, uint16_t start,
                           const float *data, uint16_t count)
{
    static const char ch_names[] = "xyz";
    char ch = (ch_idx < 3u) ? ch_names[ch_idx] : '?';

    static char frame[900];
    int pos = snprintf(frame, sizeof(frame),
                       "data: {\"ch\":\"%c\",\"s\":%u,\"d\":[", ch, (unsigned)start);
    for (uint16_t i = 0; i < count; i++) {
        if (i) { frame[pos++] = ','; }
        pos += fmt_g(frame + pos, (int)sizeof(frame) - pos, data[i]);
    }
    pos += snprintf(frame + pos, (int)sizeof(frame) - pos, "]}\n\n");
    http_sse_push(&graph_chan, frame, (uint16_t)pos);
}

void http_graph_done(void)
{
    static const char frame[] = "data: {\"done\":1}\n\n";
    http_sse_push(&graph_chan, frame, (uint16_t)(sizeof(frame) - 1u));
}

// ── /mw_stream push ───────────────────────────────────────────────────────────

void http_mw_live_feed(float vsupply_v, float r_wire_ohm, float pwm_pct)
{
    char frame[80];
    int pos = snprintf(frame, sizeof(frame), "data: {\"live\":[");
    pos += fmt_g(frame + pos, (int)sizeof(frame) - pos, vsupply_v);
    frame[pos++] = ',';
    pos += fmt_g(frame + pos, (int)sizeof(frame) - pos, r_wire_ohm);
    frame[pos++] = ',';
    pos += fmt_g(frame + pos, (int)sizeof(frame) - pos, pwm_pct);
    pos += snprintf(frame + pos, (int)sizeof(frame) - pos, "]}\n\n");
    http_sse_push(&mw_chan, frame, (uint16_t)pos);
}

// ── Route handlers ────────────────────────────────────────────────────────────

static char            status_buf[640];
static http_response_t status_resp;

static const http_response_t *handle_status(const char *req, uint16_t len)
{
    (void)req; (void)len;
    char body[512];
    int blen = build_status_json(body, (int)sizeof(body));
    int hlen = snprintf(status_buf, sizeof(status_buf),
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n"
        "\r\n", blen);
    memcpy(status_buf + hlen, body, (size_t)blen);
    status_resp.data   = status_buf;
    status_resp.length = (uint32_t)(hlen + blen);
    return &status_resp;
}

static const http_response_t *handle_graph_mode(const char *req, uint16_t len)
{
    const char *body = strstr(req, "\r\n\r\n");
    if (body) {
        body += 4;
        accel_set_graph_raw(body[0] == 'r' && body[1] == 'a');
    }
    return &http_204;
}

// ── Route table ───────────────────────────────────────────────────────────────
// Terminal (/term_stream, /term_in) and SSE paths (/status_stream,
// /accel_stream, /graph_stream, /mw_stream) are omitted — handled
// automatically by the Robot engine via built-ins and http_sse_bind().

static const http_route_t discovery_routes[] = {
    { "GET",  "/",             &resp_index, NULL               },
    { "GET",  "/status.json",  NULL,        handle_status      },
    { "POST", "/graph_mode",   NULL,        handle_graph_mode  },
};
static const int discovery_route_count =
    (int)(sizeof(discovery_routes) / sizeof(discovery_routes[0]));

// ── http_streams_init ─────────────────────────────────────────────────────────
// Wires all Discovery-specific routes and SSE channels into the Robot engine.
// Call from network_init() after http_server_init(), before http_server_start().

// Called by the engine immediately after the /status_stream SSE upgrade.
// Pushes the current state so the browser doesn't wait for the first heartbeat,
// then arms the recurring 10-second heartbeat timer.
static void status_sse_connected(void)
{
    http_status_push();
    after(secs(10), http_status_heartbeat);
}

void http_streams_init(void)
{
    // Route table.
    http_server_set_routes(discovery_routes, discovery_route_count);

    // SSE channels — engine handles upgrade and chan->pcb lifecycle.
    // on_connect is called by the engine right after the SSE headers are sent.
    status_chan.on_connect = status_sse_connected;
    http_sse_bind("/status_stream", &status_chan);
    http_sse_bind("/accel_stream",  &accel_chan);
    http_sse_bind("/graph_stream",  &graph_chan);
    http_sse_bind("/mw_stream",     &mw_chan);

    // Named-action registration lets the scheduler display heartbeat in traces.
    namedAction(http_status_heartbeat);
}
