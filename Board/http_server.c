// http_server.c — LwIP raw TCP HTTP server, port 80
//
// Design:
//   - Fixed pool of connections, no heap.
//   - LwIP raw/callback API throughout — no blocking, no netconn.
//   - Entire HTTP response (headers + body) stored in flash as const char[].
//   - Responses streamed in tcp_sndbuf()-sized chunks via tcp_sent() callback.
//   - Idle connections closed after HTTP_CONN_TIMEOUT_MS with tcp_poll().
//   - All callbacks run in the action queue context (called from within LwIP,
//     which is driven by eth_input_action).

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "tea.h"
#include "printers.h"

#include "lwip/tcp.h"
#include "lwip/err.h"

#include "http_server.h"
#include "http_content.h"      // const char[] flash content + route table

// ── Configuration ─────────────────────────────────────────────────────────────

#define HTTP_PORT               80
#define HTTP_MAX_CONNECTIONS    4
#define HTTP_REQ_BUF_SIZE       1024    // enough for GET line + Host header
#define HTTP_CONN_TIMEOUT_MS    5000   // close idle connections after this long
#define HTTP_POLL_INTERVAL      5      // tcp_poll interval in 500 ms units (= 2.5 s)

// ── Connection state machine ──────────────────────────────────────────────────

typedef enum {
    HTTP_IDLE,          // slot free
    HTTP_RECEIVING,     // accumulating request
    HTTP_SENDING,       // streaming response
    HTTP_CLOSING,       // draining then closing
    HTTP_SSE,           // persistent SSE stream — never auto-closed
} http_state_t;

typedef struct {
    struct tcp_pcb  *pcb;
    http_state_t     state;
    char             req_buf[HTTP_REQ_BUF_SIZE];
    uint16_t         req_len;
    const char      *tx_ptr;           // current position in flash response
    uint32_t         tx_remaining;     // bytes left to send
} http_conn_t;

static http_conn_t conns[HTTP_MAX_CONNECTIONS];
static struct tcp_pcb *listener;

// ── Helpers ───────────────────────────────────────────────────────────────────

static void conn_free(http_conn_t *c) {
    c->pcb         = NULL;
    c->state       = HTTP_IDLE;
    c->req_len     = 0;
    c->tx_ptr      = NULL;
    c->tx_remaining = 0;
}

/* --- GET /status.json --- */
static char            status_buf[512];
static http_response_t status_resp;

static const http_response_t *handle_status(const char *req, uint16_t len)
{
    (void)req; (void)len;

    /* TODO: populate with real values — add fields as needed */
    char body[256];
    int blen = snprintf(body, sizeof(body),
        "{\"uptime_s\":%lu,\"ip\":\"192.168.x.x\"}",
        HAL_GetTick() / 1000U);

    int hlen = snprintf(status_buf, sizeof(status_buf),
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n"
        "\r\n", blen);
    memcpy(status_buf + hlen, body, blen);
    status_resp.data   = status_buf;
    status_resp.length = (uint32_t)(hlen + blen);
    return &status_resp;
}

/* --- GET /term_stream (Server-Sent Events) --- */

// Active SSE subscriber connection.  At most one at a time; a new GET
// /term_stream evicts and closes any existing one.
static http_conn_t *sse_conn = NULL;

// Sentinel returned by handle_term_sse() so http_recv() can recognise an SSE
// request without changing the handler signature.
static const http_response_t sse_sentinel = { NULL, 0 };

// EmitEvent target: drains emitq into an SSE event frame and writes it to the
// open /term_stream connection.  Newlines inside the payload are encoded as
// multiple "data:" fields per the SSE spec so they survive the browser parser.
// A trailing newline is preserved by appending an empty "data: " field, which
// prevents the SSE engine from stripping the final LF.
static void http_sse_emit(void)
{
    if (!sse_conn || !sse_conn->pcb) return;
    if (!qbq(emitq)) return;

    char buf[600];
    int  pos         = 0;
    bool last_was_nl = false;

    memcpy(buf, "data: ", 6); pos = 6;

    while (qbq(emitq) && pos < (int)sizeof(buf) - 16) {
        char ch = (char)pullbq(emitq);
        if (ch == '\r') continue;               // strip CR from CRLF pairs
        last_was_nl = (ch == '\n');
        if (ch == '\n') {
            buf[pos++] = '\n';
            if (qbq(emitq))                     // more bytes — extend event
                memcpy(buf + pos, "data: ", 6), pos += 6;
        } else {
            buf[pos++] = ch;
        }
    }

    // Preserve trailing newline: extra empty field stops SSE from stripping it.
    if (last_was_nl)
        memcpy(buf + pos, "data: \n", 7), pos += 7;

    buf[pos++] = '\n';  // blank line terminates the SSE event
    buf[pos++] = '\n';

    tcp_write(sse_conn->pcb, buf, (uint16_t)pos, TCP_WRITE_FLAG_COPY);
    tcp_output(sse_conn->pcb);
}

static const http_response_t *handle_term_sse(const char *req, uint16_t len)
{
    (void)req; (void)len;
    return &sse_sentinel;   // http_recv() handles connection promotion to SSE
}

/* --- POST /term_in --- */
static const http_response_t *handle_term_in(const char *req, uint16_t len)
{
    /* find \r\n\r\n separating headers from body */
    const char *body = NULL;
    for (uint16_t i = 0; i + 3 < len; i++) {
        if (req[i]=='\r' && req[i+1]=='\n' && req[i+2]=='\r' && req[i+3]=='\n') {
            body = req + i + 4;
            break;
        }
    }
    if (body) {
        when(EmitEvent, http_sse_emit);     // direct CLI output to SSE stream
        autoEchoOff();                      // web terminal handles its own echo
        int blen = (int)(len - (uint16_t)(body - req));
        for (int i = 0; i < blen; i++)
            keyIn((uint8_t)body[i]);
    }
    return &http_204;
}

static const http_route_t http_routes[] = {
    { "GET",  "/",             &resp_index, NULL            },
    { "GET",  "/status.json",  NULL,        handle_status   },
    { "GET",  "/term_stream",  NULL,        handle_term_sse },
    { "POST", "/term_in",      NULL,        handle_term_in  },
};
static const int http_route_count =
    (int)(sizeof(http_routes) / sizeof(http_routes[0]));


// Close connection cleanly — clear LwIP callbacks first to prevent re-entry.
static void conn_close(http_conn_t *c)
{
    if (c == sse_conn) sse_conn = NULL;   // un-register SSE subscriber first
    struct tcp_pcb *pcb = c->pcb;
    tcp_arg(pcb,  NULL);
    tcp_recv(pcb, NULL);
    tcp_sent(pcb, NULL);
    tcp_err(pcb,  NULL);
    mem_free(c);
    tcp_close(pcb);
}

// ── Request parsing ───────────────────────────────────────────────────────────
// Extracts the URL from "GET /path HTTP/1.x" and looks it up in the route table.
// Returns a pointer to the matching flash response, or the 404 response.

static const http_response_t *route_request(const char *req, uint16_t len)
{
    if (len < 14) return &http_404;

    const char *p = (const char *)memchr(req, ' ', len);
    if (!p) return &http_404;
    uint16_t mlen = (uint16_t)(p - req);

    const char *url = p + 1;
    const char *q   = (const char *)memchr(url, ' ', len - mlen - 1);
    if (!q) return &http_404;
    uint16_t ulen = (uint16_t)(q - url);

    for (int i = 0; i < http_route_count; i++) {
        const http_route_t *r = &http_routes[i];
        if (strlen(r->method) == mlen && strncmp(r->method, req, mlen) == 0 &&
            strlen(r->url)    == ulen && strncmp(r->url,    url, ulen) == 0) {
        	// maybeCr(), printDec(len), printAsciiString(req);
            return r->handler ? r->handler(req, len) : r->response;
        }
    }
    return &http_404;
}

// ── Response streaming ────────────────────────────────────────────────────────
// Sends as much of the flash response as tcp_sndbuf() allows.
// Called both when we first have a response and from tcp_sent() as space frees.

static void send_chunk(http_conn_t *c) {
    while (c->tx_remaining > 0) {
        uint16_t space = tcp_sndbuf(c->pcb);
        if (space == 0) break;

        uint16_t chunk = (uint16_t)(c->tx_remaining < space ? c->tx_remaining : space);
        uint8_t  flags = TCP_WRITE_FLAG_COPY;
        if (c->tx_remaining > chunk) flags |= TCP_WRITE_FLAG_MORE;

        err_t err = tcp_write(c->pcb, c->tx_ptr, chunk, flags);
        if (err != ERR_OK) break;

        c->tx_ptr       += chunk;
        c->tx_remaining -= chunk;
    }

    tcp_output(c->pcb);   /* flush — data is in RAM pbuf, DMA is fine */

    if (c->tx_remaining == 0) {
        c->state = HTTP_CLOSING;
    }
}

// ── LwIP callbacks ────────────────────────────────────────────────────────────
volatile uint32_t dbg_http_recv = 0;

static err_t http_recv(void *arg, struct tcp_pcb *pcb,
                        struct pbuf *p, err_t err) {
    dbg_http_recv++;
    http_conn_t *c = (http_conn_t *)arg;

    if (!p || err != ERR_OK) {
        // Connection closed by client or error (conn_close clears sse_conn).
        conn_close(c);
        return ERR_OK;
    }

    // Accumulate into request buffer — we only need the first line.
    if (c->state == HTTP_RECEIVING) {
        struct pbuf *q = p;
        while (q && c->req_len < HTTP_REQ_BUF_SIZE - 1) {
            uint16_t copy = (uint16_t)(HTTP_REQ_BUF_SIZE - 1 - c->req_len);
            if (copy > q->len) copy = q->len;
            memcpy(c->req_buf + c->req_len, q->payload, copy);
            c->req_len += copy;
            q = q->next;
        }
        c->req_buf[c->req_len] = '\0';


        const char *hdr_end = strstr(c->req_buf, "\r\n\r\n");
        if (hdr_end) {
            uint16_t header_len = (uint16_t)(hdr_end - c->req_buf) + 4;

            /* find Content-Length (POST body size; 0 for GETs) */
            uint32_t content_length = 0;
            const char *cl = strstr(c->req_buf, "Content-Length: ");
            if (cl)
                content_length = (uint32_t)strtoul(cl + 16, NULL, 10);

            /* wait until headers + body are both in the buffer */
            if (c->req_len < header_len + content_length)
                goto done;   /* or just fall through to tcp_recved below */

            /* full request is assembled — now dispatch */
            const http_response_t *resp = route_request(c->req_buf, c->req_len);

            if (resp == &sse_sentinel) {
                // Promote connection to persistent SSE stream.
                // Evict any existing subscriber first.
                if (sse_conn && sse_conn->pcb) conn_close(sse_conn);
                sse_conn = c;
                c->state = HTTP_SSE;
                tcp_poll(c->pcb, NULL, 0);      // disable idle timeout

                static const char sse_hdr[] =
                    "HTTP/1.1 200 OK\r\n"
                    "Content-Type: text/event-stream\r\n"
                    "Cache-Control: no-cache\r\n"
                    "Connection: keep-alive\r\n"
                    "\r\n";
                tcp_write(c->pcb, sse_hdr, sizeof(sse_hdr) - 1,
                          TCP_WRITE_FLAG_COPY);
                tcp_output(c->pcb);
                // when(EmitEvent, http_sse_emit);
            } else {
                c->tx_ptr       = resp->data;
                c->tx_remaining = resp->length;
                c->state        = HTTP_SENDING;
                send_chunk(c);
            }
        }
    done:;
    }

    tcp_recved(pcb, p->tot_len);
    pbuf_free(p);

    /* if entire response fit in one send_chunk call, close now */
    if (c->state == HTTP_CLOSING) {
        conn_close(c);          /* c is not touched after this */
    }

    return ERR_OK;
}

static err_t http_sent(void *arg, struct tcp_pcb *pcb, u16_t len)
{
    (void)pcb; (void)len;
    http_conn_t *c = (http_conn_t *)arg;
    if (!c) return ERR_OK;

    if (c->state == HTTP_SENDING) {
        send_chunk(c);          /* room freed → queue more */
    }
    if (c->state == HTTP_CLOSING) {
        conn_close(c);          /* all ACKed → close cleanly */
    }
    return ERR_OK;
}

static err_t http_poll(void *arg, struct tcp_pcb *pcb) {
    (void)pcb;
    http_conn_t *c = (http_conn_t *)arg;
    // SSE connections are persistent — only close idle request/response ones.
    if (c && c->state != HTTP_SSE) conn_close(c);
    return ERR_OK;
}

static void http_err(void *arg, err_t err) {
    (void)err;
    http_conn_t *c = (http_conn_t *)arg;
    // PCB is already freed by LwIP when err is called — just release our slot.
    if (c) {
        if (c == sse_conn) sse_conn = NULL;   // un-register before zeroing pcb
        c->pcb = NULL;
        conn_free(c);
    }
}
static err_t http_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    if (err != ERR_OK) return err;

    http_conn_t *c = (http_conn_t *)mem_malloc(sizeof(http_conn_t));
    if (!c) {
        tcp_abort(newpcb);
        return ERR_MEM;
    }

    memset(c, 0, sizeof(http_conn_t));   /* ← clears all stale fields */

    c->pcb   = newpcb;
    c->state = HTTP_RECEIVING;
    /* req_len, tx_ptr, tx_remaining are all 0 from memset */

    tcp_arg(newpcb,  c);
    tcp_recv(newpcb, http_recv);
    tcp_sent(newpcb, http_sent);
    tcp_err(newpcb,  http_err);

    return ERR_OK;
}

// ── Public API ────────────────────────────────────────────────────────────────

void http_server_init(void) {
    for (int i = 0; i < HTTP_MAX_CONNECTIONS; i++) {
        conn_free(&conns[i]);
    }
    listener = NULL;
}

void http_server_start(void) {
    if (listener != NULL) return;           // already started

    listener = tcp_new();
    if (listener == NULL) {
        print("HTTP: tcp_new failed\r\n");
        return;
    }

    tcp_bind(listener, IP_ADDR_ANY, HTTP_PORT);
    listener = tcp_listen(listener);

    if (listener == NULL) {
        print("HTTP: tcp_listen failed\r\n");
        return;
    }

    tcp_accept(listener, http_accept);
    print("HTTP: listening on port 80\r\n");
}

void http_server_stats(uint8_t *active, uint8_t *idle) {
    *active = 0;
    *idle   = 0;
    for (int i = 0; i < HTTP_MAX_CONNECTIONS; i++) {
        if (conns[i].state == HTTP_IDLE) (*idle)++;
        else                             (*active)++;
    }
}

void http_server_stop(void) {
    sse_conn = NULL;    // cleared before conn_close loop to avoid double-clear
    // Close all active connections.
    for (int i = 0; i < HTTP_MAX_CONNECTIONS; i++) {
        if (conns[i].state != HTTP_IDLE) {
            conn_close(&conns[i]);
        }
    }

    // Close the listener.
    if (listener != NULL) {
        tcp_close(listener);
        listener = NULL;
    }

    print("HTTP: stopped\r\n");
}
