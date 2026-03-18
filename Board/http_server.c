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

// Close connection cleanly — clear LwIP callbacks first to prevent re-entry.
static void conn_close(http_conn_t *c)
{
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
        // Connection closed by client or error.
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

        // Detect end of HTTP request headers (blank line).
        if (strstr(c->req_buf, "\r\n\r\n") ||
            strstr(c->req_buf, "\n\n")) {

            const http_response_t *resp = route_request(c->req_buf, c->req_len);
            c->tx_ptr       = resp->data;
            c->tx_remaining = resp->length;
            c->state        = HTTP_SENDING;
            send_chunk(c);
        }
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
    http_conn_t *c = (http_conn_t *)arg;
    // Connection has been idle too long — close it.
    if (c) conn_close(c);
    return ERR_OK;
}

static void http_err(void *arg, err_t err) {
    (void)err;
    http_conn_t *c = (http_conn_t *)arg;
    // PCB is already freed by LwIP when err is called — just release our slot.
    if (c) {
        c->pcb = NULL;
        conn_free(c);
    }
}
static err_t http_accept(void *arg, struct tcp_pcb *pcb, err_t err) {
    (void)arg;
    if (err != ERR_OK) return err;
    http_conn_t *c = (http_conn_t *)mem_malloc(sizeof(http_conn_t));
    if (!c) {
        tcp_abort(pcb);
        return ERR_MEM;
    }

    c->pcb   = pcb;
    c->state = HTTP_RECEIVING;

    tcp_arg(c->pcb,  c);
    tcp_recv(c->pcb, http_recv);
    tcp_sent(c->pcb, http_sent);
    tcp_err(c->pcb,  http_err);
    tcp_poll(c->pcb, http_poll, HTTP_POLL_INTERVAL);
    tcp_setprio(c->pcb, TCP_PRIO_MIN);   // yield to Telnet/other traffic

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
