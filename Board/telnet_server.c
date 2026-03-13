// telnet_server.c — LwIP raw TCP Telnet server, port 23
//
// Input routing:
//   Before each keyIn() call, EmitEvent is pointed at this connection's
//   tcp_write function so output is automatically directed back to the
//   sender.  keyEcho controls character echo — enabled for Telnet since
//   we negotiate WILL ECHO with the client.
//
// No session layer — TimbreOS CLI handles all parsing via run_cli.
// No heap — fixed connection pool.

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "tea.h"
#include "cli.h"
#include "byteq.h"
#include "printers.h"

#include "lwip/tcp.h"
#include "lwip/err.h"

#include "telnet_server.h"

// ── Configuration ─────────────────────────────────────────────────────────────

#define TELNET_PORT             23
#define TELNET_MAX_CONNECTIONS  2
#define TELNET_POLL_INTERVAL    10      // tcp_poll units (500 ms each = 5 s)

// ── Telnet protocol constants ─────────────────────────────────────────────────

#define IAC                     0xFF
#define WILL                    0xFB
#define WONT                    0xFC
#define DO                      0xFD
#define DONT                    0xFE
#define SB                      0xFA
#define SE                      0xF0
#define OPT_ECHO                0x01    // server echoes — client suppresses local echo
#define OPT_SGA                 0x03    // suppress go-ahead — character-at-a-time

static const uint8_t telnet_greeting[] = {
    IAC, WILL, OPT_ECHO,    // we will echo
    IAC, WILL, OPT_SGA,     // we suppress go-ahead
    IAC, DO,   OPT_SGA,     // please suppress go-ahead your side too
};

static const char prompt[] = "\r\nTimbreOS> ";

// ── IAC parser state ──────────────────────────────────────────────────────────

typedef enum {
    IAC_NORMAL,
    IAC_CMD,
    IAC_OPTION,
    IAC_SB,
    IAC_SB_IAC,
} iac_state_t;

// ── Connection pool ───────────────────────────────────────────────────────────

typedef struct {
    struct tcp_pcb  *pcb;
    iac_state_t      iac_state;
    bool             active;
} telnet_conn_t;

static telnet_conn_t  conns[TELNET_MAX_CONNECTIONS];
static struct tcp_pcb *listener;

// Current connection being processed — valid only during keyIn() call.
// Safe because actions run to completion with no interleaving.
static telnet_conn_t *current_conn;

// ── EmitEvent target — called by TimbreOS to drain emitq to this connection ──

static void telnet_emit(void) {
    if (!current_conn || !current_conn->pcb) return;

    while (qbq(emitq)) {
        uint8_t ch = pullbq(emitq);
        tcp_write(current_conn->pcb, &ch, 1, TCP_WRITE_FLAG_COPY);
    }
    tcp_output(current_conn->pcb);
}

// ── Pool helpers ──────────────────────────────────────────────────────────────

static telnet_conn_t *conn_alloc(void) {
    for (int i = 0; i < TELNET_MAX_CONNECTIONS; i++) {
        if (!conns[i].active) return &conns[i];
    }
    return NULL;
}

static void conn_free(telnet_conn_t *c) {
    if (current_conn == c) current_conn = NULL;
    c->pcb       = NULL;
    c->active    = false;
    c->iac_state = IAC_NORMAL;
}

static void conn_close(telnet_conn_t *c) {
    if (c->pcb == NULL) return;
    tcp_arg(c->pcb,  NULL);
    tcp_recv(c->pcb, NULL);
    tcp_sent(c->pcb, NULL);
    tcp_err(c->pcb,  NULL);
    tcp_poll(c->pcb, NULL, 0);
    tcp_close(c->pcb);
    conn_free(c);
}

// ── IAC state machine ─────────────────────────────────────────────────────────
// Returns true if byte is normal data to pass to keyIn().

static bool iac_process(telnet_conn_t *c, uint8_t byte) {
    switch (c->iac_state) {
        case IAC_NORMAL:
            if (byte == IAC) { c->iac_state = IAC_CMD; return false; }
            return true;

        case IAC_CMD:
            switch (byte) {
                case WILL: case WONT:
                case DO:   case DONT:
                    c->iac_state = IAC_OPTION;
                    return false;
                case SB:
                    c->iac_state = IAC_SB;
                    return false;
                case IAC:                       // escaped 0xFF — pass as data
                    c->iac_state = IAC_NORMAL;
                    return true;
                default:
                    c->iac_state = IAC_NORMAL;
                    return false;
            }

        case IAC_OPTION:
            c->iac_state = IAC_NORMAL;
            return false;

        case IAC_SB:
            if (byte == IAC) c->iac_state = IAC_SB_IAC;
            return false;

        case IAC_SB_IAC:
            c->iac_state = (byte == SE) ? IAC_NORMAL : IAC_SB;
            return false;
    }
    return false;
}

// ── LwIP callbacks ────────────────────────────────────────────────────────────

static err_t telnet_recv(void *arg, struct tcp_pcb *pcb,
                          struct pbuf *p, err_t err) {
    telnet_conn_t *c = (telnet_conn_t *)arg;

    if (!p || err != ERR_OK) {
        conn_close(c);
        return ERR_OK;
    }

    // Point EmitEvent and current_conn at this connection for the duration
    // of all keyIn() calls below.  Output automatically returns to sender.
    current_conn = c;
    when(EmitEvent, telnet_emit);
    autoEchoOn();                   // server echoes — we negotiated WILL ECHO

    struct pbuf *q = p;
    while (q) {
        uint8_t *buf = (uint8_t *)q->payload;
        for (uint16_t i = 0; i < q->len; i++) {
            uint8_t byte = buf[i];
            if (iac_process(c, byte)) {
                keyIn((char)byte);
            }
        }
        q = q->next;
    }

    tcp_recved(pcb, p->tot_len);
    pbuf_free(p);
    return ERR_OK;
}

static err_t telnet_sent(void *arg, struct tcp_pcb *pcb, uint16_t len) {
    (void)arg; (void)pcb; (void)len;
    return ERR_OK;
}

static err_t telnet_poll(void *arg, struct tcp_pcb *pcb) {
    (void)pcb;
    telnet_conn_t *c = (telnet_conn_t *)arg;
    if (c) conn_close(c);
    return ERR_OK;
}

static void telnet_err(void *arg, err_t err) {
    (void)err;
    telnet_conn_t *c = (telnet_conn_t *)arg;
    if (c) { c->pcb = NULL; conn_free(c); }
}

static err_t telnet_accept(void *arg, struct tcp_pcb *pcb, err_t err) {
    (void)arg;
    if (err != ERR_OK || pcb == NULL) return ERR_VAL;

    telnet_conn_t *c = conn_alloc();
    if (c == NULL) {
        static const char busy[] = "No sessions available.\r\n";
        tcp_write(pcb, busy, sizeof(busy) - 1, TCP_WRITE_FLAG_COPY);
        tcp_output(pcb);
        tcp_close(pcb);
        return ERR_OK;
    }

    c->pcb       = pcb;
    c->active    = true;
    c->iac_state = IAC_NORMAL;

    tcp_arg(c->pcb,  c);
    tcp_recv(c->pcb, telnet_recv);
    tcp_sent(c->pcb, telnet_sent);
    tcp_err(c->pcb,  telnet_err);
    tcp_poll(c->pcb, telnet_poll, TELNET_POLL_INTERVAL);
    tcp_setprio(c->pcb, TCP_PRIO_NORMAL);

    // Negotiate Telnet options then show prompt.
    tcp_write(pcb, telnet_greeting, sizeof(telnet_greeting),
              TCP_WRITE_FLAG_COPY);
    tcp_write(pcb, prompt, sizeof(prompt) - 1, TCP_WRITE_FLAG_COPY);
    tcp_output(pcb);

    return ERR_OK;
}

// ── Public API ────────────────────────────────────────────────────────────────

void telnet_server_init(void) {
    for (int i = 0; i < TELNET_MAX_CONNECTIONS; i++) {
        conns[i].active    = false;
        conns[i].pcb       = NULL;
        conns[i].iac_state = IAC_NORMAL;
    }
    current_conn = NULL;
    listener     = NULL;
}

void telnet_server_start(void) {
    if (listener != NULL) return;

    listener = tcp_new();
    if (listener == NULL) {
        print("TELNET: tcp_new failed\r\n");
        return;
    }

    tcp_bind(listener, IP_ADDR_ANY, TELNET_PORT);
    listener = tcp_listen(listener);

    if (listener == NULL) {
        print("TELNET: tcp_listen failed\r\n");
        return;
    }

    tcp_accept(listener, telnet_accept);
    print("TELNET: listening on port 23\r\n");
}

void telnet_server_stats(uint8_t *active, uint8_t *idle) {
    *active = 0;
    *idle   = 0;
    for (int i = 0; i < TELNET_MAX_CONNECTIONS; i++) {
        if (conns[i].active) (*active)++;
        else                 (*idle)++;
    }
}

void telnet_server_stop(void) {
    for (int i = 0; i < TELNET_MAX_CONNECTIONS; i++) {
        if (conns[i].active) conn_close(&conns[i]);
    }

    if (listener != NULL) {
        tcp_close(listener);
        listener = NULL;
    }

    print("TELNET: stopped\r\n");
}
