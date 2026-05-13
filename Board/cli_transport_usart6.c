// cli_transport_usart6.c — USART6 / RS232 transport for TimbreOS CLI
//
// USART6 connects to the RS232 port on the extender board and is the
// default CLI transport — active from boot, no connection handshake needed.
//
// TX is interrupt-driven via TXE/TC flags in USART6_IRQHandler.
//
// RX uses DMA2 Stream1 Ch5 (circular, direct/no-FIFO) into a 256-byte ring
// buffer.  DMA HT and TC interrupts drain the buffer at the half-way and wrap
// points; the USART IDLE interrupt catches single bytes and short bursts.
//
// CubeMX configured: USART6 RX → DMA2 Stream1 Channel5, circular, byte width.
// If ever reassigned, update USART6_DMA_STREAM and the four flag macros below.
//
// Required wiring in stm32f4xx_it.c:
//   DMA2_Stream1_IRQHandler → usart6_dma_irq()
//   USART6_IRQHandler       → usart6_irq()  (IDLE + TX: TXE + TC flags)

#include <stdint.h>
#include <stdbool.h>

#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_dma.h"

#include "tea.h"
#include "cli.h"
#include "byteq.h"
#include "printers.h"

#include "cli_transport_usart6.h"

// ── DMA stream selection ──────────────────────────────────────────────────────
// USART6_RX maps to DMA2 Stream1 Ch5 or Stream2 Ch5.  CubeMX picks one;
// adjust these if your .ioc chose Stream2.

#define USART6_DMA              DMA2
#define USART6_DMA_STREAM       LL_DMA_STREAM_1

#define USART6_DMA_FLAG_HT_ACTIVE()     LL_DMA_IsActiveFlag_HT1(DMA2)
#define USART6_DMA_FLAG_TC_ACTIVE()     LL_DMA_IsActiveFlag_TC1(DMA2)
#define USART6_DMA_FLAG_HT_CLEAR()      LL_DMA_ClearFlag_HT1(DMA2)
#define USART6_DMA_FLAG_TC_CLEAR()      LL_DMA_ClearFlag_TC1(DMA2)

// ── Forward declarations ──────────────────────────────────────────────────────

static void usart6_rx_action(void);

// ── CLI input queue — filled by DMA drain, consumed by usart6_rx_action ──────
static BQUEUE(100, cliq);

// ── DMA RX ring buffer ────────────────────────────────────────────────────────

#define USART6_DMA_RX_SIZE  256u

static uint8_t  dma_rx_buf[USART6_DMA_RX_SIZE];
static uint32_t dma_rx_head = 0;

static inline uint32_t dma_rx_write_pos(void) {
    return USART6_DMA_RX_SIZE - LL_DMA_GetDataLength(USART6_DMA, USART6_DMA_STREAM);
}

static void usart6_dma_drain(void) {
    uint32_t wpos = dma_rx_write_pos();
    uint32_t head = dma_rx_head;

    if (head == wpos) return;

    bool was_empty = (qbq(cliq) == 0);

    if (wpos > head) {
        for (uint32_t i = head; i < wpos; i++)
            pushbq(dma_rx_buf[i], cliq);
    } else {
        for (uint32_t i = head; i < USART6_DMA_RX_SIZE; i++)
            pushbq(dma_rx_buf[i], cliq);
        for (uint32_t i = 0; i < wpos; i++)
            pushbq(dma_rx_buf[i], cliq);
    }

    dma_rx_head = wpos;

    if (was_empty && qbq(cliq))
        later(usart6_rx_action);
}

// ── usart6_dma_irq — call from DMA2_Stream1_IRQHandler ───────────────────────

void usart6_dma_irq(void) {
    if (USART6_DMA_FLAG_HT_ACTIVE()) {
        USART6_DMA_FLAG_HT_CLEAR();
        usart6_dma_drain();
    }
    if (USART6_DMA_FLAG_TC_ACTIVE()) {
        USART6_DMA_FLAG_TC_CLEAR();
        usart6_dma_drain();
    }
}

// ── EmitEvent target — kicks off interrupt-driven TX ─────────────────────────

static void usart6_emit(void) {
    if (qbq(emitq))
        LL_USART_EnableIT_TXE(USART6);
}

// ── usart6_tx_irq — call from USART6_IRQHandler when TXE flag + IT active ────

void usart6_tx_irq(void) {
    if (qbq(emitq)) {
        LL_USART_TransmitData8(USART6, pullbq(emitq));
    } else {
        LL_USART_DisableIT_TXE(USART6);
        LL_USART_EnableIT_TC(USART6);
    }
}

// ── usart6_tc_irq — call from USART6_IRQHandler when TC flag + IT active ─────

void usart6_tc_irq(void) {
    LL_USART_DisableIT_TC(USART6);
    LL_USART_ClearFlag_TC(USART6);
}

// ── usart6_rx_action — tea.c action, single-instance ─────────────────────────

static void usart6_rx_action(void) {
    when(EmitEvent, usart6_emit);
    autoEchoOff();

    while (qbq(cliq))  keyIn(pullbq(cliq));
    safe( if(qbq(cliq)) later(usart6_rx_action); )
}

// ── usart6_transport_init — call once from board init ────────────────────────
//
// Must be called after MX_DMA_Init() and MX_USART6_UART_Init().

void usart6_transport_init(void) {
    // STM32F4 DMA: all SxCR/NDTR/PAR/M0AR registers are write-protected while
    // EN=1.  A soft/debug reset can leave the stream enabled from the previous
    // run, so every write below would be silently ignored.  Disable first and
    // spin until the hardware clears EN (takes ≤ a few AHB cycles).
    LL_DMA_DisableStream(USART6_DMA, USART6_DMA_STREAM);
    while (LL_DMA_IsEnabledStream(USART6_DMA, USART6_DMA_STREAM)) {}

    // Clear any stale flags left over from the previous run.
    USART6_DMA_FLAG_HT_CLEAR();
    USART6_DMA_FLAG_TC_CLEAR();
    LL_DMA_ClearFlag_TE1(USART6_DMA);
    LL_DMA_ClearFlag_DME1(USART6_DMA);
    LL_DMA_ClearFlag_FE1(USART6_DMA);

    // CubeMX enables FIFO mode — disable for direct (byte-by-byte) transfer.
    LL_DMA_DisableFifoMode(USART6_DMA, USART6_DMA_STREAM);

    LL_DMA_SetPeriphAddress(USART6_DMA, USART6_DMA_STREAM, (uint32_t)&USART6->DR);
    LL_DMA_SetMemoryAddress(USART6_DMA, USART6_DMA_STREAM, (uint32_t)dma_rx_buf);
    LL_DMA_SetDataLength   (USART6_DMA, USART6_DMA_STREAM, USART6_DMA_RX_SIZE);

    LL_DMA_EnableIT_HT(USART6_DMA, USART6_DMA_STREAM);
    LL_DMA_EnableIT_TC(USART6_DMA, USART6_DMA_STREAM);

    LL_DMA_EnableStream(USART6_DMA, USART6_DMA_STREAM);
    LL_USART_EnableDMAReq_RX(USART6);

    // IDLE interrupt drains single bytes / short bursts before HT fires.
    LL_USART_ClearFlag_IDLE(USART6);
    LL_USART_EnableIT_IDLE(USART6);

    when(EmitEvent, usart6_emit);
    autoEchoOff();
    namedAction(usart6_rx_action);
}

// ── usart6_irq — call from USART6_IRQHandler ─────────────────────────────────
//
// Handles IDLE (→ drain DMA ring buffer) and TX (TXE + TC).

static Long pstatus = 0;

void usart6_irq(void) {
    Long status = USART6->SR;
    if ((status & USART_SR_IDLE) && LL_USART_IsEnabledIT_IDLE(USART6)) {
        (void)USART6->DR;   // SR already read above (step 1); DR read = step 2 clears IDLE
        usart6_dma_drain();
    }
    if ((status & USART_SR_TXE) && LL_USART_IsEnabledIT_TXE(USART6))
        usart6_tx_irq();
    if ((status & USART_SR_TC)  && LL_USART_IsEnabledIT_TC(USART6))
        usart6_tc_irq();
    pstatus |= status;
}
