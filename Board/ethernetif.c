#include <string.h>
#include <stdbool.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_eth.h"

#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/pbuf.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "netif/etharp.h"

#include "tea.h"
#include "ethernetif.h"

// ── Configuration ─────────────────────────────────────────────────────────────

#define ETH_RX_DESC_COUNT       4
#define ETH_TX_DESC_COUNT       4
#define ETH_RX_BUFFER_SIZE      1524    // max Ethernet frame + alignment
#define ETH_TX_TIMEOUT          100     // ms — synchronous transmit timeout

#define DP83848_PHY_ADDRESS     0x01U

// DP83848 register map
#define DP83848_BCR             0x00U
#define DP83848_BSR             0x01U
#define DP83848_PHYSTS          0x10U
#define DP83848_MICR            0x11U
#define DP83848_MISR            0x12U

// PHYSTS bits
#define DP83848_PHYSTS_LINK_UP  (1U << 0)
#define DP83848_PHYSTS_SPEED_10 (1U << 1)
#define DP83848_PHYSTS_DUPLEX   (1U << 2)

// MICR bits
#define DP83848_MICR_INTEN      (1U << 1)
#define DP83848_MICR_INTOE      (1U << 0)

// MISR bits
#define DP83848_MISR_LINK_INT   (1U << 5)

// ── STM32F4 ETH DMA RX descriptor bits (hardware, version-independent) ────────
// These are hardware register bits, not HAL defines — safe across all HAL revisions.

#define RDES0_OWN       0x80000000U     // bit 31: 1=DMA, 0=CPU
#define RDES0_ES        0x00008000U     // bit 15: Error Summary
#define RDES0_LS        0x00000100U     // bit  8: Last Segment
#define RDES0_FS        0x00000200U     // bit  9: First Segment
#define RDES0_FL_MASK   0x3FFF0000U     // bits 29:16: Frame Length (includes FCS)
#define RDES0_FL_SHIFT  16
#define RDES1_RCH       0x00004000U     // bit 14: Second Address Chained (chained mode)

// ── Static allocations (no heap) ─────────────────────────────────────────────

static ETH_DMADescTypeDef rx_desc[ETH_RX_DESC_COUNT] __attribute__((aligned(4)));
static ETH_DMADescTypeDef tx_desc[ETH_TX_DESC_COUNT] __attribute__((aligned(4)));
static uint8_t            rx_buf[ETH_RX_DESC_COUNT][ETH_RX_BUFFER_SIZE] __attribute__((aligned(4)));

// TX buffer descriptors — one per pbuf fragment in a chain.
static ETH_BufferTypeDef  tx_buffers[ETH_TX_DESC_COUNT];

// RX read index — tracks next descriptor to consume, independent of heth.RxDescList.
static uint32_t rx_read_idx = 0;

// rx_alloc_idx used only by HAL_ETH_RxAllocCallback during HAL_ETH_Start_IT().
static uint32_t rx_alloc_idx = 0;

ETH_HandleTypeDef heth;         // not static — referenced by discovery_cli.c

// gnetif is defined in LWIP/App/lwip.c (CubeMX-generated).
// ethernetif.h declares it extern so all other files see it.
// We must NOT define it here — exclude LWIP/Target/ethernetif.c from the build.

// Forward declarations
static err_t ethernetif_output(struct netif *netif, struct pbuf *p);
extern void  lwip_timeout_action(void);     // defined in network_init.c

// ── Single-instance action state machine ─────────────────────────────────────

typedef enum {
    ETH_TASK_IDLE,
    ETH_TASK_QUEUED,
    ETH_TASK_RUNNING,
    ETH_TASK_REQUEUE,
} eth_task_state_t;

static volatile eth_task_state_t eth_task_state = ETH_TASK_IDLE;

typedef enum { LINK_DOWN, LINK_UP } link_state_t;
static link_state_t link_state = LINK_DOWN;

// ── RX buffer callback — called by HAL_ETH_Start_IT() if this HAL supports it ─
// If this HAL version does not call this weak symbol, we initialise descriptors
// explicitly in ethernetif_init() below — both paths end up with the same result.

void HAL_ETH_RxAllocCallback(ETH_HandleTypeDef *heth_cb, ETH_BufferTypeDef *buff) {
    (void)heth_cb;
    buff->buffer = rx_buf[rx_alloc_idx];
    buff->len    = ETH_RX_BUFFER_SIZE;
    buff->next   = NULL;
    rx_alloc_idx = (rx_alloc_idx + 1U) % ETH_RX_DESC_COUNT;
}

// ── PHY register access ───────────────────────────────────────────────────────

static uint16_t phy_read(uint16_t reg) {
    uint32_t val = 0;
    HAL_ETH_ReadPHYRegister(&heth, DP83848_PHY_ADDRESS, reg, &val);
    return (uint16_t)val;
}

static void phy_write(uint16_t reg, uint16_t val) {
    HAL_ETH_WritePHYRegister(&heth, DP83848_PHY_ADDRESS, reg, val);
}

// ── HAL RX complete callback ──────────────────────────────────────────────────

void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth_cb) {
    (void)heth_cb;
    switch (eth_task_state) {
        case ETH_TASK_IDLE:
            eth_task_state = ETH_TASK_QUEUED;
            later(eth_input_action);
            break;
        case ETH_TASK_RUNNING:
            eth_task_state = ETH_TASK_REQUEUE;
            break;
        case ETH_TASK_QUEUED:
        case ETH_TASK_REQUEUE:
            break;
    }
}

void HAL_ETH_TxCpltCallback(ETH_HandleTypeDef *heth_cb) {
    (void)heth_cb;
}

// ── PHY interrupt callback ────────────────────────────────────────────────────

void eth_phy_irq(void) {
    later(eth_link_action);
}

// ── ethernetif_init ───────────────────────────────────────────────────────────

err_t ethernetif_init(struct netif *netif) {
    // ── HAL ETH init ─────────────────────────────────────────────────────────
    heth.Instance            = ETH;
    heth.Init.MACAddr        = netif->hwaddr;
    heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
    heth.Init.TxDesc         = tx_desc;
    heth.Init.RxDesc         = rx_desc;
    heth.Init.RxBuffLen      = ETH_RX_BUFFER_SIZE;

    if (HAL_ETH_Init(&heth) != HAL_OK) {
        return ERR_IF;
    }

    // ── Explicitly initialise RX descriptor ring (chained mode) ──────────────
    // Some HAL revisions populate descriptors via HAL_ETH_RxAllocCallback;
    // others leave them uninitialised.  Writing them here is always correct and
    // idempotent — if the HAL callback already ran, we overwrite with the same
    // values.
    rx_read_idx  = 0;
    rx_alloc_idx = 0;
    for (uint32_t i = 0; i < ETH_RX_DESC_COUNT; i++) {
        rx_desc[i].DESC2 = (uint32_t)rx_buf[i];                              // buffer
        rx_desc[i].DESC3 = (uint32_t)&rx_desc[(i + 1U) % ETH_RX_DESC_COUNT];// next desc
        rx_desc[i].DESC1 = (ETH_RX_BUFFER_SIZE & 0x1FFFU) | RDES1_RCH;      // size + chain
        rx_desc[i].DESC0 = RDES0_OWN;                                         // DMA owns
    }
    __DSB();    // ensure descriptor writes are visible to DMA before start

    // ── PHY interrupt — link status change only ───────────────────────────────
    phy_write(DP83848_MICR, DP83848_MICR_INTEN | DP83848_MICR_INTOE);
    phy_write(DP83848_MISR, DP83848_MISR_LINK_INT);

    // ── Start ETH with interrupt-driven RX ───────────────────────────────────
    HAL_ETH_Start_IT(&heth);

    // ── LwIP netif fields ─────────────────────────────────────────────────────
    netif->name[0]    = 'e';
    netif->name[1]    = 'n';
    netif->output     = etharp_output;
    netif->linkoutput = ethernetif_output;
    netif->mtu        = 1500;
    netif->hwaddr_len = ETH_HWADDR_LEN;
    netif->flags      = NETIF_FLAG_BROADCAST
                      | NETIF_FLAG_ETHARP
                      | NETIF_FLAG_ETHERNET;

    return ERR_OK;
}

// ── ethernetif_output — LwIP calls this to transmit a frame ──────────────────

static err_t ethernetif_output(struct netif *netif, struct pbuf *p) {
    (void)netif;

    // Build ETH_BufferTypeDef chain from pbuf chain — no copy needed.
    uint32_t i = 0;
    for (struct pbuf *q = p; q != NULL && i < ETH_TX_DESC_COUNT; q = q->next, i++) {
        tx_buffers[i].buffer = q->payload;
        tx_buffers[i].len    = q->len;
        tx_buffers[i].next   = (q->next != NULL && i + 1U < ETH_TX_DESC_COUNT)
                               ? &tx_buffers[i + 1U] : NULL;
    }

    ETH_TxPacketConfig TxConfig = { 0 };
    TxConfig.Length   = p->tot_len;
    TxConfig.TxBuffer = tx_buffers;
    TxConfig.pData    = p;

    if (HAL_ETH_Transmit(&heth, &TxConfig, ETH_TX_TIMEOUT) != HAL_OK) {
        return ERR_IF;
    }
    return ERR_OK;
}

// ── ethernetif_input — direct DMA descriptor access ──────────────────────────
// HAL_ETH_GetRxDataBuffer / GetRxDataLength / BuildRxDescriptors are absent in
// this HAL revision, so we read the hardware descriptor directly.
//
// After a frame arrives the hardware clears RDES0_OWN (CPU now owns the
// descriptor).  We copy the data, restore the descriptor, set OWN again, and
// kick the DMA if it stopped waiting for a descriptor.
//
// Returns true if a frame was consumed (caller should loop until false).

bool ethernetif_input(struct netif *netif) {
    ETH_DMADescTypeDef *desc   = &rx_desc[rx_read_idx];
    uint32_t            status = desc->DESC0;

    // DMA still owns this descriptor — no frame ready.
    if (status & RDES0_OWN) {
        return false;
    }

    // Frame length from RDES0 bits [29:16]; hardware count includes 4-byte FCS.
    uint32_t framelength = (status & RDES0_FL_MASK) >> RDES0_FL_SHIFT;
    if (framelength > 4U) {
        framelength -= 4U;
    } else {
        framelength = 0U;
    }

    struct pbuf *p = NULL;

    // Only process frames with no error and at least one byte after stripping FCS.
    // We only handle single-descriptor frames (FS+LS both set); buffer is 1524 B
    // so any legal Ethernet frame fits in one descriptor.
    if (framelength > 0U
        && !(status & RDES0_ES)
        && (status & RDES0_FS)
        && (status & RDES0_LS))
    {
        p = pbuf_alloc(PBUF_RAW, (uint16_t)framelength, PBUF_POOL);
        if (p != NULL) {
            uint8_t *src = rx_buf[rx_read_idx]; // fixed mapping: desc[i] ↔ buf[i]
            for (struct pbuf *q = p; q != NULL; q = q->next) {
                memcpy(q->payload, src, q->len);
                src += q->len;
            }
        }
    }

    // ── Release descriptor back to DMA ───────────────────────────────────────
    // DESC1 (buffer size + RDES1_RCH) and DESC3 (next descriptor pointer) were
    // written during init and are not modified by the DMA on receive — leave them.
    desc->DESC2 = (uint32_t)rx_buf[rx_read_idx];   // restore buffer address
    __DSB();                                         // barrier before OWN write
    desc->DESC0 = RDES0_OWN;                         // give descriptor back to DMA
    __DSB();

    rx_read_idx = (rx_read_idx + 1U) % ETH_RX_DESC_COUNT;

    // Resume DMA receive if it stopped because it found no owned descriptor.
    if (ETH->DMASR & ETH_DMASR_RBUS) {
        ETH->DMASR  = ETH_DMASR_RBUS;  // clear flag (W1C)
        ETH->DMARPDR = 0U;              // any write resumes receive DMA
    }

    if (p != NULL) {
        if (netif->input(p, netif) != ERR_OK) {
            pbuf_free(p);
        }
    }

    return true;    // descriptor consumed — caller should check for more
}

// ── eth_input_action — tea.c action, single-instance ─────────────────────────

void eth_input_action(void) {
    eth_task_state = ETH_TASK_RUNNING;

    // Drain all available frames before checking requeue state.
    while (ethernetif_input(&gnetif)) { }

    switch (eth_task_state) {
        case ETH_TASK_REQUEUE:
            // Interrupt fired while we were running — more frames may have arrived.
            eth_task_state = ETH_TASK_QUEUED;
            later(eth_input_action);
            break;
        case ETH_TASK_RUNNING:
            eth_task_state = ETH_TASK_IDLE;
            break;
        default:
            break;
    }
}

// ── ethernet_link_check_state — stub for CubeMX-generated lwip.c ─────────────

void ethernet_link_check_state(struct netif *netif) {
    (void)netif;
}

// ── eth_link_action — tea.c action, posted by PHY interrupt ──────────────────

void eth_link_action(void) {
    (void)phy_read(DP83848_MISR);   // read clears interrupt

    uint16_t physts  = phy_read(DP83848_PHYSTS);
    bool     up      = (physts & DP83848_PHYSTS_LINK_UP)  != 0;
    bool     full_dx = (physts & DP83848_PHYSTS_DUPLEX)   != 0;
    bool     spd10   = (physts & DP83848_PHYSTS_SPEED_10) != 0;

    if (up && link_state == LINK_DOWN) {
        link_state = LINK_UP;

        // Update MAC speed and duplex to match autoneg result.
        ETH_MACConfigTypeDef MACConf = { 0 };
        HAL_ETH_GetMACConfig(&heth, &MACConf);
        MACConf.DuplexMode = full_dx ? ETH_FULLDUPLEX_MODE : ETH_HALFDUPLEX_MODE;
        MACConf.Speed      = spd10   ? ETH_SPEED_10M       : ETH_SPEED_100M;
        HAL_ETH_SetMACConfig(&heth, &MACConf);

        netif_set_link_up(&gnetif);

        uint32_t next = sys_timeouts_sleeptime();
        if (next != SYS_TIMEOUTS_SLEEPTIME_INFINITE) {
            after(next, lwip_timeout_action);
        }

    } else if (!up && link_state == LINK_UP) {
        link_state = LINK_DOWN;
        netif_set_link_down(&gnetif);
    }
}
