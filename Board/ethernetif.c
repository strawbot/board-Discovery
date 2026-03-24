#include <string.h>
#include <stdbool.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_eth.h"
#include "stm32f4xx_hal_gpio.h"

#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/pbuf.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "netif/etharp.h"

#include "tea.h"
#include "printers.h"
#include "ethernetif.h"

// ── Configuration ─────────────────────────────────────────────────────────────

#define ETH_RX_DESC_COUNT       4
#define ETH_TX_DESC_COUNT       4
#define ETH_RX_BUFFER_SIZE      1524    // max Ethernet frame + alignment
#define ETH_TX_TIMEOUT          100     // ms — synchronous transmit timeout

// LAN8720 PHY address — set by PHYAD strap pin on your board (0x00 or 0x01).
#define LAN8720_PHY_ADDRESS     0x00U

// LAN8720 hardware reset GPIO — PE2 drives nRST (active LOW).
// Minimum assert time: 100 µs (we use 1 ms).
// Time from deassert to first MDIO access: ≥ 25 ms (we use 25 ms).
#define PHY_NRST_PORT           GPIOE
#define PHY_NRST_PIN            GPIO_PIN_2
#define PHY_NRST_ASSERT_MS      1U      // hold low
#define PHY_NRST_RECOVER_MS     25U     // wait after deassert before MDIO

// LAN8720 register map
#define LAN8720_BCR             0x00U   // Basic Control
#define LAN8720_BCR_SOFT_RESET  (1U << 15) // BCR bit 15: software reset (self-clearing)
#define LAN8720_BCR_AUTONEG_EN  (1U << 12) // BCR bit 12: auto-negotiation enable
#define LAN8720_BCR_RESTART_AN  (1U <<  9) // BCR bit  9: restart auto-negotiation
#define LAN8720_BSR             0x01U   // Basic Status
#define LAN8720_ISF             0x1DU   // Interrupt Source Flag  (read-to-clear)
#define LAN8720_IMR             0x1EU   // Interrupt Mask Register
#define LAN8720_SCSR            0x1FU   // Special Control/Status (speed + duplex)

// BSR bits
#define LAN8720_BSR_LINK_UP     (1U << 2)   // Link status

// SCSR bits [4:2]: HCDSPEED — speed/duplex after autoneg
#define LAN8720_SCSR_SPEED_MASK 0x001CU     // bits [4:2]
#define LAN8720_SCSR_10_HALF    (1U << 2)   // 001
#define LAN8720_SCSR_100_HALF   (2U << 2)   // 010
#define LAN8720_SCSR_10_FULL    (5U << 2)   // 101
#define LAN8720_SCSR_100_FULL   (6U << 2)   // 110


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

// Diagnostic counters — readable via show_ethernet().
volatile uint32_t eth_rx_irq_count      = 0;  // incremented by HAL_ETH_RxCpltCallback
volatile uint32_t eth_rx_frame_count    = 0;  // incremented each frame consumed by ethernetif_input
volatile uint32_t eth_rx_input_err      = 0;  // netif->input returned non-ERR_OK
volatile uint32_t eth_rx_etype_arp      = 0;  // EtherType 0x0806 (ARP) frames received
volatile uint32_t eth_rx_etype_ip       = 0;  // EtherType 0x0800 (IPv4) frames received
volatile uint32_t eth_rx_etype_other    = 0;  // EtherType not ARP or IPv4
volatile uint32_t eth_tx_call_count     = 0;  // incremented every call to ethernetif_output
volatile uint32_t eth_tx_ok_count       = 0;  // HAL_ETH_Transmit returned HAL_OK
volatile uint32_t eth_tx_err_count      = 0;  // HAL_ETH_Transmit returned HAL_ERROR
volatile uint32_t eth_tx_timeout_count  = 0;  // HAL_ETH_Transmit returned HAL_TIMEOUT
volatile uint32_t eth_tx_arp_count      = 0;  // outgoing frames with EtherType 0x0806 (ARP)
volatile uint32_t eth_tx_ip_count       = 0;  // outgoing frames with EtherType 0x0800 (IP)
uint8_t           eth_tx_last_ip_dst[6] = {0}; // dest MAC of most recent outgoing IP frame
volatile uint32_t dbg_eth_input_entry   = 0;  // increments if ethernet_input is reached at all
volatile uint32_t dbg_eth_input_lendrop = 0;  // p->len <= SIZEOF_ETH_HDR
volatile uint32_t dbg_eth_input_hdrdrop = 0;  // pbuf_remove_header failed
volatile uint32_t dbg_eth_input_nodispatch = 0; // ethertype fell through switch

// rx_alloc_idx used only by HAL_ETH_RxAllocCallback during HAL_ETH_Start_IT().
static uint32_t rx_alloc_idx = 0;

// ── PHY / MAC watchdog and recovery state ─────────────────────────────────────
// eth_tx_fail_streak  — increments when TX fails while link_state==LINK_UP;
//                       reset to 0 on any successful TX or on recovery.
// ETH_TX_FAIL_THRESH  — number of consecutive failures that triggers recovery.
// eth_recovery_pending — true while recovery is queued in the action loop.
// eth_recovery_count  — total number of recovery attempts since boot.
// eth_recovery_last_ms — HAL_GetTick() timestamp of most recent recovery.

#define ETH_TX_FAIL_THRESH      3u

static          uint8_t  eth_tx_fail_streak   = 0;
static          bool     eth_recovery_pending = false;
volatile uint32_t eth_recovery_count    = 0;
volatile uint32_t eth_recovery_last_ms  = 0;

ETH_HandleTypeDef heth;         // not static — referenced by discovery_cli.c

// Saved init results — readable from show_ethernet() after USART6 is ready.
// 0xFF means "not yet called".
HAL_StatusTypeDef eth_init_rc     = (HAL_StatusTypeDef)0xFF;
HAL_StatusTypeDef eth_start_rc    = (HAL_StatusTypeDef)0xFF;
uint8_t           eth_init_gstate  = 0;
uint32_t          eth_init_error   = 0;
uint8_t           eth_start_gstate = 0;

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
    HAL_ETH_ReadPHYRegister(&heth, LAN8720_PHY_ADDRESS, reg, &val);
    return (uint16_t)val;
}

static void phy_write(uint16_t reg, uint16_t val) __attribute__((unused));
static void phy_write(uint16_t reg, uint16_t val) {
    HAL_ETH_WritePHYRegister(&heth, LAN8720_PHY_ADDRESS, reg, (uint32_t)val);
}

// ── HAL RX complete callback ──────────────────────────────────────────────────

void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth_cb) {
    (void)heth_cb;
    eth_rx_irq_count++;
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


// ── HAL_ETH_MspInit ───────────────────────────────────────────────────────────
// CubeMX normally generates this inside LWIP/Target/ethernetif.c.  Since we
// exclude that file (to avoid duplicate symbol errors), we provide it here.
//
// RMII pinout for STM32F4-Discovery + LAN8720 extender board:
//   PA1  ETH_REF_CLK   — AF11 = ETH_RMII_REF_CLK  (50 MHz from LAN8720 REFCLK0)
//   PA2  ETH_MDIO      — AF11 = ETH_MDIO
//   PA7  ETH_CRS_DV    — AF11 = ETH_RMII_CRS_DV
//   PB11 ETH_TX_EN     — AF11 = ETH_RMII_TX_EN
//   PB12 ETH_TXD0      — AF11 = ETH_RMII_TXD0
//   PB13 ETH_TXD1      — AF11 = ETH_RMII_TXD1
//   PC1  ETH_MDC       — AF11 = ETH_MDC
//   PC4  ETH_RXD0      — AF11 = ETH_RMII_RXD0
//   PC5  ETH_RXD1      — AF11 = ETH_RMII_RXD1
//
// Verified against .ioc: PB11=TX_EN, PB12=TXD0, PB13=TXD1.

void HAL_ETH_MspInit(ETH_HandleTypeDef *hethMsp) {
    if (hethMsp->Instance != ETH) { return; }

    // ── Enable peripheral clocks ──────────────────────────────────────────────
    __HAL_RCC_ETHMAC_CLK_ENABLE();      // ETH MAC
    __HAL_RCC_ETHMACTX_CLK_ENABLE();   // ETH TX
    __HAL_RCC_ETHMACRX_CLK_ENABLE();   // ETH RX

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();      // PE2 = PHY nRST
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    // ── Select RMII (not MII) via SYSCFG_PMC ─────────────────────────────────
    SET_BIT(SYSCFG->PMC, SYSCFG_PMC_MII_RMII_SEL);

    // ── Configure all RMII pins as AF11, no pull, very-high speed ────────────
    GPIO_InitTypeDef gpio = { 0 };
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF11_ETH;

    // PA1 (REF_CLK), PA2 (MDIO), PA7 (CRS_DV)
    gpio.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &gpio);

    // PB11 (TX_EN), PB12 (TXD0), PB13 (TXD1)
    gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
    HAL_GPIO_Init(GPIOB, &gpio);

    // PC1 (MDC), PC4 (RXD0), PC5 (RXD1)
    gpio.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOC, &gpio);

    // ── PE2 — LAN8720 nRST (active LOW) ──────────────────────────────────────
    // Drive HIGH first to avoid glitching the reset line during reconfiguration,
    // then init the pin.  WritePin before Init is safe: BSRR is always writable.
    HAL_GPIO_WritePin(PHY_NRST_PORT, PHY_NRST_PIN, GPIO_PIN_SET);
    GPIO_InitTypeDef rst_gpio = { 0 };
    rst_gpio.Pin   = PHY_NRST_PIN;
    rst_gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    rst_gpio.Pull  = GPIO_NOPULL;
    rst_gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PHY_NRST_PORT, &rst_gpio);

    // ── Enable ETH interrupt ──────────────────────────────────────────────────
    HAL_NVIC_SetPriority(ETH_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(ETH_IRQn);
}

// ── eth_rx_ring_init — reset the RX DMA descriptor ring ──────────────────────
// Called once during ethernetif_init() and again during PHY/MAC recovery.
// Must be called after HAL_ETH_Init() so that heth.Init.RxDesc is populated.

static void eth_rx_ring_init(void) {
    rx_read_idx  = 0;
    rx_alloc_idx = 0;
    for (uint32_t i = 0; i < ETH_RX_DESC_COUNT; i++) {
        rx_desc[i].DESC2 = (uint32_t)rx_buf[i];
        rx_desc[i].DESC3 = (uint32_t)&rx_desc[(i + 1U) % ETH_RX_DESC_COUNT];
        rx_desc[i].DESC1 = (ETH_RX_BUFFER_SIZE & 0x1FFFU) | RDES1_RCH;
        rx_desc[i].DESC0 = RDES0_OWN;
    }
    __DSB();    // ensure writes are visible to DMA before start
}

// ── ethernetif_init ───────────────────────────────────────────────────────────
volatile uint8_t dbg_hwaddr[6];
// Also capture the destination MAC of a received ICMP frame:
volatile uint8_t dbg_icmp_dst_mac[6];

err_t ethernetif_init(struct netif *netif) {
    // ── MAC address ───────────────────────────────────────────────────────────
    // CubeMX sets the MAC inside the excluded LWIP/Target/ethernetif.c.
    // We must set it here so HAL_ETH_Init programs it into hardware.
    // 0x02 = locally-administered unicast.  Change to your real OUI if needed.
    static const uint8_t default_mac[ETH_HWADDR_LEN] = {
        0x02, 0x00, 0x00, 0x00, 0x00, 0x01
    };
    netif->hwaddr_len = ETH_HWADDR_LEN;
    memcpy(netif->hwaddr, default_mac, ETH_HWADDR_LEN);

    // ── Boot-time PHY hardware reset ──────────────────────────────────────────
    // HAL_ETH_MspInit (called by HAL_ETH_Init below) configures PE2 and drives
    // it HIGH, but it does NOT pulse nRST.  An explicit pulse here guarantees
    // the PHY starts from a known clean state regardless of any prior activity
    // (e.g. debugger resets that left the PHY in a partial state).
    // PE2 must already be clocked before this point — MspInit will be called by
    // HAL_ETH_Init, which runs first for the very first boot.  For idempotency
    // we enable the clock and configure the pin here before using it.
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitTypeDef boot_rst = { 0 };
    boot_rst.Pin   = PHY_NRST_PIN;
    boot_rst.Mode  = GPIO_MODE_OUTPUT_PP;
    boot_rst.Pull  = GPIO_NOPULL;
    boot_rst.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PHY_NRST_PORT, &boot_rst);
    HAL_GPIO_WritePin(PHY_NRST_PORT, PHY_NRST_PIN, GPIO_PIN_RESET);
    HAL_Delay(PHY_NRST_ASSERT_MS);
    HAL_GPIO_WritePin(PHY_NRST_PORT, PHY_NRST_PIN, GPIO_PIN_SET);
    HAL_Delay(PHY_NRST_RECOVER_MS);

    // ── HAL ETH init ─────────────────────────────────────────────────────────
    heth.Instance            = ETH;
    heth.Init.MACAddr        = netif->hwaddr;
    heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
    heth.Init.TxDesc         = tx_desc;
    heth.Init.RxDesc         = rx_desc;
    heth.Init.RxBuffLen      = ETH_RX_BUFFER_SIZE;

    eth_init_rc     = HAL_ETH_Init(&heth);
    eth_init_gstate = heth.gState;
    eth_init_error  = heth.ErrorCode;
    if (eth_init_rc != HAL_OK) {
        return ERR_IF;
    }

    // ── Explicitly initialise RX descriptor ring (chained mode) ──────────────
    // Some HAL revisions populate descriptors via HAL_ETH_RxAllocCallback;
    // others leave them uninitialised.  eth_rx_ring_init() is always correct
    // and idempotent — if the HAL callback already ran we overwrite with the
    // same values.
    eth_rx_ring_init();

    // ── PHY link polling ──────────────────────────────────────────────────────
    // LAN8720 NINT/REFCLK0 (PA1) is used as ETH_REF_CLK — the 50 MHz RMII
    // reference clock.  The interrupt output is therefore not available.
    // Link state is detected by polling BSR once per second via eth_link_action.

    // ── Start ETH with interrupt-driven RX ───────────────────────────────────
    eth_start_rc     = HAL_ETH_Start_IT(&heth);
    eth_start_gstate = heth.gState;

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

                      // In ethernetif_init or after netif_add:
memcpy(dbg_hwaddr, netif->hwaddr, 6);

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
    TxConfig.Attributes   = ETH_TX_PACKETS_FEATURES_CSUM;
    TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;

    eth_tx_call_count++;
    // Classify outgoing frame by EtherType (bytes 12-13 of the pbuf payload).
    // The pbuf passed to linkoutput already has the Ethernet header prepended.
    if (p->len >= 14U) {
        const uint8_t *b = (const uint8_t *)p->payload;
        uint16_t etype = ((uint16_t)b[12] << 8) | b[13];
        if      (etype == 0x0806U) eth_tx_arp_count++;
        else if (etype == 0x0800U) {
            eth_tx_ip_count++;
            memcpy((void *)eth_tx_last_ip_dst, b, 6U); // first 6 bytes = dest MAC
        }
    }
    HAL_StatusTypeDef tx_rc = HAL_ETH_Transmit(&heth, &TxConfig, ETH_TX_TIMEOUT);
    if (tx_rc == HAL_OK) {
        eth_tx_ok_count++;
        eth_tx_fail_streak = 0;     // clear watchdog streak on any success
        return ERR_OK;
    } else if (tx_rc == HAL_TIMEOUT) {
        eth_tx_timeout_count++;
    } else {
        eth_tx_err_count++;
    }

    // Consecutive TX failures while link appears up → schedule PHY/MAC recovery.
    // Only arm once per episode (eth_recovery_pending guards against repeat queuing).
    if (link_state == LINK_UP) {
        eth_tx_fail_streak++;
        if (eth_tx_fail_streak >= ETH_TX_FAIL_THRESH && !eth_recovery_pending) {
            eth_recovery_pending = true;
            later(eth_recovery_action);
        }
    }
    return ERR_IF;
}

// Add to your IP frame debug variables:
volatile uint16_t eth_rx_pbuf_tot_len = 0;
volatile uint16_t eth_rx_ip_hdr_len_field = 0;
volatile uint8_t  eth_rx_last_ip_proto = 0;   // IP protocol byte
volatile uint8_t  eth_rx_last_src_ip[4] = {0}; // source IP
volatile uint8_t  eth_rx_last_dst_ip[4] = {0}; // destination IP
volatile uint32_t eth_rx_icmp_count = 0;        // frames where proto==1
volatile uint32_t dbg_icmp_fl = 0;   // add at file scope

// ── ethernetif_input — direct DMA descriptor access ──────────────────────────
// HAL_ETH_GetRxDataBuffer / GetRxDataLength / BuildRxDescriptors are absent in
// this HAL revision, so we read the hardware descriptor directly.
//
// After a frame arrives the hardware clears RDES0_OWN (CPU now owns the
// descriptor).  We copy the data, restore the descriptor, set OWN again, and
// kick the DMA if it stopped waiting for a descriptor.
//
// Returns true if a frame was consumed (caller should loop until false).
volatile uint8_t eth_rx_ip_ver_byte;   // add to header
volatile uint32_t dbg_raw_fl = 0;

bool ethernetif_input(struct netif *netif) {
    ETH_DMADescTypeDef *desc   = &rx_desc[rx_read_idx];
    uint32_t            status = desc->DESC0;

    // DMA still owns this descriptor — no frame ready.
    if (status & RDES0_OWN) {
        return false;
    }

    // Frame length from RDES0 bits [29:16]; hardware count includes 4-byte FCS.
    uint32_t framelength = (status & RDES0_FL_MASK) >> RDES0_FL_SHIFT;
    dbg_raw_fl = framelength;   // ← what hardware reported
    if (framelength > 4U) {
        // framelength -= 4U;
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
        eth_rx_frame_count++;

        // Peek at EtherType (bytes 12-13 of the Ethernet header) from the pbuf
        // payload, which is a safe copy of the DMA buffer made above.
        // p->payload starts at byte 0 of the Ethernet frame.
        if (p->tot_len >= 14U) {
            const uint8_t *pl = (const uint8_t *)p->payload;
            uint16_t etype = ((uint16_t)pl[12] << 8) | pl[13];
            if      (etype == 0x0806U) eth_rx_etype_arp++;
            else if (etype == 0x0800U) {
                eth_rx_etype_ip++;
                eth_rx_ip_ver_byte = ((const uint8_t *)p->payload)[14];
                // In ethernetif_input, where you detect EtherType 0x0800:
                eth_rx_last_ip_proto  = pl[23];          // IP protocol (1=ICMP, 17=UDP, 6=TCP)
                eth_rx_last_src_ip[0] = pl[26];          // source IP bytes
                eth_rx_last_src_ip[1] = pl[27];
                eth_rx_last_src_ip[2] = pl[28];
                eth_rx_last_src_ip[3] = pl[29];
                eth_rx_last_dst_ip[0] = pl[30];
                eth_rx_last_dst_ip[1] = pl[31];
                eth_rx_last_dst_ip[2] = pl[32];
                eth_rx_last_dst_ip[3] = pl[33];
                if (pl[23] == 0x01U) { 
                    eth_rx_icmp_count++;
                    dbg_icmp_fl = framelength;   // ← framelength after the -= 4 subtraction
                    memcpy(dbg_icmp_dst_mac, pl, 6);  // b[0..5] = dst MAC
                } // ICMP frames
            }
                                             // byte 14 = first byte of IP header (should be 0x45)
            else                       eth_rx_etype_other++;
        }
        if (netif->input(p, netif) != ERR_OK) {
            eth_rx_input_err++;
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
            dbg_eth_input_nodispatch++;
            break;
    }
}

// ── ethernet_link_check_state — stub for CubeMX-generated lwip.c ─────────────

void ethernet_link_check_state(struct netif *netif) {
    (void)netif;
}

// ── eth_link_action — tea.c action, polls BSR once per second ────────────────
// NINT is unavailable (PA1 = ETH_REF_CLK), so we poll rather than interrupt.
// Call eth_link_action() once at startup to begin the polling loop.

void eth_link_action(void) {
    after(1000, eth_link_action);   // reschedule before any early return

    uint16_t bsr  = phy_read(LAN8720_BSR);
    bool     up   = (bsr & LAN8720_BSR_LINK_UP) != 0;

    if (up && link_state == LINK_DOWN) {
        link_state = LINK_UP;

        // Read speed/duplex from SCSR bits [4:2] (valid after autoneg).
        uint16_t speed   = phy_read(LAN8720_SCSR) & LAN8720_SCSR_SPEED_MASK;
        bool     full_dx = (speed == LAN8720_SCSR_10_FULL)  || (speed == LAN8720_SCSR_100_FULL);
        bool     spd100  = (speed == LAN8720_SCSR_100_HALF) || (speed == LAN8720_SCSR_100_FULL);

        // Update MAC speed and duplex to match autoneg result.
        ETH_MACConfigTypeDef MACConf = { 0 };
        HAL_ETH_GetMACConfig(&heth, &MACConf);
        MACConf.DuplexMode = full_dx ? ETH_FULLDUPLEX_MODE : ETH_HALFDUPLEX_MODE;
        MACConf.Speed      = spd100  ? ETH_SPEED_100M      : ETH_SPEED_10M;
        HAL_ETH_SetMACConfig(&heth, &MACConf);

        netif_set_link_up(&gnetif);

        uint32_t next = sys_timeouts_sleeptime();
        if (next != SYS_TIMEOUTS_SLEEPTIME_INFINITE) {
            after(next, lwip_timeout_action);
        }

    } else if (!up && link_state == LINK_UP) {
        link_state = LINK_DOWN;
        eth_tx_fail_streak = 0;     // link-drop clears streak — not a TX hang
        netif_set_link_down(&gnetif);
    }
}

// ── eth_recovery_action — PHY soft-reset and MAC DMA restart ─────────────────
//
// Triggered automatically when eth_tx_fail_streak reaches ETH_TX_FAIL_THRESH
// consecutive TX failures while link_state == LINK_UP, indicating the MAC or
// PHY has locked up without a detectable link-down event.
//
// Also callable directly from the CLI ("eth-recover") for manual testing.
//
// Recovery sequence:
//   1. Tell LwIP the link is down — prevents further TX attempts during reset.
//   2. Stop MAC DMA (HAL_ETH_Stop_IT).
//   3. Assert PE2 (nRST) LOW for PHY_NRST_ASSERT_MS, deassert, then wait
//      PHY_NRST_RECOVER_MS for the LAN8720 to complete its power-on init.
//      Hardware nRST is more reliable than BMCR soft reset for hard hangs.
//   4. Full HAL deinit + reinit — resets MAC DMA registers and descriptor rings.
//   5. Restart DMA (HAL_ETH_Start_IT).
//   6. eth_link_action() picks up from here and calls netif_set_link_up() once
//      BSR confirms the link is re-established (typically within 1–3 s).
//
// Note: HAL_ETH_MspInit is called again during HAL_ETH_Init but is idempotent —
// re-enabling clocks and re-configuring GPIO AF is safe.  PE2 is written HIGH
// inside MspInit so the PHY is never left in reset across subsequent HAL inits.

void eth_recovery_action(void) {
    eth_recovery_pending  = false;
    eth_recovery_count++;
    eth_recovery_last_ms  = HAL_GetTick();
    eth_tx_fail_streak    = 0;

    print("ETH: TX hang — starting recovery\r\n");

    // Step 1 — tell LwIP link is down so it stops generating TX.
    link_state = LINK_DOWN;
    netif_set_link_down(&gnetif);

    // Step 2 — stop MAC DMA (sets heth.gState = READY; MDIO still usable).
    HAL_ETH_Stop_IT(&heth);

    // Step 3 — hardware PHY reset via PE2 (nRST, active LOW).
    // HAL_Delay is acceptable here: Ethernet is already non-functional and
    // the total block is only PHY_NRST_ASSERT_MS + PHY_NRST_RECOVER_MS ms.
    HAL_GPIO_WritePin(PHY_NRST_PORT, PHY_NRST_PIN, GPIO_PIN_RESET);  // assert
    HAL_Delay(PHY_NRST_ASSERT_MS);
    HAL_GPIO_WritePin(PHY_NRST_PORT, PHY_NRST_PIN, GPIO_PIN_SET);    // deassert
    HAL_Delay(PHY_NRST_RECOVER_MS);   // wait for PHY power-on init to complete

    // Step 4 — HAL deinit + reinit.
    // HAL_ETH_DeInit sets gState = RESET so the subsequent HAL_ETH_Init will
    // call HAL_ETH_MspInit (idempotent) and fully reprogram DMA registers.
    HAL_ETH_DeInit(&heth);
    heth.Init.TxDesc = tx_desc;
    heth.Init.RxDesc = rx_desc;
    HAL_StatusTypeDef rc = HAL_ETH_Init(&heth);
    if (rc != HAL_OK) {
        print("ETH: HAL_ETH_Init failed during recovery\r\n");
        return;
    }

    // Redo our custom RX ring setup — HAL_ETH_Init may or may not have touched
    // the descriptors (HAL-version-dependent).  Always safe to overwrite.
    eth_rx_ring_init();

    // Step 5 — restart DMA.
    HAL_ETH_Start_IT(&heth);

    print("ETH: recovery done — waiting for link\r\n");
    // Step 6 — eth_link_action() will detect BSR link-up on its next 1-second
    // tick and call netif_set_link_up(), restoring IP connectivity.
}
