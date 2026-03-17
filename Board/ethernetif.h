#ifndef ETHERNETIF_H
#define ETHERNETIF_H

#include <stdbool.h>

#include "lwip/netif.h"
#include "lwip/err.h"

// ── Public API ───────────────────────────────────────────────────────────────

// Call once after clocks and GPIO are ready.
// Initialises the LAN8720 PHY, DMA descriptors, and registers the LwIP netif.
err_t ethernetif_init(struct netif *netif);

// Called by CubeMX lwip.c and by eth_input_action.
// Consumes one frame from the DMA ring and passes it to LwIP.
// Returns true if a frame was processed (call again until false).
bool  ethernetif_input(struct netif *netif);

// ── tea.c Actions ────────────────────────────────────────────────────────────

// Posted by ETH DMA RX interrupt — single-instance, state-machine guarded.
void eth_input_action(void);

// Polls BSR once per second for link up / link down (NINT unavailable).
// Call once at startup to begin the self-rescheduling polling loop.
void eth_link_action(void);

// Stub for CubeMX-generated lwip.c — link state is handled by PHY interrupt.
void ethernet_link_check_state(struct netif *netif);

// Global netif instance — accessible to network_init() and lwip_timeout_action().
extern struct netif gnetif;

// Saved results of HAL_ETH_Init / HAL_ETH_Start_IT — readable after boot.
// 0xFF = not yet called; 0 = HAL_OK; 1 = HAL_ERROR; 2 = HAL_BUSY; 3 = HAL_TIMEOUT.
#include "stm32f4xx_hal_eth.h"
extern volatile uint32_t eth_rx_irq_count;
extern volatile uint32_t eth_rx_frame_count;
extern volatile uint32_t eth_rx_input_err;
extern volatile uint32_t eth_rx_etype_arp;
extern volatile uint32_t eth_rx_etype_ip;
extern volatile uint32_t eth_rx_etype_other;
extern volatile uint32_t eth_tx_call_count;
extern volatile uint32_t eth_tx_ok_count;
extern volatile uint32_t eth_tx_err_count;
extern volatile uint32_t eth_tx_timeout_count;
extern volatile uint32_t eth_tx_arp_count;     // outgoing frames with EtherType 0x0806
extern volatile uint32_t eth_tx_ip_count;      // outgoing frames with EtherType 0x0800
extern uint8_t           eth_tx_last_ip_dst[6]; // dest MAC of most recent outgoing IP frame
extern HAL_StatusTypeDef eth_init_rc;
extern HAL_StatusTypeDef eth_start_rc;
extern uint8_t           eth_init_gstate;
extern uint32_t          eth_init_error;
extern uint8_t           eth_start_gstate;

#endif // ETHERNETIF_H
