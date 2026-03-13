#ifndef ETHERNETIF_H
#define ETHERNETIF_H

#include <stdbool.h>

#include "lwip/netif.h"
#include "lwip/err.h"

// ── Public API ───────────────────────────────────────────────────────────────

// Call once after clocks and GPIO are ready.
// Initialises the DP83848 PHY, DMA descriptors, and registers the LwIP netif.
err_t ethernetif_init(struct netif *netif);

// Called by CubeMX lwip.c and by eth_input_action.
// Consumes one frame from the DMA ring and passes it to LwIP.
// Returns true if a frame was processed (call again until false).
bool  ethernetif_input(struct netif *netif);

// ── tea.c Actions ────────────────────────────────────────────────────────────

// Posted by ETH DMA RX interrupt — single-instance, state-machine guarded.
void eth_input_action(void);

// Posted by PHY INT pin interrupt — handles link up / link down.
void eth_link_action(void);

// Stub for CubeMX-generated lwip.c — link state is handled by PHY interrupt.
void ethernet_link_check_state(struct netif *netif);

// Global netif instance — accessible to network_init() and lwip_timeout_action().
extern struct netif gnetif;

#endif // ETHERNETIF_H
