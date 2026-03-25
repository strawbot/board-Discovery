// network_init.h — Board-specific network bring-up for Discovery

#ifndef NETWORK_INIT_H
#define NETWORK_INIT_H

// eth_status — returns a human-readable description of the current Ethernet
// interface state, e.g. "192.168.100.24 (DHCP)" or "link down".
// The returned pointer is valid until the next call (uses an internal static buffer).
const char *eth_status(void);

// network_update_default_route — pick the best available internet-facing netif.
// Prefers Ethernet when its link is up; falls back to USB when a host is
// connected and Ethernet is down.  Call on every interface state change.
void network_update_default_route(void);

#endif // NETWORK_INIT_H
