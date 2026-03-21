// network_init.h — Board-specific network bring-up for Discovery

#ifndef NETWORK_INIT_H
#define NETWORK_INIT_H

// eth_status — returns a human-readable description of the current Ethernet
// interface state, e.g. "192.168.100.24 (DHCP)" or "link down".
// The returned pointer is valid until the next call (uses an internal static buffer).
const char *eth_status(void);

#endif // NETWORK_INIT_H
