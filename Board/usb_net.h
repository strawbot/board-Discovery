// usb_net.h — USB CDC-NCM LwIP netif
//
// Provides a 192.168.7.1/24 Ethernet interface over USB.
// Call usb_netif_init() from network_init() after LwIP is ready.

#ifndef USB_NET_H
#define USB_NET_H

#include "lwip/netif.h"

// The USB network interface — available after usb_netif_init() returns.
extern struct netif usb_netif;

// usb_netif_init — register the USB netif with LwIP.
// Must be called after LwIP has been initialised (MX_LWIP_Init done).
void usb_netif_init(void);

#endif // USB_NET_H
