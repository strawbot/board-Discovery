// usb_dhcpd.h — minimal DHCP server for the USB NCM point-to-point link
//
// Always leases 192.168.7.2/24 (gateway 192.168.7.1) to the USB host so
// the host network adapter self-configures on plug-in without any manual
// IP assignment.  Call usb_dhcpd_init() once after the USB netif is up.

#ifndef USB_DHCPD_H
#define USB_DHCPD_H

void usb_dhcpd_init(void);

#endif // USB_DHCPD_H
