#ifndef DISCOVERY_CLI_H
#define DISCOVERY_CLI_H

// Discovery board CLI command implementations.
// All functions are void(void) as required by the WordLists parser.
// Bound to CLI text commands via discoverywords.txt → wordlist.c

// network
void show_ethernet(void);   // Ethernet link status, speed, duplex, PHY
void show_ip(void);         // IP address, gateway, netmask, DHCP state
void show_net(void);        // LwIP RX/TX counts and error statistics
void show_http(void);       // HTTP server state and active connections
void show_telnet(void);     // Telnet server state and active connections

// usb
void show_usb(void);        // USB connection state and CDC line status

// accelerometer
void show_acc(void);        // Chip type, WHO_AM_I, pitch, roll, sample and tap counts

// system
void show_sys(void);        // Clock frequencies and uptime
void show_timers(void);     // STM32F407 hardware timer survey: TIM1-TIM14 + RTC (clock gate, CEN, PSC/ARR/CNT, active CC channels)
void do_reboot(void);       // NVIC system reset

#endif // DISCOVERY_CLI_H
