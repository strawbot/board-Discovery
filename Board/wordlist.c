 // names are kept in flash; arrays are used; reduces space requirements
#include "cli.h"
#include <stdio.h>

#define NAMES(name) const char name[] = {
#define NAME(s) s "\000"
#define END_NAMES ""}; // empty string to cover empty array

#define NONAMES(name) const char name[] = {""};
#define NOBODIES(functions) const vector functions[] = {NULL};
#define NOCONBODS(constants) const struct constantCall constants[] = {{NULL}};
#define BODIES(functions) const vector functions[] = {
#define CBODIES const struct constantCall constantbodies[] = {
#define BODY(f) (vector)f,
#define CONSTANTBODY(f)  { cii, &f },
#define CONSTANTNUMBER(n)  { cii, (Byte *)n },
#define END_BODIES };

void cii(void);


// Words
NAMES(wordnames)
	NAME("show-eth")		//  show Ethernet link status, speed, duplex and PHY info
	NAME("show-ip")		//  show IP address, gateway, netmask and DHCP state
	NAME("show-net")		//  show LwIP network statistics: RX/TX counts, errors
	NAME("show-http")		//  show HTTP server state and active connections
	NAME("show-telnet")		//  show Telnet server state and active connections
	NAME("show-usb")		//  show USB connection state and CDC line status
	NAME("show-sys")		//  show system info: clock frequencies and uptime
	NAME("show-timer")		//  show delta timer state and UTC tick counter
	NAME("reboot")		//  reboot the device via NVIC system reset
END_NAMES

void show_ethernet(void);
void show_ip(void);
void show_net(void);
void show_http(void);
void show_telnet(void);
void show_usb(void);
void show_sys(void);
void show_timer(void);
void do_reboot(void);

BODIES(wordbodies)
	BODY(show_ethernet)
	BODY(show_ip)
	BODY(show_net)
	BODY(show_http)
	BODY(show_telnet)
	BODY(show_usb)
	BODY(show_sys)
	BODY(show_timer)
	BODY(do_reboot)
END_BODIES

// Immediates
NAMES(immediatenames)
END_NAMES

NOBODIES(immediatebodies)
// Constants
NAMES(constantnames)
	NAME("gnetif")		//  ( - a ) LwIP netif structure for Ethernet
END_NAMES

extern Byte gnetif;

CBODIES
	CONSTANTBODY(gnetif)
END_BODIES

