#include <stdlib.h>
#include <string.h>
#include "printers.h"
#include "cli.h"
#include <ctype.h>

bool visible_word(char *s);

static char *filter;

const char *strcasestr_r(const char *haystack, const char *needle) {
    size_t nlen = strlen(needle);
    if (nlen == 0) return haystack;
    for (; *haystack; haystack++) {
        if (strncasecmp(haystack, needle, nlen) == 0) return haystack;
    }
    return NULL;
}

static void printif(char *s) {
	if (strcasestr_r(s, filter) != NULL && visible_word(s))
		print(s);
}

static void helphelp() {
	print("Print commands with one line help with wild card filtering: help <filter>");
    print("\nCommands with Arguments");
    print("\n Some arguments precede the command while others have arguments after.");
    print("\n Generally, numbers come before while strings come after. A number as");
    print("\n a string will come after such as the commands setting port addresses");
    print("\n or ip addresses.");
    print("\n\n  Use:");
    print("\n   <s> for strings following a command");
    print("\n   (n) for parameters preceding a command");
    print("\n       if there are results they are preceded by -");
    print("\n       so ( a - n ) uses a and returns n");
    print("\n  For example:");
    print("\n    	rm   <pattern> remove files whose name matches pattern");
    print("\n      seek   ( n )  seek to position n in the file; -1 to end");
    print("\n        s!   ( h a - ) store next into memory using top as address (16 bit)");
    print("\n    negate   ( n - -n ) two's complement of top data stack item");
    print("\n\n  Command examples:");
    print("\n   rm somefilename");
    print("\n   124 seek");
    print("\n   1001 portaddress s!");
    print("\n   100 negate");
}

void help(void) {
	cursorReturn();
	parse(0);
	here();
	filter = (char *)ret()+1;
	if (strcmp("help", filter) == 0) {
		helphelp();
		return;
	}
    printif("gnetif   ( - a ) LwIP netif structure for Ethernet\n");
    printif("reboot   reboot the device via NVIC system reset\n");
    printif("show-eth   show Ethernet link status, speed, duplex and PHY info\n");
    printif("show-http   show HTTP server state and active connections\n");
    printif("show-ip   show IP address, gateway, netmask and DHCP state\n");
    printif("show-net   show LwIP network statistics: RX/TX counts, errors\n");
    printif("show-sys   show system info: clock frequencies and uptime\n");
    printif("show-telnet   show Telnet server state and active connections\n");
    printif("show-timer   show delta timer state and UTC tick counter\n");
    printif("show-usb   show USB connection state and CDC line status\n");
}
