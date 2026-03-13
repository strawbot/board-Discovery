// discovery_cli.c — CLI command implementations for the Discovery board.
// All functions are void(void) — output via print() / printers.h to emitq.

#include <stdbool.h>
#include <stdint.h>

#include "tea.h"
#include "printers.h"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_eth.h"
#include "stm32f4xx_ll_rcc.h"

#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include "lwip/dhcp.h"
#include "lwip/stats.h"

#include "tusb.h"

#include "ethernetif.h"
#include "http_server.h"
#include "telnet_server.h"
#include "discovery_cli.h"

// ── DP83848 register addresses (duplicated from ethernetif.c) ─────────────────

#define DP83848_PHY_ADDRESS     0x01U
#define DP83848_PHYSTS          0x10U
#define DP83848_PHYSTS_LINK_UP  (1U << 0)
#define DP83848_PHYSTS_SPEED_10 (1U << 1)
#define DP83848_PHYSTS_DUPLEX   (1U << 2)

// ── Network ───────────────────────────────────────────────────────────────────

void show_ethernet(void) {
    extern ETH_HandleTypeDef heth;
    uint32_t physts_val = 0;
    HAL_ETH_ReadPHYRegister(&heth, DP83848_PHY_ADDRESS, DP83848_PHYSTS, &physts_val);
    uint16_t physts = (uint16_t)physts_val;

    bool link  = (physts & DP83848_PHYSTS_LINK_UP)  != 0;
    bool full  = (physts & DP83848_PHYSTS_DUPLEX)   != 0;
    bool spd10 = (physts & DP83848_PHYSTS_SPEED_10) != 0;

    print("ETH link:   "); print(link  ? "up"       : "down");   printCr();
    print("ETH speed:  "); print(spd10 ? "10 Mbps"  : "100 Mbps"); printCr();
    print("ETH duplex: "); print(full  ? "full"     : "half");    printCr();
    print("ETH MAC:    ");
    for (int i = 0; i < 6; i++) {
        dotnb(2, 2, gnetif.hwaddr[i], 16);
        if (i < 5) print(":");
    }
    printCr();
}

void show_ip(void) {
    print("IP addr:  "); print(ip4addr_ntoa(netif_ip4_addr(&gnetif)));    printCr();
    print("gateway:  "); print(ip4addr_ntoa(netif_ip4_gw(&gnetif)));      printCr();
    print("netmask:  "); print(ip4addr_ntoa(netif_ip4_netmask(&gnetif))); printCr();

    print("DHCP:     ");
    if (!netif_dhcp_data(&gnetif)) {
        print("static");
    } else if (dhcp_supplied_address(&gnetif)) {
        print("bound");
    } else {
        print("searching");
    }
    printCr();

    print("netif:    ");
    print(netif_is_up(&gnetif)      ? "up "   : "down ");
    print(netif_is_link_up(&gnetif) ? "link-up" : "link-down");
    printCr();
}

void show_net(void) {
#if LWIP_STATS
    print("RX packets: "); printDec(lwip_stats.ip.recv);  printCr();
    print("TX packets: "); printDec(lwip_stats.ip.xmit);  printCr();
    print("RX drop:    "); printDec(lwip_stats.ip.drop);  printCr();
    print("RX errors:  "); printDec(lwip_stats.ip.err);   printCr();
    print("TCP err:    "); printDec(lwip_stats.tcp.err);  printCr();
    print("pbuf alloc: "); printDec(lwip_stats.mem.used); printCr();
#else
    print("LWIP_STATS not enabled in lwipopts.h"); printCr();
#endif
}

void show_http(void) {
    // Connection state counts reported from http_server.c
    // http_server_stats() fills the provided counters.
    uint8_t active = 0, idle = 0;
    http_server_stats(&active, &idle);
    print("HTTP active: "); printDec(active); printCr();
    print("HTTP idle:   "); printDec(idle);   printCr();
    print("HTTP port:   80"); printCr();
}

void show_telnet(void) {
    uint8_t active = 0, idle = 0;
    telnet_server_stats(&active, &idle);
    print("Telnet active: "); printDec(active); printCr();
    print("Telnet idle:   "); printDec(idle);   printCr();
    print("Telnet port:   23"); printCr();
}

// ── USB ───────────────────────────────────────────────────────────────────────

void show_usb(void) {
    print("USB mounted:   "); print(tud_mounted()      ? "yes" : "no");  printCr();
    print("CDC connected: "); print(tud_cdc_connected() ? "yes" : "no"); printCr();
}

// ── System ────────────────────────────────────────────────────────────────────

void show_sys(void) {
    LL_RCC_ClocksTypeDef clocks;
    LL_RCC_GetSystemClocksFreq(&clocks);
    print("SYSCLK:  "); printDec(clocks.SYSCLK_Frequency / 1000000); print(" MHz"); printCr();
    print("HCLK:    "); printDec(clocks.HCLK_Frequency   / 1000000); print(" MHz"); printCr();
    print("PCLK1:   "); printDec(clocks.PCLK1_Frequency  / 1000000); print(" MHz"); printCr();
    print("PCLK2:   "); printDec(clocks.PCLK2_Frequency  / 1000000); print(" MHz"); printCr();
    print("uptime:  "); printDec(get_ticks());                        print(" ticks"); printCr();
    show_timer();
}

void do_reboot(void) {
    print("rebooting..."); printCr();
    NVIC_SystemReset();
}
