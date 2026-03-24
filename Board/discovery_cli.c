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
#include "lwip/pbuf.h"
#include "lwip/mem.h"
#include "netif/ethernet.h"

#include "tusb.h"
#include "usb_net.h"

#include "ethernetif.h"
#include "http_server.h"
#include "telnet_server.h"
#include "discovery_cli.h"

// Diagnostic counters — readable via show_ethernet().
extern volatile uint8_t eth_rx_ip_ver_byte;   // add to header
extern volatile uint16_t eth_rx_pbuf_tot_len;
extern volatile uint16_t eth_rx_ip_hdr_len_field;
extern volatile uint8_t  eth_rx_last_ip_proto;   // IP protocol byte
extern volatile uint8_t  eth_rx_last_src_ip[4]; // source IP
extern volatile uint8_t  eth_rx_last_dst_ip[4]; // destination IP
extern volatile uint32_t eth_rx_icmp_count;        // frames where proto==1
extern volatile uint8_t dbg_hwaddr[6];
extern volatile uint8_t dbg_icmp_dst_mac[6];
extern volatile uint32_t dbg_eth_input_entry   ;  // increments if ethernet_input is reached at all
extern volatile uint32_t dbg_eth_input_lendrop ;  // p->len <= SIZEOF_ETH_HDR
extern volatile uint32_t dbg_eth_input_hdrdrop ;  // pbuf_remove_header failed
extern volatile uint32_t dbg_eth_input_nodispatch ; // ethertype fell through switch
extern volatile uint32_t dbg_raw_fl;
extern volatile uint32_t dbg_icmp_fl;   // add at file scope

// ── LAN8720 register addresses (duplicated from ethernetif.c) ─────────────────

#define LAN8720_PHY_ADDRESS     0x00U   // match ethernetif.c — check PHYAD strap
#define LAN8720_PHYID1          0x02U   // PHY Identifier 1 (OUI bits 3-18) → 0x0007 for LAN8720
#define LAN8720_PHYID2          0x03U   // PHY Identifier 2 (OUI bits 19-24 + model) → 0xC0Fx for LAN8720
#define LAN8720_BSR             0x01U
#define LAN8720_SCSR            0x1FU
#define LAN8720_BSR_LINK_UP     (1U << 2)
#define LAN8720_SCSR_SPEED_MASK 0x001CU
#define LAN8720_SCSR_10_FULL    (5U << 2)
#define LAN8720_SCSR_100_HALF   (2U << 2)
#define LAN8720_SCSR_100_FULL   (6U << 2)

// ── Network ───────────────────────────────────────────────────────────────────
void printIp(volatile Byte *ip) { for(Byte i=0; i<3; i++ ) printDec0(ip[i]), print(".");  printDec(ip[3]); }
void printMac(volatile Byte *mac) { for(Byte i=0; i<5; i++ ) dotnb(2,2,mac[i],16), print(":");  dotnb(2,2,mac[5],16); }

void show_ethernet(void) {
    extern ETH_HandleTypeDef heth;

    // ── Hardware diagnostic — printed before any MDIO so you can see why ─────
    // ETHMACEN=bit25, ETHMACTXEN=bit26, ETHMACRXEN=bit27 of RCC_AHB1ENR.
    uint32_t ahb1 = RCC->AHB1ENR;
    print("RCC ETH clk: ");
    print((ahb1 & (1U << 25)) ? "MAC " : "MAC-OFF ");
    print((ahb1 & (1U << 26)) ? "TX "  : "TX-OFF ");
    print((ahb1 & (1U << 27)) ? "RX"   : "RX-OFF");
    printCr();

    // heth.gState: 0=RESET, 1=READY, 2=STARTED (ready & started), 0x20=BUSY, 0x30=ERROR
    print("heth.gState: 0x"); dotnb(2, 2, heth.gState, 16); printCr();

    // MACMIIAR: shows CR clock divider and whether MB (busy) bit is stuck
    print("MACMIIAR:    0x"); dotnb(8, 8, ETH->MACMIIAR, 16); printCr();

    // ── PHY address scan — try 0x00 and 0x01 ─────────────────────────────────
    // LAN8720: PHYID1=0x0007, PHYID2=0xC0Fx. HAL returns HAL_OK=0 / HAL_ERROR=1
    // / HAL_BUSY=2 / HAL_TIMEOUT=3. If all reads return HAL_ERROR or HAL_BUSY
    // the ETH peripheral is not ready (clock off, or HAL_ETH_Init not called).
    print("PHY scan (HAL_OK=0 ERR=1 BUSY=2 TO=3):\r\n");
    for (uint32_t addr = 0; addr <= 1; addr++) {
        uint32_t id1 = 0, id2 = 0, bsr = 0;
        HAL_StatusTypeDef r1 = HAL_ETH_ReadPHYRegister(&heth, addr, LAN8720_PHYID1, &id1);
        HAL_StatusTypeDef r2 = HAL_ETH_ReadPHYRegister(&heth, addr, LAN8720_PHYID2, &id2);
        HAL_StatusTypeDef r3 = HAL_ETH_ReadPHYRegister(&heth, addr, LAN8720_BSR,    &bsr);
        print("  addr 0x0"); printDec(addr);
        print(": rc="); printDec(r1); print("/"); printDec(r2); print("/"); printDec(r3);
        print("  ID1=0x"); dotnb(4, 4, id1, 16);
        print(" ID2=0x");  dotnb(4, 4, id2, 16);
        print(" BSR=0x");  dotnb(4, 4, bsr, 16);
        printCr();
    }

    // ── HAL init / start results (saved during boot) ─────────────────────────
    print("HAL_ETH_Init:  rc="); printDec((int)eth_init_rc);
    print(" state=0x"); dotnb(2, 2, eth_init_gstate,  16);
    print(" err=0x");   dotnb(8, 8, eth_init_error,   16); printCr();
    print("HAL_ETH_Start: rc="); printDec((int)eth_start_rc);
    print(" state=0x"); dotnb(2, 2, eth_start_gstate, 16); printCr();

    // ── ETH DMA registers — definitive TX/RX running state ───────────────────
    // DMAOMR bit 1 = SR (start/stop receive), bit 13 = ST (start/stop transmit)
    uint32_t dmaomr = ETH->DMAOMR;
    print("DMA RX: "); print((dmaomr & (1U <<  1)) ? "run" : "STOP"); print("  ");
    print("DMA TX: "); print((dmaomr & (1U << 13)) ? "run" : "STOP"); printCr();
    print("DMARDLAR: 0x"); dotnb(8, 8, ETH->DMARDLAR, 16); printCr();
    print("DMATDLAR: 0x"); dotnb(8, 8, ETH->DMATDLAR, 16); printCr();
    print("DMASR:    0x"); dotnb(8, 8, ETH->DMASR,    16); printCr();

    // ── MAC Control Register — TX enable, speed, duplex ──────────────────────
    // MACCR bit3=TE(TX en), bit2=RE(RX en), bit14=FES(1=100M), bit11=DM(1=full)
    {
        uint32_t maccr = ETH->MACCR;
        print("MACCR TE:"); print((maccr & (1U <<  3)) ? "1" : "0");
        print(" RE:");      print((maccr & (1U <<  2)) ? "1" : "0");
        print(" Speed:");   print((maccr & (1U << 14)) ? "100M" : "10M");
        print(" Duplex:");  print((maccr & (1U << 11)) ? "Full" : "Half");
        printCr();
    }

    // ── GPIO AF readback — verify TX pins are actually AF11 at runtime ────────
    // GPIOB AFR[1] = AFRH, covers pins 8-15.  Each pin gets 4 bits.
    // PB11: bits[15:12]  PB12: bits[19:16]  PB13: bits[23:20]
    // AF11 = 0xB.  If any value != 11 the pin was reconfigured after MspInit.
    {
        uint32_t pb_afrh  = GPIOB->AFR[1];
        uint32_t pb_moder = GPIOB->MODER;
        // MODER bits: 2*n+1 : 2*n.  AF mode = 0b10 = 2.
        // PB11: bits[23:22], PB12: bits[25:24], PB13: bits[27:26]
        print("PB11 AF:"); printDec((pb_afrh >> 12) & 0xF);
        print(" MODER:"); printDec((pb_moder >> 22) & 3); printCr(); // expect AF=11 MODER=2
        print("PB12 AF:"); printDec((pb_afrh >> 16) & 0xF);
        print(" MODER:"); printDec((pb_moder >> 24) & 3); printCr(); // expect AF=11 MODER=2
        print("PB13 AF:"); printDec((pb_afrh >> 20) & 0xF);
        print(" MODER:"); printDec((pb_moder >> 26) & 3); printCr(); // expect AF=11 MODER=2
    }
    // ── Also check PA1/PA2/PA7 (RX side) and PC1/PC4/PC5 ─────────────────────
    // PA AFRL (AFR[0]) covers pins 0-7.  PA1: bits[7:4], PA2: bits[11:8], PA7: bits[31:28]
    {
        uint32_t pa_afrl  = GPIOA->AFR[0];
        uint32_t pa_moder = GPIOA->MODER;
        print("PA1 AF:"); printDec((pa_afrl >>  4) & 0xF);
        print(" PA2 AF:"); printDec((pa_afrl >>  8) & 0xF);
        print(" PA7 AF:"); printDec((pa_afrl >> 28) & 0xF); printCr(); // all expect 11
        (void)pa_moder;
    }
    // PC AFRL (AFR[0]) covers pins 0-7.  PC1: bits[7:4], PC4: bits[19:16]->AFRH? No:
    // PC1: AFR[0] bits[7:4], PC4: AFR[0] bits[19:16], PC5: AFR[0] bits[23:20]
    {
        uint32_t pc_afrl = GPIOC->AFR[0];
        print("PC1 AF:"); printDec((pc_afrl >>  4) & 0xF);
        print(" PC4 AF:"); printDec((pc_afrl >> 16) & 0xF);
        print(" PC5 AF:"); printDec((pc_afrl >> 20) & 0xF); printCr(); // all expect 11
    }

    print("RX IRQs:  "); printDec(eth_rx_irq_count);      printCr();
    print("RX frames:"); printDec(eth_rx_frame_count);    printCr();
    print("RX inp err:"); printDec(eth_rx_input_err);     printCr();
    print("RX ARP:   "); printDec(eth_rx_etype_arp);      printCr();
    print("RX IP:    "); printDec(eth_rx_etype_ip);       printCr();
    print("RX other: "); printDec(eth_rx_etype_other);    printCr();
    // Confirm whether netif->input is ethernet_input (correct for NO_SYS=1)
    // or tcpip_input (broken without RTOS).  On NO_SYS=1, tcpip_input is a
    // macro alias for ethernet_input so both addresses will be identical.
    print("netif->input:  0x"); dotnb(8, 8, (uint32_t)gnetif.input,      16); printCr();
    print("ethernet_input:0x"); dotnb(8, 8, (uint32_t)ethernet_input,    16); printCr();
    // Probe whether PBUF_RAM allocation works.  ARP replies, ICMP replies, and
    // all LwIP-generated TX frames use pbuf_alloc(..., PBUF_RAM), which draws
    // from LwIP's internal static heap (MEM_SIZE in lwipopts.h).  If MEM_SIZE
    // is 0 or too small every outgoing frame silently fails at etharp_raw().
    {
        struct pbuf *probe = pbuf_alloc(PBUF_RAW, 64, PBUF_RAM);
        print("PBUF_RAM probe:"); print(probe ? "OK" : "FAIL(MEM_SIZE=0?)"); printCr();
        if (probe) pbuf_free(probe);
    }
    print("TX calls: "); printDec(eth_tx_call_count);     printCr();
    print("TX ok:    "); printDec(eth_tx_ok_count);       printCr();
    print("TX err:   "); printDec(eth_tx_err_count);      printCr();
    print("TX t/o:   "); printDec(eth_tx_timeout_count);  printCr();
    print("PHY resets:"); printDec(eth_recovery_count);   printCr();
    if (eth_recovery_count > 0) {
        uint32_t age_s = (HAL_GetTick() - eth_recovery_last_ms) / 1000U;
        print("  last:   "); printDec(age_s); print("s ago"); printCr();
    }
    print("TX ARP:   "); printDec(eth_tx_arp_count);       printCr();
    print("TX IP:    "); printDec(eth_tx_ip_count);        printCr();
    print("TX IP dst:");
    for (int i = 0; i < 6; i++) {
        dotnb(2, 2, eth_tx_last_ip_dst[i], 16);
        if (i < 5) print(":");
    }
    printCr();

    // ── Link state using configured address ──────────────────────────────────
    uint32_t bsr_val = 0, scsr_val = 0;
    HAL_ETH_ReadPHYRegister(&heth, LAN8720_PHY_ADDRESS, LAN8720_BSR,  &bsr_val);
    HAL_ETH_ReadPHYRegister(&heth, LAN8720_PHY_ADDRESS, LAN8720_SCSR, &scsr_val);

    bool     link   = ((uint16_t)bsr_val  & LAN8720_BSR_LINK_UP) != 0;
    uint16_t speed  = (uint16_t)scsr_val  & LAN8720_SCSR_SPEED_MASK;
    bool     full   = (speed == LAN8720_SCSR_10_FULL)  || (speed == LAN8720_SCSR_100_FULL);
    bool     spd100 = (speed == LAN8720_SCSR_100_HALF) || (speed == LAN8720_SCSR_100_FULL);

    print("ETH link:   "); print(link   ? "up"       : "down");    printCr();
    print("ETH speed:  "); print(spd100 ? "100 Mbps" : "10 Mbps"); printCr();
    print("ETH duplex: "); print(full   ? "full"     : "half");     printCr();
    print("ETH MAC:    ");
    for (int i = 0; i < 6; i++) {
        dotnb(2, 2, gnetif.hwaddr[i], 16);
        if (i < 5) print(":");
    }
    printCr();
    // ip.err, ip.lenerr, netif.flags, netif.ip.addr, and eth_rx_ip_ver_byte
    print("IP  err:    "); printDec(lwip_stats.ip.err);    printCr();
    print("ip.lenerr=");printDec(lwip_stats.ip.lenerr); printCr();
    print("netif.flags=0x"); printHex(gnetif.flags); printCr();
    print("netif.ip.addr=0x"); printHex(gnetif.ip_addr.addr); printCr();
    print("eth_rx_ip_ver_byte=0x"), printHex2(eth_rx_ip_ver_byte);
    print("\neth_rx_pbuf_tot_len="), printDec(eth_rx_pbuf_tot_len);
    print("\neth_rx_ip_hdr_len_field="), printDec(eth_rx_ip_hdr_len_field);

    print("\neth_rx_icmp_count="), printDec(eth_rx_icmp_count);
    print("\neth_rx_last_ip_proto="), printDec(eth_rx_last_ip_proto);
    print("\neth_rx_last_src_ip="), printIp(eth_rx_last_src_ip);
    print("\neth_rx_last_dst_ip="), printIp(eth_rx_last_dst_ip);
    print("\nPBUF_POOL_BUFSIZE="), printDec(PBUF_POOL_BUFSIZE);
    print("\nETH_RX_BUF_SIZE="), printDec(ETH_RX_BUF_SIZE);
    print("\ndbg_hwaddr: "), printMac(dbg_hwaddr);
    print("\ndbg_icmp_dst_mac: "), printMac(dbg_icmp_dst_mac);
    print("\ndbg_eth_input_entry="), printDec(dbg_eth_input_entry);
    print("\ndbg_eth_input_lendrop="), printDec(dbg_eth_input_lendrop);
    print("\ndbg_eth_input_hdrdrop="), printDec(dbg_eth_input_hdrdrop);
    print("\ndbg_eth_input_nodispatch="), printDec(dbg_eth_input_nodispatch);
    print("\ndbg_raw_fl="), printDec(dbg_raw_fl);
    print("\ndbg_icmp_fl="), printDec(dbg_icmp_fl);
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

    print("netif.flags=0x"); printHex(gnetif.flags); printCr();
    print("netif.ip.addr=0x"); printHex(gnetif.ip_addr.addr); printCr();
    // Expected: flags has bit0 (NETIF_FLAG_UP=0x01) and bit2 (NETIF_FLAG_LINK_UP=0x04) set
    // Expected: ip.addr = 0x1864A8C0  (192.168.100.24 in LE)
}

void uitoa(uint32_t n, char *buf) {
    if (n == 0) {
        buf[0] = '0';
        buf[1] = '\0';
        return;
    }
    char tmp[11];
    int i = 0;
    while (n > 0) {
        tmp[i++] = '0' + (n % 10);
        n /= 10;
    }
    // reverse
    for (int j = 0; j < i; j++)
        buf[j] = tmp[i - 1 - j];
    buf[i] = '\0';
}

void show_net(void) {
    // Compile-time flags — always printed regardless of LWIP_STATS
#if LWIP_ICMP
    print("LWIP_ICMP:  YES"); printCr();
#else
    print("LWIP_ICMP:  NO  <-- ICMP disabled, ping impossible"); printCr();
#endif
#if CHECKSUM_GEN_IP
    print("CKSUM_GEN_IP:   YES"); printCr();
#else
    print("CKSUM_GEN_IP:   NO  <-- TX IP frames have zero checksum, PC drops them"); printCr();
#endif
#if CHECKSUM_GEN_ICMP
    print("CKSUM_GEN_ICMP: YES"); printCr();
#else
    print("CKSUM_GEN_ICMP: NO  <-- TX ICMP frames have zero checksum, PC drops them"); printCr();
#endif
#if CHECKSUM_CHECK_IP
    print("CKSUM_CHK_IP:   yes"); printCr();
#else
    print("CKSUM_CHK_IP:   skip"); printCr();
#endif
#if CHECKSUM_CHECK_ICMP
    print("CKSUM_CHK_ICMP: yes"); printCr();
#else
    print("CKSUM_CHK_ICMP: skip"); printCr();
#endif
#if LWIP_STATS
    print("IP  RX:     "); printDec(lwip_stats.ip.recv);    printCr();
    print("IP  TX:     "); printDec(lwip_stats.ip.xmit);   printCr();
    print("IP  drop:   "); printDec(lwip_stats.ip.drop);   printCr();
    print("IP  err:    "); printDec(lwip_stats.ip.err);    printCr();
    print("IP  chkerr: "); printDec(lwip_stats.ip.chkerr); printCr();
    print("IP  proterr:"); printDec(lwip_stats.ip.proterr); printCr();
# if LWIP_ICMP
    print("ICMP RX:    "); printDec(lwip_stats.icmp.recv);  printCr();
    print("ICMP TX:    "); printDec(lwip_stats.icmp.xmit);  printCr();
    print("ICMP drop:  "); printDec(lwip_stats.icmp.drop);  printCr();
    print("ICMP err:   "); printDec(lwip_stats.icmp.err);   printCr();
    print("ICMP chkerr:"); printDec(lwip_stats.icmp.chkerr); printCr();
# else
    print("LWIP_ICMP disabled"); printCr();
# endif
    print("ARP  TX:    "); printDec(lwip_stats.etharp.xmit); printCr();
    print("ARP  RX:    "); printDec(lwip_stats.etharp.recv); printCr();
    print("ARP  drop:  "); printDec(lwip_stats.etharp.drop); printCr();
    print("TCP err:    "); printDec(lwip_stats.tcp.err);    printCr();
    print("pbuf used:  "); printDec(lwip_stats.mem.used);   printCr();
#else
    print("LWIP_STATS not enabled in lwipopts.h"); printCr();
#endif
#if LWIP_STATS
#include "lwip/stats.h"
    char buf[24];
    print("IP  recv:"); uitoa(lwip_stats.ip.recv,   buf); print(buf); printCr();
    print("IP  drop:"); uitoa(lwip_stats.ip.drop,   buf); print(buf); printCr();
    print("IP  chkerr:"); uitoa(lwip_stats.ip.chkerr, buf); print(buf); printCr();
    print("ICMP recv:"); uitoa(lwip_stats.icmp.recv, buf); print(buf); printCr();
    print("ICMP xmit:"); uitoa(lwip_stats.icmp.xmit, buf); print(buf); printCr();
    print("ICMP drop:"); uitoa(lwip_stats.icmp.drop, buf); print(buf); printCr();
    print("MEM used:"); uitoa(lwip_stats.mem.used,  buf); print(buf); printCr();
    print("MEM err:"); uitoa(lwip_stats.mem.err,   buf); print(buf); printCr();
#endif
}

#if LWIP_STATS
#include "lwip/stats.h"
extern volatile uint32_t dbg_http_recv;

void show_stats(void) {
    char b[12];
    print("ip.recv=");  uitoa(lwip_stats.ip.recv,   b); print(b); printCr();
    print("ip.drop=");  uitoa(lwip_stats.ip.drop,   b); print(b); printCr();
    print("ip.chkerr=");uitoa(lwip_stats.ip.chkerr, b); print(b); printCr();
    print("ip.opterr=");uitoa(lwip_stats.ip.opterr, b); print(b); printCr();
    print("ip.err=");   uitoa(lwip_stats.ip.err,    b); print(b); printCr();
    print("ip.lenerr=");uitoa(lwip_stats.ip.lenerr, b); print(b); printCr();
    print("icmp.recv=");uitoa(lwip_stats.icmp.recv, b); print(b); printCr();
    print("icmp.xmit=");uitoa(lwip_stats.icmp.xmit, b); print(b); printCr();
    print("icmp.drop=");uitoa(lwip_stats.icmp.drop, b); print(b); printCr();
    print("icmp.err="); uitoa(lwip_stats.icmp.err,  b); print(b); printCr();
    print("mem.used="); uitoa(lwip_stats.mem.used,  b); print(b); printCr();
    print("mem.err=");  uitoa(lwip_stats.mem.err,   b); print(b); printCr();
    print("\nlwip_sntats.memp[MEMP_PBUF_POOL]->used="), printDec(lwip_stats.memp[MEMP_PBUF_POOL]->used);
    print("\nlwip_stats.memp[MEMP_PBUF_POOL]->max="), printDec(lwip_stats.memp[MEMP_PBUF_POOL]->max);
    print("\nlwip_stats.mem.used="), printDec(lwip_stats.mem.used);
    print("\ndbg_http_recv="), printDec(dbg_http_recv);
}
#endif


void show_http(void) {
    // Connection state counts reported from http_server.c
    // http_server_stats() fills the provided counters.
    uint8_t active = 0, idle = 0;
    http_server_stats(&active, &idle);
    print("HTTP active: "); printDec(active); printCr();
    print("HTTP idle:   "); printDec(idle);   printCr();
    print("HTTP port:   80"); printCr();
}
                 // heap bytes in use
void show_telnet(void) {
    uint8_t active = 0, idle = 0;
    telnet_server_stats(&active, &idle);
    print("Telnet active: "); printDec(active); printCr();
    print("Telnet idle:   "); printDec(idle);   printCr();
    print("Telnet port:   23"); printCr();
}

// ── USB ───────────────────────────────────────────────────────────────────────

void show_usb(void) {
    // ── USB device layer ──────────────────────────────────────────────────────
    bool connected  = tud_connected();
    bool mounted    = tud_mounted();
    bool suspended  = tud_suspended();

    print("USB device:    ");
    if (!connected)       print("disconnected");
    else if (suspended)   print("suspended");
    else if (mounted)     print("mounted (configured)");
    else                  print("connected (enumerating)");
    printCr();

    // ── CDC ACM — serial terminal ─────────────────────────────────────────────
    print("CDC serial:    ");
    if (!mounted)                   print("n/a");
    else if (tud_cdc_connected())   print("terminal open");
    else                            print("no terminal");
    printCr();

    // ── CDC NCM — USB network interface ──────────────────────────────────────
    print("NCM netif:     ");
    if (!mounted) {
        print("n/a");
    } else if (netif_is_up(&usb_netif)) {
        print("up  ");
        print(ip4addr_ntoa(netif_ip4_addr(&usb_netif)));
        if (!netif_is_link_up(&usb_netif)) print(" (link down)");
    } else {
        print("down");
    }
    printCr();
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
