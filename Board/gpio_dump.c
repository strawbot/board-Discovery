/* =============================================================================
 * gpio_dump.c  —  Dump all GPIO pin states (STM32F4 / F7, bare registers)
 *
 * Prints for every pin:
 *   PIN | NAME (user alias) | MODE | AF number | Output type | Pull | IDR | ODR | FUNCTION
 *
 * Usage:
 *   1. Populate alias_table[] with your project's pin assignments.
 *   2. Trim port_table[] to the ports present on your package.
 *   3. Call gpio_dump_all()  — shows every pin.
 *      Call gpio_dump_active() — shows only pins not in reset (ANALOG) state.
 *
 * Depends on: printers.h  →  print(), printDec(), printHex2(), tabTo()
 *             stm32f4xx.h  (or stm32f7xx.h)
 * =============================================================================*/

#include <stdint.h>
#include <stdio.h>              /* snprintf                                   */
#include <ctype.h>
#include "stm32f4xx.h"          /* swap for stm32f7xx.h on F7 devices         */
#include "printers.h"           /* print() printDec() printHex2() tabTo()     */
#include "cli.h"

// utilty for caseless strstr
// Source - https://stackoverflow.com/a/27304609
// Posted by Clifford, modified by community. See post 'Timeline' for change history
// Retrieved 2026-03-24, License - CC BY-SA 3.0

char* stristr( const char* str1, const char* str2 )
{
    const char* p1 = str1 ;
    const char* p2 = str2 ;
    const char* r = *p2 == 0 ? str1 : 0 ;

    while( *p1 != 0 && *p2 != 0 )
    {
        if( tolower( (unsigned char)*p1 ) == tolower( (unsigned char)*p2 ) )
        {
            if( r == 0 )
            {
                r = p1 ;
            }

            p2++ ;
        }
        else
        {
            p2 = str2 ;
            if( r != 0 )
            {
                p1 = r + 1 ;
            }

            if( tolower( (unsigned char)*p1 ) == tolower( (unsigned char)*p2 ) )
            {
                r = p1 ;
                p2++ ;
            }
            else
            {
                r = 0 ;
            }
        }

        p1++ ;
    }

    return *p2 == 0 ? (char*)r : 0 ;
}

/* =============================================================================
 * 1.  PORT TABLE
 *     List every GPIO port present on your specific package.
 *     Remove entries for ports your device doesn't have.
 * ===========================================================================*/

typedef struct {
    GPIO_TypeDef *port;
    const char   *prefix;   /* "PA", "PB", … used when building "PA6" labels */
} PortEntry_t;

static const PortEntry_t port_table[] = {
    { GPIOA, "PA" },
    { GPIOB, "PB" },
    { GPIOC, "PC" },
    { GPIOD, "PD" },
    { GPIOE, "PE" },
    /* Uncomment if your package includes these ports:
    { GPIOF, "PF" },
    { GPIOG, "PG" },
    { GPIOH, "PH" },
    { GPIOI, "PI" },
    { GPIOJ, "PJ" },
    { GPIOK, "PK" },
    */
};
#define NUM_PORTS  ( sizeof(port_table) / sizeof(port_table[0]) )


/* =============================================================================
 * 2.  PIN ALIAS TABLE
 *     Assign human-readable names to the pins your application uses.
 *     Format:  { port_index, pin, "NAME" }
 *     port_index is the row in port_table[] above  (0 = PA, 1 = PB, …)
 * ===========================================================================*/

typedef struct {
    uint8_t     port_idx;   /* index into port_table[] */
    uint8_t     pin;        /* 0 – 15                  */
    const char *name;       /* descriptive label        */
} PinAlias_t;

static const PinAlias_t alias_table[] = {
    /* ── port 0 = GPIOA ───────────────────── */
    { 0,  0, "B1"                 },   /* PA0   */
    { 0,  4, "I2S3_WS"            },   /* PA4   */
    { 0,  5, "SPI1_SCK"           },   /* PA5   */
    { 0,  6, "SPI1_MISO"          },   /* PA6   */
    { 0,  9, "VBUS_FS"            },   /* PA9   */
    { 0, 10, "OTG_FS_ID"          },   /* PA10  */
    { 0, 11, "OTG_FS_DM"          },   /* PA11  */
    { 0, 12, "OTG_FS_DP"          },   /* PA12  */
    { 0, 13, "SWDIO"              },   /* PA13  */
    { 0, 14, "SWCLK"              },   /* PA14  */

    /* ── port 1 = GPIOB ───────────────────── */
    { 1,  2, "BOOT1"              },   /* PB2   */
    { 1,  3, "SWO"                },   /* PB3   */
    { 1,  6, "Audio_SCL"          },   /* PB6   */
    { 1,  9, "Audio_SDA"          },   /* PB9   */
    { 1, 10, "CLK_IN"             },   /* PB10  */

    /* ── port 2 = GPIOC ───────────────────── */
    { 2,  0, "OTG_FS_PowerSwitchOn" },   /* PC0   */
    { 2,  3, "PDM_OUT"            },   /* PC3   */
    { 2, 14, "PC14_OSC32_IN"      },   /* PC14  */
    { 2, 15, "PC15_OSC32_OUT"     },   /* PC15  */

    /* ── port 3 = GPIOD ───────────────────── */
    { 3,  4, "Audio_RST"          },   /* PD4   */
    { 3,  5, "OTG_FS_OverCurrent" },   /* PD5   */
    { 3, 12, "LD4"                },   /* PD12  */
    { 3, 13, "LD3"                },   /* PD13  */
    { 3, 14, "LD5"                },   /* PD14  */
    { 3, 15, "LD6"                },   /* PD15  */

    /* ── port 4 = GPIOE ───────────────────── */
    { 4,  1, "MEMS_INT2"          },   /* PE1   */
    { 4,  3, "CS_I2C_SPI"         },   /* PE3   */

    /* ── port 7 = GPIOH ───────────────────── */
    { 7,  0, "PH0_OSC_IN"         },   /* PH0   */
    { 7,  1, "PH1_OSC_OUT"        },   /* PH1   */







};
#define NUM_ALIASES  ( sizeof(alias_table) / sizeof(alias_table[0]) )


/* =============================================================================
 * 3.  DECODE HELPERS
 * ===========================================================================*/

/* MODER[1:0] → 6-char fixed-width label */
static const char *decode_mode(uint32_t m2)
{
    switch (m2) {
        case 0:  return "IN    ";
        case 1:  return "OUT   ";
        case 2:  return "ALT   ";
        case 3:  return "ANALOG";
        default: return "?     ";
    }
}

/* PUPDR[1:0] → label */
static const char *decode_pupd(uint32_t p2)
{
    switch (p2) {
        case 0:  return "NONE";
        case 1:  return "PU  ";
        case 2:  return "PD  ";
        default: return "RSVD";
    }
}

/* OTYPER bit → label  (output type, relevant when MODE = OUT or ALT) */
static const char *decode_otype(uint32_t bit)
{
    return bit ? "OD" : "PP";   /* Open-Drain vs Push-Pull */
}

/* Search alias_table; returns pointer to name string, or "" if not found */
static const char *find_alias(uint8_t port_idx, uint8_t pin)
{
    for (uint8_t i = 0; i < NUM_ALIASES; i++) {
        if (alias_table[i].port_idx == port_idx &&
            alias_table[i].pin      == pin)
        {
            return alias_table[i].name;
        }
    }
    return "";
}


/* =============================================================================
 * 4.  PRINT ONE PIN ROW
 *
 *  Column layout (suits an 80-char terminal):
 *
 *  Col  0  PIN      6 chars   "PA6   "
 *  Col  6  NAME    16 chars   "SPI1_MISO      "
 *  Col 22  MODE     8 chars   "ALT     "
 *  Col 30  AF       5 chars   "AF5  " or "--   "
 *  Col 35  OTYP     4 chars   "PP  " or "OD  "
 *  Col 39  PUPD     6 chars   "NONE  "
 *  Col 45  IDR      5 chars   "1    "
 *  Col 50  ODR      5 chars   "0    "
 *  Col 55  FUNCTION           alias, "GPIO_IN", "GPIO_OUT", "ANALOG"
 * ===========================================================================*/

static void print_pin_row(uint8_t port_idx, uint8_t pin,
                          uint32_t moder, uint32_t idr, uint32_t odr,
                          uint32_t afrl,  uint32_t afrh,
                          uint32_t pupdr, uint32_t otyper)
{
    char buf[12];

    uint32_t mode  = (moder  >> (pin * 2u)) & 0x3u;
    uint32_t pupd  = (pupdr  >> (pin * 2u)) & 0x3u;
    uint32_t otype = (otyper >>  pin)        & 0x1u;
    uint8_t  ibit  = (uint8_t)((idr >> pin) & 0x1u);
    uint8_t  obit  = (uint8_t)((odr >> pin) & 0x1u);
    uint8_t  af    = (pin < 8u)
                   ? (uint8_t)((afrl >> (pin        * 4u)) & 0xFu)
                   : (uint8_t)((afrh >> ((pin - 8u) * 4u)) & 0xFu);

    const char *alias = find_alias(port_idx, pin);

    /* ── PIN ───────────── col 0 */
    snprintf(buf, sizeof(buf), "%s%u", port_table[port_idx].prefix, pin);
    print(buf);
    tabTo(6);

    /* ── NAME ──────────── col 6 */
    print(alias);
    tabTo(22);

    /* ── MODE ──────────── col 22 */
    print(decode_mode(mode));
    tabTo(30);

    /* ── AF number ─────── col 30 */
    if (mode == 2u) {
        print("AF");
        printDec(af);
    } else {
        print("--");
    }
    tabTo(35);

    /* ── Output type ───── col 35  (only meaningful for OUT/ALT) */
    if (mode == 1u || mode == 2u) {
        print(decode_otype(otype));
    } else {
        print("--");
    }
    tabTo(40);

    /* ── Pull ──────────── col 39 */
    print(decode_pupd(pupd));
    tabTo(46);

    /* ── IDR ───────────── col 45 */
    printDec(ibit);
    tabTo(51);

    /* ── ODR ───────────── col 50 */
    printDec(obit);
    tabTo(56);

    /* ── FUNCTION ──────── col 55 */
    if (alias[0] != '\0') {
        /* User gave this pin a name — that IS the function description */
        print(alias);
    } else {
        /* Generic label based on mode */
        switch (mode) {
            case 0:  print("GPIO_IN");          break;
            case 1:  print("GPIO_OUT");         break;
            case 2:
                print("AF");
                printDec(af);
                print(" (unnamed)");
                break;
            case 3:  print("ANALOG");           break;
            default: print("?");                break;
        }
    }

    maybeCr();
}


/* =============================================================================
 * 5a.  gpio_dump_all()
 *      Prints every pin on every port, regardless of mode.
 * ===========================================================================*/

static char * pin_match = NULL;

void gpio_dump_all(void)
{
    print("PIN   NAME            MODE    AF   OTYP PUPD  IDR  ODR  FUNCTION\n");
    print("----------------------------------------------------------------------\n");

    pin_match = (char *)parseWord(0);
    for (uint8_t p = 0; p < NUM_PORTS; p++) {
        GPIO_TypeDef *port = port_table[p].port;

        /* Read all registers once — guarantees a consistent snapshot */
        uint32_t moder  = port->MODER;
        uint32_t idr    = port->IDR;
        uint32_t odr    = port->ODR;
        uint32_t afrl   = port->AFR[0];
        uint32_t afrh   = port->AFR[1];
        uint32_t pupdr  = port->PUPDR;
        uint32_t otyper = port->OTYPER;

        bool port_printed = false;
        for (uint8_t pin = 0; pin < 16u; pin++) {
            const char *alias = find_alias(p, pin);
            if (pin_match) {
                const char *name = stristr(alias, pin_match);
                if (name == NULL) {
                    char buf[6];
                    snprintf(buf, sizeof(buf), "%s%u", port_table[p].prefix, pin);
                    name = stristr(buf, pin_match);
                    if (name == NULL)
                        continue;
                }
            }
            print_pin_row(p, pin, moder, idr, odr, afrl, afrh, pupdr, otyper);
            port_printed = true;
        }

        if (port_printed)  print("\r\n");  /* blank line between ports */
    }
}


/* =============================================================================
 * 5b.  gpio_dump_active()
 *      Skips pins still in their power-on reset state (ANALOG, no pull,
 *      push-pull) unless they appear in the alias table.
 *      Useful to cut clutter on densely-pinned devices.
 * ===========================================================================*/

void gpio_dump_active(void)
{
    print("PIN   NAME            MODE    AF   OTYP PUPD  IDR  ODR  FUNCTION\r\n");
    print("----------------------------------------------------------------------\r\n");

    for (uint8_t p = 0; p < NUM_PORTS; p++) {
        GPIO_TypeDef *port = port_table[p].port;

        uint32_t moder  = port->MODER;
        uint32_t idr    = port->IDR;
        uint32_t odr    = port->ODR;
        uint32_t afrl   = port->AFR[0];
        uint32_t afrh   = port->AFR[1];
        uint32_t pupdr  = port->PUPDR;
        uint32_t otyper = port->OTYPER;

        for (uint8_t pin = 0; pin < 16u; pin++) {
            uint32_t mode = (moder >> (pin * 2u)) & 0x3u;
            uint32_t pupd = (pupdr >> (pin * 2u)) & 0x3u;
            uint8_t  otyp = (uint8_t)((otyper >> pin) & 0x1u);

            /* "Reset state" on F4/F7: ANALOG, no pull, push-pull (otyp=0) */
            int is_reset_state = (mode == 3u) && (pupd == 0u) && (otyp == 0u);

            /* Always print if aliased, or if not in reset state */
            if (!is_reset_state || find_alias(p, pin)[0] != '\0') {
                print_pin_row(p, pin, moder, idr, odr, afrl, afrh, pupdr, otyper);
            }
        }

        print("\r\n");
    }
}


/* =============================================================================
 * 6.  gpio_dump_port()
 *     Dump a single port by index (0=PA, 1=PB, …).
 *     Handy for spot-checking one port without the full listing.
 * ===========================================================================*/

void gpio_dump_port(uint8_t port_idx)
{
    if (port_idx >= NUM_PORTS) {
        print("gpio_dump_port: invalid index\r\n");
        return;
    }

    GPIO_TypeDef *port = port_table[port_idx].port;

    print("PIN   NAME            MODE    AF   OTYP PUPD  IDR  ODR  FUNCTION\r\n");
    print("----------------------------------------------------------------------\r\n");

    uint32_t moder  = port->MODER;
    uint32_t idr    = port->IDR;
    uint32_t odr    = port->ODR;
    uint32_t afrl   = port->AFR[0];
    uint32_t afrh   = port->AFR[1];
    uint32_t pupdr  = port->PUPDR;
    uint32_t otyper = port->OTYPER;

    for (uint8_t pin = 0; pin < 16u; pin++) {
        print_pin_row(port_idx, pin, moder, idr, odr, afrl, afrh, pupdr, otyper);
    }
}