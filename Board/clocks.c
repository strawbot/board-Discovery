// Clocks — Discovery board-specific clock code (STM32F407, LL driver)
//
// TimbreOS/clocks.c provides: timestamp_to_utc, epoch_to_tm, tm_to_epoch,
// over_due, micro_sleep, print_build_banner, set_delta_alarm, delta_alarm,
// get_ticks, init_clocks.
//
// This file provides the board-specific overrides declared via CLOCK_HAS_*:
//   blink_leds()  — CLOCK_HAS_BLINK: 4-LED colour-wheel (LD3/4/5/6)
//   show_timer()  — CLOCK_HAS_SHOW:  UTC formatted via gmtime_r/strftime
//   set_utc()     — write epoch to the STM32F4 hardware RTC

#include "tea.h"
#include "gpio.h"
#include "printers.h"
#include "cli.h"
#include "project_defs.h"
#include "tim.h"
#include "stm32f4xx_ll_rtc.h"
#include <time.h>
#include <stdlib.h>

// ── 4-LED colour-wheel heartbeat ─────────────────────────────────────────

#define set_pin(pin)   LL_GPIO_SetOutputPin(pin##_GPIO_Port, pin##_Pin)
#define reset_pin(pin) LL_GPIO_ResetOutputPin(pin##_GPIO_Port, pin##_Pin)

#define gron()  set_pin(LD4)
#define groff() reset_pin(LD4)
#define oron()  set_pin(LD3)
#define oroff() reset_pin(LD3)
#define blon()  set_pin(LD6)
#define bloff() reset_pin(LD6)
#define reon()  set_pin(LD5)
#define reoff() reset_pin(LD5)

void blink_leds(void) {
    static enum { GREEN, GREENORANGE, ORANGE, ORANGERED,
                  RED, REDBLUE, BLUE, BLUEGREEN } color = GREEN;
    switch (color) {
    case GREEN:       bloff();  color = GREENORANGE; break;
    case GREENORANGE: oron();   color = ORANGE;      break;
    case ORANGE:      groff();  color = ORANGERED;   break;
    case ORANGERED:   reon();   color = RED;          break;
    case RED:         oroff();  color = REDBLUE;      break;
    case REDBLUE:     blon();   color = BLUE;         break;
    case BLUE:        reoff();  color = BLUEGREEN;    break;
    case BLUEGREEN:   gron();   color = GREEN;        break;
    }
    after(msec(125), blink_leds);
}

// ── show_timer with formatted UTC ────────────────────────────────────────

void show_timer(void) {
    print("  ticks/S:"); printDec(ONE_SECOND);
    print("  Timer:");   dotnb(8, 8, TIM2->CNT, 16);
    print("  UTC:");
    char time_str[100];
    time_t now = (time_t)get_utc();
    printDec((Long)now);
    struct tm tm_buf;
    struct tm *utc_tm = gmtime_r(&now, &tm_buf);
    if (utc_tm == NULL) { print("\n gmtime failed "); return; }
    strftime(time_str, sizeof(time_str), " %Y-%m-%d %H:%M:%S ", utc_tm);
    print(time_str);
}

// ── set_utc — write epoch to STM32F4 hardware RTC ────────────────────────

void set_utc(Long utc) {
    time_t t_val = (time_t)utc;
    struct tm tm_buf;
    struct tm *t = gmtime_r(&t_val, &tm_buf);

    uint8_t wday = (t->tm_wday == 0) ? 7U : (uint8_t)t->tm_wday;

    LL_RTC_DisableWriteProtection(RTC);
    LL_RTC_EnableInitMode(RTC);
    while (!LL_RTC_IsActiveFlag_INIT(RTC)) {}

    LL_RTC_TIME_Config(RTC,
        LL_RTC_TIME_FORMAT_AM_OR_24,
        __LL_RTC_CONVERT_BIN2BCD((uint8_t)t->tm_hour),
        __LL_RTC_CONVERT_BIN2BCD((uint8_t)t->tm_min),
        __LL_RTC_CONVERT_BIN2BCD((uint8_t)t->tm_sec));

    LL_RTC_DATE_Config(RTC,
        wday,
        __LL_RTC_CONVERT_BIN2BCD((uint8_t)t->tm_mday),
        __LL_RTC_CONVERT_BIN2BCD((uint8_t)(t->tm_mon + 1)),
        __LL_RTC_CONVERT_BIN2BCD((uint8_t)(t->tm_year % 100)));

    LL_RTC_DisableInitMode(RTC);
    LL_RTC_EnableWriteProtection(RTC);

    LL_RTC_BAK_SetRegister(RTC, LL_RTC_BKP_DR0, 0x32F2);
}
