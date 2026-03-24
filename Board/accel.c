// accel.c — LIS302DL / LIS3DSH accelerometer driver for STM32F4DISCOVERY
//
// Hardware (STM32F4DISCOVERY schematic):
//   SPI1 via LL: SCK=PA5  MISO=PA6  MOSI=PA7  CS=PE3
//   CubeMX configures SPI1 MOSI as PB5, but PA7 is physically wired to the
//   accelerometer chip.  Before every SPI transaction PA7 is switched from
//   ETH_CRS_DV (AF11) to SPI1_MOSI (AF5) by writing the AFRL register
//   directly; afterwards it is restored.
//   To prevent the LAN8720A from fighting PA7, the PHY ISOLATE bit (BCR[10])
//   is set before the switch and cleared after restoration — the PHY's CRS_DV
//   driver goes high-Z while ISOLATE is asserted, and the 50 MHz REFCLKO
//   continues to run, so the MAC DMA clock is never interrupted.
//   CS (PE3) is managed manually as GPIO.
//
// Chip detection (same SPI pins, same CS on all Discovery board revisions):
//   WHO_AM_I register (0x0F):
//     0x3F → LIS3DSH  — newer boards (post ~2012), 16-bit output
//     0x3B → LIS302DL — older boards, 8-bit output
//
// Timing architecture:
//
//   LIS3DSH — hardware FIFO driven (preferred path):
//     LIS3DSH runs at 100 Hz ODR, FIFO in stream mode, watermark=25 samples.
//     When 25 samples accumulate the chip asserts INT1 (PE0).
//     EXTI0_IRQHandler clears the flag and calls later(accel_batch_process).
//     accel_batch_process() (event-loop context):
//       1. Isolates PHY, switches PA7 to AF5 (SPI1_MOSI).
//       2. Reads FIFO_SRC to get count; drains all samples in one burst.
//       3. Restores PA7 to AF11 (ETH_CRS_DV), de-isolates PHY.
//       4. Runs IIR filter and tap check across all samples.
//       5. Calls http_accel_push() if orientation changed or tap detected.
//     Ethernet is disrupted for ≈ 344 µs once every 250 ms (0.14%).
//
//   LIS302DL — timer-driven (fallback, older boards):
//     in(msec(40), accel_read_isr) fires from the timer interrupt.  The ISR
//     only reschedules itself (while accel_running is true) and defers real
//     work via later(accel_process_one).
//     accel_process_one() (event-loop context) performs the same PHY-isolate /
//     single-sample SPI read / restore sequence, then pushes.
//
// Enable / disable:
//   accel_start()  — arms the hardware interrupt (LIS3DSH) or the timer
//                    (LIS302DL) and sets accel_running = true.
//   accel_stop()   — disables EXTI0 (LIS3DSH) or clears accel_running
//                    (LIS302DL); the next ISR invocation will not reschedule.
//
// Push policy:
//   http_accel_push() is called only when |Δpitch| or |Δroll| > 1.0°,
//   or immediately on a tap event.  Zero traffic when the board is still.

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "main.h"                    // pin/port definitions, HAL, LL headers
#include "stm32f4xx_ll_spi.h"        // LL SPI functions (not in main.h chain)
#include "lwip.h"                    // extern ETH_HandleTypeDef heth
#include "tea.h"
#include "cli.h"
#include "printers.h"
#include "http_server.h"
#include "accel.h"

// ── SPI CS — managed manually ─────────────────────────────────────────────────

#define CS_LOW()   LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_3)
#define CS_HIGH()  LL_GPIO_SetOutputPin(GPIOE,   LL_GPIO_PIN_3)

// ── PA7 alternate-function helpers ────────────────────────────────────────────
//
// PA7 is physically wired to both LIS3DSH MOSI and LAN8720A ETH_CRS_DV.
// Only the AF register is changed; the pin stays push-pull, no-pull,
// very-high-speed as configured by HAL_ETH_MspInit in ethernetif.c.

static void pa7_as_spi_mosi(void)
{
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_7, LL_GPIO_AF_5);   // SPI1_MOSI
}

static void pa7_as_eth_crs_dv(void)
{
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_7, LL_GPIO_AF_11);  // ETH_CRS_DV
}

// ── LAN8720A PHY gating ───────────────────────────────────────────────────────
//
// BCR bit 10 (ISOLATE) puts CRS_DV, RXD[1:0] and RXER into high-Z.
// REFCLKO (50 MHz → PA1) continues to run — MAC DMA clock is unaffected.

#define PHY_ADDR    0x00U       // matches LAN8720_PHY_ADDRESS in ethernetif.c
#define PHY_BCR     0x00U       // Basic Control Register
#define PHY_ISOLATE 0x0400U     // BCR bit 10

// accel_spi_begin: isolate PHY, switch PA7 to SPI1_MOSI.
//                 Returns original BCR value for restoration.
static uint32_t accel_spi_begin(void)
{
    uint32_t bmcr = 0;
    HAL_ETH_ReadPHYRegister(&heth, PHY_ADDR, PHY_BCR, &bmcr);
    HAL_ETH_WritePHYRegister(&heth, PHY_ADDR, PHY_BCR, bmcr | PHY_ISOLATE);
    pa7_as_spi_mosi();
    return bmcr;
}

// accel_spi_end: restore PA7 to ETH_CRS_DV, de-isolate PHY.
static void accel_spi_end(uint32_t bmcr)
{
    pa7_as_eth_crs_dv();
    HAL_ETH_WritePHYRegister(&heth, PHY_ADDR, PHY_BCR, bmcr & ~PHY_ISOLATE);
}

// ── LL SPI1 primitive ─────────────────────────────────────────────────────────
//
// Single full-duplex byte exchange on SPI1.  Caller holds CS and has already
// called accel_spi_begin() so PA7 is in AF5.
// SPI1 must be enabled (LL_SPI_Enable) before first use; accel_init() does this.

static uint8_t spi1_txrx(uint8_t data)
{
    while (!LL_SPI_IsActiveFlag_TXE(SPI1));
    LL_SPI_TransmitData8(SPI1, data);
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
    return LL_SPI_ReceiveData8(SPI1);
}

// ── SPI primitives (call only between accel_spi_begin / accel_spi_end) ────────

// Single-byte register read.
static uint8_t lis_read(uint8_t reg)
{
    CS_LOW();
    spi1_txrx(0x80u | reg);
    uint8_t val = spi1_txrx(0x00u);
    while (LL_SPI_IsActiveFlag_BSY(SPI1));
    CS_HIGH();
    return val;
}

// Single-byte register write.
static void lis_write(uint8_t reg, uint8_t val)
{
    CS_LOW();
    spi1_txrx(reg & 0x7Fu);    // R/W=0
    spi1_txrx(val);
    while (LL_SPI_IsActiveFlag_BSY(SPI1));
    CS_HIGH();
}

// Small burst read — cmd byte (R|MS|addr) then n data bytes.
// Suitable for n ≤ 7 (single sample, WHO_AM_I).
static void lis_read_burst(uint8_t cmd, uint8_t *buf, int n)
{
    CS_LOW();
    spi1_txrx(cmd);
    for (int i = 0; i < n; i++)
        buf[i] = spi1_txrx(0x00u);
    while (LL_SPI_IsActiveFlag_BSY(SPI1));
    CS_HIGH();
}

// Large burst read — cmd byte then n data bytes.
// Used for FIFO batch reads where n can reach 32 × 6 = 192 bytes.
static void lis_read_fifo(uint8_t cmd, uint8_t *buf, uint16_t n)
{
    CS_LOW();
    spi1_txrx(cmd);
    for (uint16_t i = 0; i < n; i++)
        buf[i] = spi1_txrx(0x00u);
    while (LL_SPI_IsActiveFlag_BSY(SPI1));
    CS_HIGH();
}

// ── LIS3DSH registers (WHO_AM_I = 0x3F) ──────────────────────────────────────

#define LIS3DSH_CTRL_REG4  0x20   // ODR[7:4], BDU[3], ZEN[2], YEN[1], XEN[0]
#define LIS3DSH_CTRL_REG5  0x24   // BW[7:6], FSCALE[5:3]
#define LIS3DSH_CTRL_REG6  0x25   // BOOT[7], FIFO_EN[6], WTM_EN[5], ADD_INC[4]
#define LIS3DSH_FIFO_CTRL  0x2E   // FMODE[7:5], FTH[4:0]
#define LIS3DSH_FIFO_SRC   0x2F   // WTM[7], OVRN[6], EMPTY[5], FSS[4:0]
#define LIS3DSH_OUT_X_L    0x28   // first of 6 contiguous output bytes

// CTRL_REG4: ODR=0110b (100 Hz), BDU=1, all axes  →  0x6F
// CTRL_REG5: BW=800 Hz, FS=±2 g (defaults)        →  0x00
// CTRL_REG6: FIFO_EN=1, WTM_EN=1                  →  0x60
// FIFO_CTRL: Stream mode (FMODE=010), watermark=25 →  0x59
#define LIS3DSH_CTRL4_VAL     0x6Fu
#define LIS3DSH_CTRL5_VAL     0x00u
#define LIS3DSH_CTRL6_VAL     (0x40u | 0x20u)          // FIFO_EN | WTM_EN
#define LIS3DSH_FIFO_CTRL_VAL ((0x02u << 5) | 25u)     // stream + WTM=25

#define LIS3DSH_FIFO_WTM      25u  // samples per batch → 4 Hz push rate at 100 Hz ODR

// Burst address for FIFO reads: R/W=1, MS=1 (auto-increment 0x28–0x2D per sample).
#define LIS3DSH_BURST_CMD  (0x80u | 0x40u | LIS3DSH_OUT_X_L)

// Sensitivity at FS=±2 g: 2 g / 32768 LSB.
#define LIS3DSH_SENS  0.000061f

// ── LIS302DL registers (WHO_AM_I = 0x3B) ─────────────────────────────────────

#define LIS302DL_CTRL_REG1  0x20   // PD[5], FS[4], ZEN[2], YEN[1], XEN[0]
#define LIS302DL_OUT_X      0x29   // 8-bit signed, non-contiguous

// CTRL_REG1: PD=1 (active), FS=0 (±2 g), all axes  →  0x47
#define LIS302DL_CTRL1_VAL  0x47u

// Burst from 0x29 yields 5 bytes; valid data at indices 0 (X), 2 (Y), 4 (Z).
#define LIS302DL_BURST_CMD  (0x80u | 0x40u | LIS302DL_OUT_X)

// Sensitivity at FS=±2 g: 18 mg/digit.
#define LIS302DL_SENS  0.018f

// Sample period for LIS302DL timer path (ms).
#define SAMPLE_MS  40u

// ── Shared state ──────────────────────────────────────────────────────────────

static accel_type_t accel_type  = ACCEL_NONE;
static uint8_t      detected_id = 0;

// Batch buffer: holds up to 32 LIS3DSH samples (32 × 6 bytes).
// Also reused for a single LIS302DL sample (5 bytes).
static uint8_t raw_batch[32u * 6u];

// IIR filter state and derived angles — written only by event-loop functions.
static float    ax_f = 0.0f, ay_f = 0.0f, az_f = 1.0f;
static float    last_pitch = 0.0f, last_roll = 0.0f;

// Tap state — set inside accel_update_sample(), cleared after push.
static bool     tap_pending  = false;
static uint32_t tap_last_ms  = 0;
static uint32_t tap_count    = 0;

// Running sample counter for show_acc().
static uint32_t sample_count = 0;

// Task running state.  Set by accel_start(), cleared by accel_stop().
// The LIS302DL ISR checks this before rescheduling.
static volatile bool accel_running = false;

// IIR coefficient — α=0.15.
// At 100 Hz ODR this gives a ~62 ms time constant.
#define ALPHA        0.15f

// Minimum angle change (degrees) that triggers an SSE push.
#define ANGLE_THRESH 1.0f

// Tap: instantaneous magnitude must exceed filtered baseline by this much (g).
#define TAP_THRESH_G 0.35f

// Minimum time between consecutive tap events (ms).
#define TAP_DEBOUNCE 400u

// ── Common per-sample processing ──────────────────────────────────────────────

static void accel_update_sample(float ax, float ay, float az)
{
    ax_f += ALPHA * (ax - ax_f);
    ay_f += ALPHA * (ay - ay_f);
    az_f += ALPHA * (az - az_f);

    float mag_raw = sqrtf(ax*ax + ay*ay + az*az);
    float mag_flt = sqrtf(ax_f*ax_f + ay_f*ay_f + az_f*az_f);
    uint32_t now  = HAL_GetTick();

    if ((mag_raw - mag_flt) > TAP_THRESH_G &&
        (now - tap_last_ms) > TAP_DEBOUNCE) {
        tap_pending = true;
        tap_last_ms = now;
        tap_count++;
    }
}

static void accel_maybe_push(void)
{
    float pitch = atan2f(ay_f, sqrtf(ax_f*ax_f + az_f*az_f)) * (180.0f / 3.14159265f);
    float roll  = atan2f(-ax_f, az_f)                         * (180.0f / 3.14159265f);

    bool changed = (fabsf(pitch - last_pitch) > ANGLE_THRESH) ||
                   (fabsf(roll  - last_roll)  > ANGLE_THRESH);
    bool tap     = tap_pending;
    tap_pending  = false;

    if (changed || tap) {
        http_accel_push((int16_t)(pitch * 10.0f), (int16_t)(roll * 10.0f), tap);
        if (changed) { last_pitch = pitch; last_roll = roll; }
    }
}

// ── LIS3DSH path: FIFO batch (event-loop) ────────────────────────────────────

static void accel_batch_process(void)
{
    uint32_t bmcr = accel_spi_begin();

    uint8_t fifo_src = lis_read(LIS3DSH_FIFO_SRC);
    uint8_t count    = fifo_src & 0x1Fu;   // FSS[4:0]
    if (count == 0) count = 32u;           // FSS=0 when FIFO exactly full

    lis_read_fifo(LIS3DSH_BURST_CMD, raw_batch, (uint16_t)(count * 6u));

    accel_spi_end(bmcr);

    tap_pending = false;
    for (uint8_t i = 0; i < count; i++) {
        uint8_t *s  = raw_batch + (size_t)i * 6u;
        int16_t  rx = (int16_t)(((uint16_t)s[1] << 8) | s[0]);
        int16_t  ry = (int16_t)(((uint16_t)s[3] << 8) | s[2]);
        int16_t  rz = (int16_t)(((uint16_t)s[5] << 8) | s[4]);
        accel_update_sample(rx * LIS3DSH_SENS, ry * LIS3DSH_SENS, rz * LIS3DSH_SENS);
    }
    sample_count += count;

    accel_maybe_push();
}

// ── LIS302DL path: timer ISR + single-sample event-loop ──────────────────────

static void accel_process_one(void)
{
    uint32_t bmcr = accel_spi_begin();
    lis_read_burst(LIS302DL_BURST_CMD, raw_batch, 5);
    accel_spi_end(bmcr);

    float ax = (float)(int8_t)raw_batch[0] * LIS302DL_SENS;
    float ay = (float)(int8_t)raw_batch[2] * LIS302DL_SENS;
    float az = (float)(int8_t)raw_batch[4] * LIS302DL_SENS;

    tap_pending = false;
    accel_update_sample(ax, ay, az);
    sample_count++;

    accel_maybe_push();
}

void accel_regs_read(void) // ( a n )
{
    Byte n = ret();
    Byte address = ret() | 0x80u;
    Byte data[n];

    uint32_t bmcr = accel_spi_begin();
    lis_read_burst(address, data, n);
    accel_spi_end(bmcr);

    hbytes(data, n);
}

static void accel_read_isr(void)
{
    if (!accel_running) return;     // accel_stop() clears this; skip reschedule
    in(msec(SAMPLE_MS), accel_read_isr);
    later(accel_process_one);
}

// ── INT1 ISR trampoline (LIS3DSH FIFO path) ──────────────────────────────────

void accel_int1_isr(void)
{
    later(accel_batch_process);
}

// ── Public API ────────────────────────────────────────────────────────────────

void accel_init(void)
{
    static bool once = false;
    if (once == false) { once = true; namedAction(accel_read_isr); }

    CS_HIGH();

    // Enable SPI1 peripheral (MX_SPI1_Init configures but does not enable it).
    LL_SPI_Enable(SPI1);

    uint32_t bmcr = accel_spi_begin();
    detected_id   = lis_read(REG_WHO_AM_I);

    if (detected_id == 0x3Fu) {
        accel_type = ACCEL_LIS3DSH;
        lis_write(LIS3DSH_CTRL_REG4, LIS3DSH_CTRL4_VAL);   // 100 Hz, BDU, all axes
        lis_write(LIS3DSH_CTRL_REG5, LIS3DSH_CTRL5_VAL);   // ±2 g, 800 Hz AA
    } else { //if (detected_id == 0x3Bu) {
        accel_type = ACCEL_LIS302DL;
        lis_write(LIS302DL_CTRL_REG1, LIS302DL_CTRL1_VAL); // active, ±2 g, all axes
    // } else {
    //     accel_type = ACCEL_NONE;
    }

    accel_spi_end(bmcr);
}

void accel_start(void)
{
    accel_running = true;

    if (accel_type == ACCEL_LIS3DSH) {
        uint32_t bmcr = accel_spi_begin();
        lis_write(LIS3DSH_CTRL_REG6,  LIS3DSH_CTRL6_VAL);    // FIFO_EN | WTM_EN
        lis_write(LIS3DSH_FIFO_CTRL,  LIS3DSH_FIFO_CTRL_VAL);// stream + WTM=25
        accel_spi_end(bmcr);

        LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTE, LL_SYSCFG_EXTI_LINE0);
        LL_EXTI_InitTypeDef exti = {0};
        exti.Line_0_31   = LL_EXTI_LINE_0;
        exti.LineCommand = ENABLE;
        exti.Mode        = LL_EXTI_MODE_IT;
        exti.Trigger     = LL_EXTI_TRIGGER_RISING;
        LL_EXTI_Init(&exti);
        HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    } else {
        // LIS302DL: timer-driven
        in(msec(SAMPLE_MS), accel_read_isr);
    }
}

void accel_stop(void)
{
    accel_running = false;
    if (accel_type == ACCEL_LIS3DSH)
        HAL_NVIC_DisableIRQ(EXTI0_IRQn);
    // LIS302DL: accel_read_isr will see accel_running==false and not reschedule.
}

// ── show_acc ──────────────────────────────────────────────────────────────────
//
// Prints chip type, task state, SPI1 bus status, orientation and counters.

void show_acc(void)
{
    // ── Chip and task state ──
    print("Accel:   ");
    switch (accel_type) {
        case ACCEL_LIS3DSH:
            print("LIS3DSH (FIFO/INT1, 100 Hz ODR, WTM=");
            printDec(LIS3DSH_FIFO_WTM);
            print(")");
            break;
        case ACCEL_LIS302DL:
            print("LIS302DL (timer, 25 Hz)");
            break;
        default:
            print("not found");
            break;
    }
    print("  WHO_AM_I=0x"); dotnb(2, 2, detected_id, 16); printCr();

    print("Task:    ");
    if (accel_type == ACCEL_NONE) {
        print("not started"); printCr();
        return;
    }
    if (!accel_running) {
        print("stopped"); printCr();
    } else if (accel_type == ACCEL_LIS3DSH) {
        // For LIS3DSH, cross-check the NVIC IRQ enable bit as the ground truth.
        print(NVIC_GetEnableIRQ(EXTI0_IRQn) ? "running (EXTI0 enabled)"
                                             : "running (EXTI0 disabled — stalled)");
        printCr();
    } else {
        print("running (timer)"); printCr();
    }

    // ── SPI1 bus status ──
    print("SPI1:    ");
    if (!LL_SPI_IsEnabled(SPI1)) {
        print("disabled"); printCr();
    } else {
        // Baud rate: CR1[5:3] encodes prescaler as log2(div)-1.
        // LL_SPI_GetBaudRatePrescaler returns the raw masked field (multiple of 8).
        uint32_t br  = LL_SPI_GetBaudRatePrescaler(SPI1);
        uint32_t div = 2u << (br >> 3);   // DIV2→4→8→16→32→64→128→256

        // SPI mode from CPOL and CPHA bits.
        uint32_t cpol = LL_SPI_GetClockPolarity(SPI1) ? 1u : 0u;
        uint32_t cpha = LL_SPI_GetClockPhase(SPI1)    ? 1u : 0u;
        uint32_t mode = (cpol << 1) | cpha;

        // PCLK2 = 84 MHz (APB2 = SYSCLK/2).  Actual SPI clock = 84/div MHz.
        // Report as tenths so we can use integer arithmetic.
        uint32_t khz10 = 840000u / div;   // tenths of kHz  (e.g. 84000/16=5250 → 5.25 MHz)

        print("enabled  mode="); printDec(mode);
        print("  div=");         printDec(div);
        print("  clk=");         printDec0(khz10 / 10000u);
        print(".");              printDec((khz10 / 1000u) % 10u);
        print(" MHz");

        // Error flags
        bool ovr  = LL_SPI_IsActiveFlag_OVR(SPI1)  != 0u;
        bool modf = LL_SPI_IsActiveFlag_MODF(SPI1) != 0u;
        bool bsy  = LL_SPI_IsActiveFlag_BSY(SPI1)  != 0u;
        if (ovr || modf || bsy) {
            print("  [");
            if (bsy)  print("BSY ");
            if (ovr)  print("OVR ");
            if (modf) print("MODF");
            print("]");
        }
        printCr();
    }

    // ── Orientation and counters ──
    print("Pitch:   "); printFloat(last_pitch, 1); print(" deg"); printCr();
    print("Roll:    "); printFloat(last_roll,  1); print(" deg"); printCr();
    print("Samples: "); printDec(sample_count); printCr();
    print("Taps:    "); printDec(tap_count);    printCr();
}
