// accel.c — LIS3DSH accelerometer driver for STM32F4DISCOVERY
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
// Chip (same SPI pins and CS on all STM32F4DISCOVERY revisions):
//   WHO_AM_I (0x0F): 0x3F = LIS3DSH production, 0x01 = early/engineering sample.
//   Register map is identical; silicon ID is the only difference.
//
// Timing architecture — soft-scheduled FIFO poll:
//   LIS3DSH runs at 100 Hz ODR, FIFO in stream mode, watermark=25 samples.
//   accel_poll() fires from after() every ~100 ms and calls accel_batch_process().
//   At 100 Hz ODR roughly 10 samples accumulate per poll; FIFO_SRC is checked
//   first so an empty FIFO is a no-op.  The FIFO absorbs any jitter in the
//   poll interval — the MCU timing is soft, the chip timing is hard.
//   accel_batch_process() (event-loop context):
//     1. Isolates PHY, switches PA7 to AF5 (SPI1_MOSI).
//     2. Reads FIFO_SRC; drains all samples in one burst.
//     3. Restores PA7 to AF11 (ETH_CRS_DV), de-isolates PHY.
//     4. Runs adaptive IIR filter and tap check across all samples.
//     5. Calls http_accel_push() if orientation changed or tap detected.
//   Ethernet is disrupted for ≈ 344 µs per poll (0.03% at 100 ms interval).
//
// Enable / disable:
//   accel_start() — writes FIFO config registers, schedules first accel_poll().
//   accel_stop()  — clears accel_running; accel_poll() sees the flag and stops.
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

#define PHY_ADDR       0x00U       // matches LAN8720_PHY_ADDRESS in ethernetif.c
#ifndef PHY_BCR
#define PHY_BCR        0x00U       // Basic Control Register
#endif
#ifndef PHY_ISOLATE
#define PHY_ISOLATE    0x0400U     // BCR bit 10
#endif

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

// read 8 bytes from acc fifo; x:2 y:2 z:2 ctrl:status (0x20 is empty bit, 0x40 is overrun)

// ── LIS3DSH registers (WHO_AM_I = 0x3F) ──────────────────────────────────────

#define LIS3DSH_CTRL_REG4  0x20   // ODR[7:4], BDU[3], ZEN[2], YEN[1], XEN[0]
#define LIS3DSH_CTRL_REG5  0x24   // BW[7:6], FSCALE[5:3]
#define LIS3DSH_CTRL_REG6  0x25   // BOOT[7], FIFO_EN[6], WTM_EN[5], ADD_INC[4]
#define LIS3DSH_FIFO_CTRL  0x2E   // FMODE[7:5], FTH[4:0]
#define LIS3DSH_FIFO_SRC   0x2F   // WTM[7], OVRN[6], EMPTY[5], FSS[4:0]
#define LIS3DSH_OUT_X_L    0x28   // first of 6 contiguous output bytes

// CTRL_REG4: ODR=0110b (100 Hz), BDU=1, all axes  →  0x6F
// CTRL_REG5: BW=800 Hz, FS=±2 g (defaults)        →  0x00
// CTRL_REG6: FIFO_EN=1, WTM_EN=1, ADD_INC=1       →  0x70
// FIFO_CTRL: Stream mode (FMODE=010), watermark=25 →  0x59
#define LIS3DSH_CTRL4_VAL     0x6Fu
#define LIS3DSH_CTRL5_VAL     0x00u
#define LIS3DSH_CTRL6_VAL     (0x40u | 0x20u | 0x10u)  // FIFO_EN | WTM_EN | ADD_INC
#define LIS3DSH_FIFO_CTRL_VAL ((0x02u << 5) | 25u)     // stream + WTM=25

#define LIS3DSH_FIFO_WTM      25u  // samples per batch → 4 Hz push rate at 100 Hz ODR

// Burst address for FIFO reads: R/W=1, MS=1 (auto-increment 0x28–0x2D per sample).
#define LIS3DSH_BURST_CMD  (0x80u | LIS3DSH_OUT_X_L)

// Sensitivity at FS=±2 g: 2 g / 32768 LSB.
#define LIS3DSH_SENS  0.000061f

// Poll interval (ms): at 100 Hz ODR ~10 samples accumulate per poll.
// FIFO absorbs jitter — the MCU timing is soft, the chip timing is hard.
#define ACCEL_POLL_MS 100

// ── Shared state ──────────────────────────────────────────────────────────────

static accel_type_t accel_type  = ACCEL_NONE;
static uint8_t      detected_id = 0;

// Batch buffer: holds up to 32 LIS3DSH samples (32 × 6 bytes).
static uint8_t raw_batch[32u * 6u];

// IIR filter state and derived angles — written only by event-loop functions.
static float    ax_f = 0.0f, ay_f = 0.0f, az_f = 1.0f;
static float    last_pitch = 0.0f, last_roll = 0.0f;

// Tap state — set inside accel_update_sample(), cleared after push.
static bool     tap_pending  = false;
static uint32_t tap_last_ms  = 0;
static uint32_t tap_count    = 0;
// Consecutive-sample counter for tap hold-time qualification.
// A tap fires when the delta has been above TAP_THRESH_G for exactly
// TAP_MIN_SAMPLES consecutive samples — then it is ignored for the rest of
// the ring.  Requires a sustained impulse, rejects single-sample SPI
// corruption and brief environmental spikes.
static bool  tap_above = false;
#define TAP_MIN_SAMPLES 3u

// Running sample counter for show_acc().
static uint32_t sample_count = 0;

// Tap-pipeline diagnostic counters — reset on accel_start(), shown by show_acc().
//   discard_count   : samples rejected by the 0.3–3.5 g plausibility gate
//   gate_hi_count   : samples above MAG_GATE_HI (the tap-magnitude zone)
//   tap_thresh_count: rising edges where delta exceeded TAP_THRESH_G
//   tap_debounce_ct : rising edges blocked by the debounce window
//   mag_peak        : highest magnitude seen (helps spot FS saturation)
//   tap_delta_peak  : highest (mag_raw − mag_flt) seen (compare to TAP_THRESH_G)
static uint32_t discard_count    = 0;
static uint32_t gate_hi_count    = 0;
static uint32_t tap_thresh_count = 0;
static uint32_t tap_debounce_ct  = 0;
static float    mag_peak         = 0.0f;
static float    tap_delta_peak   = 0.0f;

// Task running state.  Set by accel_start(), cleared by accel_stop().
// accel_poll() checks this before rescheduling.
static volatile bool accel_running = false;

// IIR coefficients — adaptive based on whether the board is moving.
// When stationary (|‖g‖ − 1g| < STILL_THRESH) the slower coefficient is used,
// which suppresses the angular-noise amplification that appears near the flat
// (horizontal) position: roll = atan2(−ax, az) has ∂roll/∂ax ≈ 57 °/g when
// ax ≈ 0, so even modest X-axis noise becomes a large angle swing.
// When the board is moved the fast coefficient restores quick tracking.
#define ALPHA_MOVE   0.15f   // τ ≈  62 ms at 100 Hz — responsive during motion
#define ALPHA_STILL  0.03f   // τ ≈ 330 ms at 100 Hz — quiet when stationary
#define STILL_THRESH  0.1f   // |‖g‖ − 1g| > this → classify as in motion

// Minimum angle change (degrees) that triggers an SSE push.
#define ANGLE_THRESH 1.0f

// Tap: instantaneous magnitude must exceed filtered baseline by this much (g).
// mag_flt stays near 1.0 g (IIR gate excludes high-magnitude samples), so
// this is effectively a floor on the total spike magnitude of 1g + TAP_THRESH_G.
// Desk vibration / footsteps rarely exceed 0.3 g above baseline; a deliberate
// finger tap on the PCB produces 1.5–3 g above baseline.  Set conservatively.
#define TAP_THRESH_G 1.5f

// Minimum time between consecutive tap events (ms).
#define TAP_DEBOUNCE 400u

// ── Common per-sample processing ──────────────────────────────────────────────

// Orientation gate: samples must be within this band around 1 g to update
// the IIR filter.  Taps and shocks push the raw magnitude well outside this
// window, so they have no effect on the displayed orientation.
// The band is wide enough to pass normal static positions and slow tilts.
#define MAG_GATE_LO  0.85f   // below 1 g — rules out free-fall fragments
#define MAG_GATE_HI  1.15f   // above 1 g — rules out taps / impacts

static void accel_update_sample(float ax, float ay, float az)
{
    float mag_raw = sqrtf(ax*ax + ay*ay + az*az);

    if (mag_raw > mag_peak) mag_peak = mag_raw;

    // Discard samples outside the physically plausible range.
    // Upper bound is the 3-D diagonal of the ±2 g full-scale range:
    // sqrt(3 × 2²) = 3.46 g.  Use 3.5 g to pass genuine tap spikes
    // (observed up to ~2.9 g) while still rejecting SPI corruption.
    if (mag_raw < 0.3f || mag_raw > 3.5f) { discard_count++; return; }

    // Tap detection runs before the orientation gate.
    float    mag_flt = sqrtf(ax_f*ax_f + ay_f*ay_f + az_f*az_f);
    float    delta   = mag_raw - mag_flt;
    uint32_t now     = HAL_GetTick();

    if (delta > tap_delta_peak) tap_delta_peak = delta;

    if (delta > TAP_THRESH_G) {
        if (!tap_above) {
            // Rising edge: first sample above threshold — one tap event.
            tap_above = true;
            tap_thresh_count++;
            if ((now - tap_last_ms) > TAP_DEBOUNCE) {
                tap_pending = true;
                tap_last_ms = now;
                tap_count++;
            } else {
                tap_debounce_ct++;
            }
        }
        // Sustained samples above threshold (ringing): ignored.
    } else {
        tap_above = false;   // signal returned below threshold; next crossing is a new tap
    }

    // Orientation filter: only update when the sample is close to 1 g.
    // Taps, impacts and strong vibration push mag outside the gate and are
    // ignored here, keeping the 3-D model smooth.
    if (mag_raw < MAG_GATE_LO || mag_raw > MAG_GATE_HI) {
        if (mag_raw > MAG_GATE_HI) gate_hi_count++;
        return;
    }

    // Adaptive time constant: slow when the board is stationary (suppresses
    // angular noise near flat), faster when being deliberately tilted.
    float alpha = (fabsf(mag_raw - 1.0f) > STILL_THRESH) ? ALPHA_MOVE : ALPHA_STILL;

    ax_f += alpha * (ax - ax_f);
    ay_f += alpha * (ay - ay_f);
    az_f += alpha * (az - az_f);
}

static void accel_maybe_push(void)
{
    float mag = sqrtf(ax_f*ax_f + ay_f*ay_f + az_f*az_f);
    if (mag < 0.1f) return;                 // no valid orientation without gravity

    float gx = ax_f / mag, gy = ay_f / mag, gz = az_f / mag;

    // Y-up angle formulas — on the STM32F4DISCOVERY the LIS3DSH Y axis is
    // perpendicular to the PCB surface (ay ≈ +1 g when flat on a desk):
    //   pitch = atan2(-gz, sqrt(gx²+gy²))   — positive nose-up
    //   roll  = atan2( gx, sqrt(gy²+gz²))   — positive right-side-up
    // Used only for change detection; the browser receives the raw gravity
    // vector so it can orient the 3-D model without any axis-convention math.
    float pitch = atan2f(-gz, sqrtf(gx*gx + gy*gy)) * (180.0f / 3.14159265f);
    float roll  = atan2f( gx, sqrtf(gy*gy + gz*gz)) * (180.0f / 3.14159265f);

    bool changed = (fabsf(pitch - last_pitch) > ANGLE_THRESH) ||
                   (fabsf(roll  - last_roll)  > ANGLE_THRESH);
    bool tap     = tap_pending;
    tap_pending  = false;

    if (changed || tap) {
        http_accel_push((int16_t)(gx * 1000.0f),
                        (int16_t)(gy * 1000.0f),
                        (int16_t)(gz * 1000.0f), tap);
        if (changed) { last_pitch = pitch; last_roll = roll; }
    }
}

// ── Raw-capture tabulator ─────────────────────────────────────────────────────
//
// Triggered explicitly by accel_capture_start() (CLI / web button).
// Fills table[3][ROWS] with the next ROWS samples then ships them to the
// /graph_stream SSE client as 100-sample chunks via an after() chain.
// Live streaming continues uninterrupted during and after the capture.

#define ROWS          1000
#define GR_CHUNK_SIZE 100u   // samples per SSE capture-chunk frame

// Capture buffer — stores g-values as float regardless of mode.
//   Raw mode:     frx = rx * LIS3DSH_SENS  (direct ADC→g, no filtering)
//   Refined mode: ax_f, ay_f, az_f          (IIR-filtered gravity vector)
static float    table[3][ROWS];
static Short    row = 0;

// Graph-stream sender state — owned by graph_send_next() after() chain.
static uint8_t  gr_ch    = 0;   // 0=X  1=Y  2=Z
static uint16_t gr_start = 0;   // sample offset within current channel

// capture_active — true while a snapshot is filling; cleared by dump_table().
// Never set automatically — must be triggered via accel_capture_start().
static bool capture_active = false;

// graph_mode_raw — selects what live streaming and capture record/send.
//   true  (default) → raw ADC-derived g-values (unfiltered)
//   false           → IIR-filtered g-values (ax_f, ay_f, az_f)
// Changed at run-time by accel_set_graph_raw(), called from POST /graph_mode.
static bool graph_mode_raw = true;

void accel_set_graph_raw(bool raw) { graph_mode_raw = raw; }

void accel_stop(void);                    // forward declaration (defined below)
static void graph_send_next(void);        // forward declaration

// dump_table — called when the capture buffer is full.
// Clears capture_active and starts the chunk-send chain.
// Does NOT stop the accelerometer; live streaming continues.
static void dump_table(void)
{
    capture_active = false;
    gr_ch          = 0;
    gr_start       = 0;
    after(msec(20), graph_send_next);
}

static void tabulate(float x, float y, float z)
{
    if (row == ROWS) return;
    table[0][row] = x;
    table[1][row] = y;
    table[2][row] = z;
    if (++row == ROWS) { dump_table(); row = 0; }
}

// accel_capture_start — begin a 1000-sample snapshot.
// Once the buffer is full the data is shipped to the /graph_stream SSE client
// as chunk messages.  Live streaming continues in parallel throughout.
void accel_capture_start(void)
{
    if (!accel_running)  { print("accel: not running\r\n");        return; }
    if (capture_active)  { print("accel: capture in progress\r\n"); return; }
    row            = 0;
    capture_active = true;
    print("accel: capture started\r\n");
}

// graph_send_next — sends one 100-sample chunk of the captured table to the
// /graph_stream SSE client, then reschedules itself until all three channels
// (X→Y→Z) are sent.  Sends {"done":1} after the last chunk so the browser
// knows to redraw.  Silently skips if no SSE client is connected.
static void graph_send_next(void)
{
    http_graph_push_chunk(gr_ch, gr_start,
                          table[gr_ch] + gr_start, GR_CHUNK_SIZE);
    gr_start += GR_CHUNK_SIZE;
    if (gr_start >= ROWS) {
        gr_start = 0;
        gr_ch++;
        if (gr_ch >= 3) {
            http_graph_done();
            return;
        }
    }
    after(msec(20), graph_send_next);
}

// ── LIS3DSH path: FIFO batch (event-loop) ────────────────────────────────────

static void accel_batch_process(void)
{
    uint32_t bmcr = accel_spi_begin();
    
	// if (lis_read(LIS3DSH_FIFO_SRC) & 0x40u) { // OVRN_FIFO bit: reset
    //     lis_write(LIS3DSH_CTRL_REG6, 0x80 | LIS3DSH_CTRL6_VAL); // restart
	// 	accel_spi_end(bmcr);
	// 	return;
	// }

    // Short count = 0;
    // while((lis_read(LIS3DSH_FIFO_SRC) & 0x20) != 0x20) // EMPTY bit:
        // lis_read_fifo(LIS3DSH_BURST_CMD, raw_batch + count*6, 6u), count++;
    uint8_t fifo_src = lis_read(LIS3DSH_FIFO_SRC);
    uint8_t count    = fifo_src & 0x1Fu;   // FSS[4:0]
    if (count == 0) {
        if (fifo_src & 0x20u) {            // EMPTY bit: nothing to drain
            accel_spi_end(bmcr);
            return;
        }
        count = 32u;                       // FSS=0 when FIFO exactly full
    }
    lis_read_fifo(LIS3DSH_BURST_CMD, raw_batch, (uint16_t)(count * 6u));

    // lis_read_fifo(LIS3DSH_BURST_CMD, raw_batch, 6u);
    accel_spi_end(bmcr);

    for (uint8_t i = 0; i < count; i++) {
        tap_pending = false;
        uint8_t *s  = raw_batch + (size_t)i * 6u;
        int16_t  rx = (int16_t)(((uint16_t)s[1] << 8) | s[0]);
        int16_t  ry = (int16_t)(((uint16_t)s[3] << 8) | s[2]);
        int16_t  rz = (int16_t)(((uint16_t)s[5] << 8) | s[4]);
        // Convert raw ADC counts to g for all downstream processing.
        float frx = rx * LIS3DSH_SENS;
        float fry = ry * LIS3DSH_SENS;
        float frz = rz * LIS3DSH_SENS;

        if (graph_mode_raw) {
            // Raw mode: feed/tabulate the unfiltered g-values first,
            // then run the IIR so orientation display is unaffected.
            http_graph_live_feed(frx, fry, frz);
            if (capture_active) tabulate(rx, ry, rz);
            accel_update_sample(frx, fry, frz);
        } else {
            // Refined mode: IIR update runs first so ax_f/ay_f/az_f
            // are current before feeding the graph and capture buffer.
            accel_update_sample(frx, fry, frz);
            http_graph_live_feed(ax_f, ay_f, az_f);
            if (capture_active) tabulate(frx, fry, frz);
        }
    }

    sample_count += count;

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

void accel_reg_write() { // ( n a )
    Byte address = ret();
    Byte n = ret();
    uint32_t bmcr = accel_spi_begin();
    lis_write(address, n);
    accel_spi_end(bmcr);

}

void xyz_read(void) {
    Byte n = 6;
    Byte address = LIS3DSH_OUT_X_L | 0x80u;
    Byte data[n];

    uint32_t bmcr = accel_spi_begin();
    lis_read_burst(address, data, n);
    accel_spi_end(bmcr);

    print("\nX: "),printDec((short)(data[1]<<8|data[0]));
    print("  Y: "),printDec((short)(data[3]<<8|data[2]));
    print("  Z: "),printDec((short)(data[5]<<8|data[4]));
}

// Soft-scheduled FIFO poll.  Reschedules itself via after() while running;
// stops automatically when accel_running is cleared by accel_stop().
static void accel_poll(void)
{
    if (!accel_running) return;
    in(msec(ACCEL_POLL_MS), accel_poll);
    later(accel_batch_process);
}

// Heartbeat: keeps the browser's green indicator alive when the board is still
// and no orientation data is being pushed.  Sends {"run":1} every second so
// the browser's 3-second timeout is never reached during normal operation.
// Also ensures any browser that (re)connects mid-session goes green within 1 s
// without needing a dedicated "send state on connect" mechanism.
#define HEARTBEAT_MS 1000u

static void accel_heartbeat(void)
{
    if (!accel_running) return;
    after(msec(HEARTBEAT_MS), accel_heartbeat);
    http_accel_state(true);
}

// ── Public API ────────────────────────────────────────────────────────────────
static bool initialized = false;

void accel_init(void)
{
    initialized = true;
    static bool once = false;
    if (once == false) {
        once = true;
        namedAction(accel_poll);
        namedAction(accel_heartbeat);
        namedAction(graph_send_next);
    }

    CS_HIGH();

    // Enable SPI1 peripheral (MX_SPI1_Init configures but does not enable it).
    LL_SPI_Enable(SPI1);

    uint32_t bmcr = accel_spi_begin();
    detected_id   = lis_read(REG_WHO_AM_I);

    if (detected_id == 0x3Fu || detected_id == 0x01u) {
        // 0x3F = production LIS3DSH; 0x01 = early/engineering-sample silicon.
        // Register map is identical — same init sequence for both.
        accel_type = ACCEL_LIS3DSH;
        lis_write(LIS3DSH_CTRL_REG4, LIS3DSH_CTRL4_VAL);   // 100 Hz, BDU, all axes
        lis_write(LIS3DSH_CTRL_REG5, LIS3DSH_CTRL5_VAL);   // ±2 g, 800 Hz AA
    } else {
        accel_type = ACCEL_NONE;
    }

    accel_spi_end(bmcr);
}

void accel_start(void)
{
    if (!initialized)
        accel_init();

    if (accel_type != ACCEL_LIS3DSH) return;   // no recognised chip

    uint32_t bmcr = accel_spi_begin();
    lis_write(LIS3DSH_CTRL_REG6,  LIS3DSH_CTRL6_VAL);     // FIFO_EN | WTM_EN | ADD_INC
    lis_write(LIS3DSH_FIFO_CTRL,  LIS3DSH_FIFO_CTRL_VAL); // stream + WTM=25
    accel_spi_end(bmcr);

    accel_running = true;

    // Reset diagnostic counters, tap edge state, and capture flag.
    sample_count = 0; discard_count = 0; gate_hi_count = 0;
    tap_thresh_count = 0; tap_debounce_ct = 0;
    mag_peak = 0.0f; tap_delta_peak = 0.0f;
    tap_above = false;
    capture_active = false;
    row = 0;

    http_accel_state(true);
    after(msec(ACCEL_POLL_MS), accel_poll);
    after(msec(HEARTBEAT_MS), accel_heartbeat);
}

void accel_stop(void)
{
    accel_running = false;
    http_accel_state(false);   // tell browser immediately — don't wait for timeout
    // accel_poll() sees the cleared flag on its next fire and does not reschedule.
}

bool accel_is_running(void) { return accel_running; }

// ── show_acc ──────────────────────────────────────────────────────────────────
//
// Prints chip type, task state, SPI1 bus status, orientation and counters.

void show_acc(void)
{
    // ── Chip and task state ──
    print("Accel:   ");
    if (accel_type == ACCEL_LIS3DSH) {
        print(detected_id == 0x01u ? "LIS3DSH early" : "LIS3DSH");
        print(" (FIFO poll, 100 Hz ODR, WTM=");
        printDec(LIS3DSH_FIFO_WTM);
        print(")");
    } else {
        print("not found");
    }
    print("  WHO_AM_I=0x"); dotnb(2, 2, detected_id, 16); printCr();

    print("Task:    ");
    if (accel_type == ACCEL_NONE) {
        print("not started"); printCr();
        return;
    }
    print(accel_running ? "running" : "stopped"); printCr();

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

    // ── Filtered acceleration ──
    print("ax:      "); printFloat(ax_f, 3); print(" g"); printCr();
    print("ay:      "); printFloat(ay_f, 3); print(" g"); printCr();
    print("az:      "); printFloat(az_f, 3); print(" g"); printCr();

    // ── Sample pipeline counters ──
    print("Samples: "); printDec(sample_count);  printCr();
    print("Discard: "); printDec(discard_count);
    print("  (outside 0.3-3.5 g)"); printCr();
    print("Gate-hi: "); printDec(gate_hi_count);
    print("  (above "); printFloat(MAG_GATE_HI, 2); print(" g — tap zone)"); printCr();

    // ── Tap diagnostics ──
    print("Tap thr: "); printFloat(TAP_THRESH_G, 3); print(" g  (threshold)"); printCr();
    print("δ|g|peak:"); printFloat(tap_delta_peak, 3);
    print(" g  (peak mag_raw - mag_flt seen)"); printCr();
    print("|g| peak:"); printFloat(mag_peak, 3); print(" g  (peak raw magnitude)"); printCr();
    print("Thr hits:"); printDec(tap_thresh_count);
    print("  debounced: "); printDec(tap_debounce_ct); printCr();
    print("Taps:    "); printDec(tap_count); printCr();
}

// ── show_regs ─────────────────────────────────────────────────────────────────
//
// Reads every LIS3DSH control and status register over SPI and prints a
// human-decoded summary, similar in style to show_acc().
// Safe to call while accel is running — PHY isolation is applied as usual.

void show_regs(void)
{
    uint32_t bmcr = accel_spi_begin();

    uint8_t info1     = lis_read(0x07u);               // INFO1 (factory)
    uint8_t info2     = lis_read(0x08u);               // INFO2 (factory)
    uint8_t who_am_i  = lis_read(REG_WHO_AM_I);
    uint8_t ctrl4     = lis_read(LIS3DSH_CTRL_REG4);
    uint8_t ctrl5     = lis_read(LIS3DSH_CTRL_REG5);
    uint8_t ctrl6     = lis_read(LIS3DSH_CTRL_REG6);
    uint8_t status    = lis_read(0x27u);               // STATUS
    uint8_t out[6];
    lis_read_burst(0x80u | LIS3DSH_OUT_X_L, out, 6);    // OUT_X_L..OUT_Z_H burst
    uint8_t fifo_ctrl = lis_read(LIS3DSH_FIFO_CTRL);
    uint8_t fifo_src  = lis_read(LIS3DSH_FIFO_SRC);

    accel_spi_end(bmcr);

    int16_t rx = (int16_t)(((uint16_t)out[1] << 8) | out[0]);
    int16_t ry = (int16_t)(((uint16_t)out[3] << 8) | out[2]);
    int16_t rz = (int16_t)(((uint16_t)out[5] << 8) | out[4]);

    static const char *odr_tbl[16] = {
        "off","3.125","6.25","12.5","25","50","100","400","800","1600",
        "?","?","?","?","?","?"
    };
    static const char *bw_tbl[4]   = { "800","400","200","50" };
    static const char *fs_tbl[5]   = { "+-2g","+-4g","+-6g","+-8g","+-16g" };
    static const char *fm_tbl[8]   = {
        "bypass","FIFO","stream","stream->FIFO","bypass->stream","?","?","?"
    };

    print("INFO1:    0x"); dotnb(2,2,info1,16);
    print("  INFO2:   0x"); dotnb(2,2,info2,16); printCr();

    print("WHO_AM_I: 0x"); dotnb(2,2,who_am_i,16);
    if      (who_am_i == 0x3Fu) print("  (LIS3DSH production)");
    else if (who_am_i == 0x01u) print("  (LIS3DSH engineering)");
    else                        print("  (UNKNOWN)");
    printCr();

    // CTRL_REG4
    print("CTRL_REG4 [0x20] = 0x"); dotnb(2,2,ctrl4,16);
    print("   ODR="); print(odr_tbl[(ctrl4 >> 4) & 0xFu]); print(" Hz");
    print("  BDU="); printDec((ctrl4 >> 3) & 1u);
    print("  ZEN="); printDec((ctrl4 >> 2) & 1u);
    print("  YEN="); printDec((ctrl4 >> 1) & 1u);
    print("  XEN="); printDec( ctrl4        & 1u); printCr();

    // CTRL_REG5
    uint8_t fscale = (ctrl5 >> 3) & 7u;
    print("CTRL_REG5 [0x24] = 0x"); dotnb(2,2,ctrl5,16);
    print("   BW="); print(bw_tbl[(ctrl5 >> 6) & 3u]); print(" Hz");
    print("  FS="); print(fscale < 5u ? fs_tbl[fscale] : "?"); printCr();

    // CTRL_REG6
    print("CTRL_REG6 [0x25] = 0x"); dotnb(2,2,ctrl6,16);
    print("   BOOT=");    printDec((ctrl6 >> 7) & 1u);
    print("  FIFO_EN=");  printDec((ctrl6 >> 6) & 1u);
    print("  WTM_EN=");   printDec((ctrl6 >> 5) & 1u);
    print("  ADD_INC=");  printDec((ctrl6 >> 4) & 1u);
    print("  I2C_DIS=");  printDec((ctrl6 >> 3) & 1u); printCr();

    // STATUS
    print("STATUS    [0x27] = 0x"); dotnb(2,2,status,16);
    print("   ZYXDA="); printDec((status >> 3) & 1u);
    print("  ZDA=");    printDec((status >> 2) & 1u);
    print("  YDA=");    printDec((status >> 1) & 1u);
    print("  XDA=");    printDec( status        & 1u);
    print("  ZYXOR=");  printDec((status >> 7) & 1u); printCr();

    // FIFO_CTRL
    print("FIFO_CTRL [0x2E] = 0x"); dotnb(2,2,fifo_ctrl,16);
    print("   FMODE="); print(fm_tbl[(fifo_ctrl >> 5) & 7u]);
    print("  FTH="); printDec(fifo_ctrl & 0x1Fu); printCr();

    // FIFO_SRC
    print("FIFO_SRC  [0x2F] = 0x"); dotnb(2,2,fifo_src,16);
    print("   WTM=");   printDec((fifo_src >> 7) & 1u);
    print("  OVRN=");   printDec((fifo_src >> 6) & 1u);
    print("  EMPTY=");  printDec((fifo_src >> 5) & 1u);
    print("  FSS=");    printDec( fifo_src        & 0x1Fu); printCr();

    // Live output register snapshot
    print("OUT_X: "); printDec(rx);
    print("  ("); printFloat(rx * LIS3DSH_SENS, 4); print(" g)");
    print("  OUT_Y: "); printDec(ry);
    print("  ("); printFloat(ry * LIS3DSH_SENS, 4); print(" g)");
    print("  OUT_Z: "); printDec(rz);
    print("  ("); printFloat(rz * LIS3DSH_SENS, 4); print(" g)"); printCr();
}
