/**
 * @file    adc_driver.h
 * @brief   Multi-channel ADC driver for STM32F4Discovery (STM32F407)
 *
 * Reads 7 channels continuously via DMA in circular/scan mode on ADC1:
 *   IN3  (PA3), IN8 (PB0), IN9 (PB1), IN12 (PC2),
 *   Internal temperature sensor (CH16),
 *   VREFINT (CH17), VBAT/4 (CH18)
 *
 * Built on top of the ST LL (Low-Level) driver – no HAL required.
 *
 * -----------------------------------------------------------------------
 * CubeMX setup notes
 * -----------------------------------------------------------------------
 *  • Do NOT enable ADC1 or DMA2 in CubeMX; this driver owns both.
 *  • Do set your system/APB2 clocks in CubeMX as usual.
 *  • If printf-based output is desired, enable UART and retarget _write()
 *    (or use ITM / semihosting as preferred).
 * -----------------------------------------------------------------------
 */

#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------
 * Configuration
 * ---------------------------------------------------------------------- */

/** Total number of channels managed by this driver. */
#define ADC_NUM_CHANNELS        7U

/**
 * ADC clock prescaler.
 * APB2 is 84 MHz on the F407 at 168 MHz core.
 * /4  → 21 MHz  (recommended; fits 10 µs internal-channel requirement)
 * /8  → 10.5 MHz (more margin, lower throughput)
 */
#define ADC_PRESCALER           LL_ADC_CLOCK_SYNC_PCLK_DIV4

/**
 * DMA oversampling factor.
 *
 * The DMA circular buffer holds ADC_OVERSAMPLE complete 4-channel sweeps.
 * ADC_GetLastRaw() averages all copies of each channel before returning,
 * acting as a hardware-rate FIR box filter.
 *
 * At 21 MHz ADC clock with 56-cycle sample time:
 *   one 4-channel sweep ≈ 13 µs  →  scan rate ≈ 77 kHz
 *   ADC_OVERSAMPLE = 20  →  averaging window ≈ 261 µs
 *
 * 261 µs ≈ 1.04 periods of a 4 kHz interferer, giving near-complete
 * cancellation of the 4 kHz fundamental by coherent averaging.
 * Increase toward 32 for heavier filtering at the cost of RAM (64 bytes).
 */
#define ADC_OVERSAMPLE          20U

/* -------------------------------------------------------------------------
 * Channel index – matches DMA buffer order (rank 1 … rank 7)
 * ---------------------------------------------------------------------- */
typedef enum
{
    ADC_IDX_IN3     = 0,  /**< PA3  – ADC1_IN3                          */
    ADC_IDX_IN8     = 1,  /**< PB0  – ADC1_IN8                          */
    ADC_IDX_IN9     = 2,  /**< PB1  – ADC1_IN9                          */
    ADC_IDX_IN12    = 3,  /**< PC2  – ADC1_IN12                         */
    ADC_IDX_TEMP    = 4,  /**< Die temperature sensor – ADC1_IN16        */
    ADC_IDX_VREFINT = 5,  /**< Internal 1.21 V reference – ADC1_IN17    */
    ADC_IDX_VBAT    = 6,  /**< VBAT/4 bridge – ADC1_IN18                */
} ADC_ChannelIdx_t;

/* -------------------------------------------------------------------------
 * Result structure
 * ---------------------------------------------------------------------- */
typedef struct
{
    uint16_t raw[ADC_NUM_CHANNELS]; /**< Raw 12-bit ADC codes (0–4095)   */

    /**
     * Calibrated voltages in millivolts.
     *   [IN3]  .. [IN12]   – pin voltage 0–VDDA
     *   [TEMP]             – NOT a voltage; see temperature_c below
     *   [VREFINT]          – internal reference (~1210 mV, useful sanity check)
     *   [VBAT]             – actual VBAT in mV (bridge divider already removed)
     */
    float voltage_mv[ADC_NUM_CHANNELS];

    float vdda_mv;        /**< Calibrated VDDA derived from VREFINT (mV)  */
    float temperature_c;  /**< Die temperature (°C), factory-cal formula   */
    float vbat_v;         /**< Battery voltage (V) = voltage_mv[VBAT]/1000 */
} ADC_Results_t;

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

/**
 * @brief  Initialise GPIO pins, DMA2 Stream0, and ADC1 in scan+continuous
 *         mode.  Starts conversions immediately.
 *         Call once before the main loop (or after every low-power wakeup).
 */
void ADC_Driver_Init(void);

/**
 * @brief  Snapshot the DMA buffer and compute calibrated values.
 *
 * Safe to call from any context (briefly disables interrupts during copy).
 * Call at whatever rate you need; conversions run independently in hardware.
 *
 * @param[out] results  Populated with the latest readings.
 */
void ADC_Driver_Update(ADC_Results_t *results);

/**
 * @brief  Print a formatted table of all readings via printf().
 *
 * Requires printf to be redirected to a UART / ITM port.
 *
 * @param[in]  results  Previously filled by ADC_Driver_Update().
 */
void ADC_Driver_PrintAll(const ADC_Results_t *results);

/**
 * @brief  Read the two external muscle-wire channels directly from the DMA
 *         buffer without triggering any new conversion or blocking.
 *
 * Safe to call from ISR context: the implementation is two LDRH instructions
 * against the volatile s_dma_buf[] array — no locking, no processing.
 *
 * @param[out] in3_out  Raw 12-bit code for IN3 (PA3 — supply sense).
 * @param[out] in8_out  Raw 12-bit code for IN8 (PB0 — node sense).
 */
void ADC_GetLastRaw(uint16_t *in3_out, uint16_t *in8_out);

/**
 * @brief  Return the most recently calibrated VDDA in millivolts.
 *
 * Updated by each call to ADC_Driver_Update().  Defaults to 3300 mV until
 * a full update has been performed.  Used by lightweight callers that
 * obtain raw codes via ADC_GetLastRaw() and need to convert to voltages.
 *
 * Formula: voltage_mv = raw * ADC_GetVDDA_mv() / 4095.0f
 */
float ADC_GetVDDA_mv(void);

/* -------------------------------------------------------------------------
 * Dual regular simultaneous single-shot  (ADC1 master IN3, ADC2 slave IN8)
 *
 * Both channels are captured at exactly the same instant so that supply-
 * rail noise common to vsup and vnode cancels in the R_wire ratio:
 *
 *   R_wire = R_sense × (vsup − vnode) / vnode
 *          = R_sense × (k·V_dc + k·v_noise − k·V_dc − k·v_noise) / ...
 *                              ^^ noise cancels when sampled together ^^
 *
 * One simultaneous pair takes ~1.5 µs  (3 sample + 12 convert = 15 ADC
 * clock cycles at 21 MHz).  No DMA; results are read directly from CDR.
 *
 * Usage:
 *   ADC_SimInit();             // once, from MW_Init()
 *   ADC_SimTrigger();          // arm one conversion pair
 *   while (!ADC_SimReady()) {} // wait ~1.5 µs
 *   ADC_SimRead(&vsup, &vnode);
 *
 * Calling context:  ADC_SimTrigger / ADC_SimReady / ADC_SimRead may be
 * called from ISR or main-loop context.
 *
 * Conflict with ADC_Driver_Update():  ADC_Driver_Update() temporarily
 * points ADC1 rank-1 at VREFINT/TEMP/VBAT for its internal-channel reads.
 * ADC_SimTrigger() re-sets rank-1 to IN3 before each trigger, so there is
 * no persistent conflict.  Do not call ADC_SimTrigger while a Driver_Update
 * is blocking inside read_internal_channels().
 * ---------------------------------------------------------------------- */

/** @brief  Configure ADC1+ADC2 for dual regular simultaneous mode.
 *          Stops the ADC2 continuous DMA scan; call once from MW_Init(). */
void ADC_SimInit(void);

/** @brief  Arm one simultaneous conversion of IN3 (ADC1) and IN8 (ADC2).
 *          Re-sets ADC1 rank-1 to IN3 so a preceding Driver_Update cannot
 *          leave a stale channel assignment. */
void ADC_SimTrigger(void);

/** @brief  Return true when both ADC1 (master) and ADC2 (slave) EOC flags
 *          are set, indicating the simultaneous conversion has completed. */
bool ADC_SimReady(void);

/** @brief  Read results after ADC_SimReady() returns true.
 *
 *  Reads ADC1->DR (vsup) and ADC2->DR (vnode) directly.  Reading each DR
 *  also clears its EOC flag, readying the pair for the next ADC_SimTrigger.
 *
 *  @param[out] raw_vsup   12-bit ADC1 result for IN3 (PA3, supply sense).
 *  @param[out] raw_vnode  12-bit ADC2 result for IN8 (PB0, node sense). */
void ADC_SimRead(uint16_t *raw_vsup, uint16_t *raw_vnode);

#ifdef __cplusplus
}
#endif

#endif /* ADC_DRIVER_H */