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

#ifdef __cplusplus
}
#endif

#endif /* ADC_DRIVER_H */