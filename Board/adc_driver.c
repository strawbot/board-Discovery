/**
 * @file    adc_driver.c
 * @brief   Multi-channel ADC driver implementation – STM32F407, LL drivers.
 *
 * Hardware resources used
 * -----------------------
 *   ADC1                – internal channels only, single-shot on demand
 *   ADC2                – external pins, continuous scan + DMA
 *   DMA2 Stream 2 Ch 1  – circular transfer, ADC2->DR → s_dma_buf[]
 *   GPIOA pin 3         – IN3  (ADC12_IN3)
 *   GPIOB pins 0, 1     – IN8, IN9  (ADC12_IN8/9 – same pins on both ADCs)
 *   GPIOC pin 2         – IN12 (ADC12_IN12)
 *   (internal channels need no GPIO)
 *
 * ADC2 – regular sequence, continuous DMA
 * ----------------------------------------
 *   Rank 1  IN3   (PA3)   56-cycle sample  (~2.7 µs @ 21 MHz ADC clk)
 *   Rank 2  IN8   (PB0)   56-cycle sample
 *   Rank 3  IN9   (PB1)   56-cycle sample
 *   Rank 4  IN12  (PC2)   56-cycle sample
 *
 * ADC1 – single-shot reads, called from ADC_Driver_Update()
 * ----------------------------------------------------------
 *   Pass A  TSVREFE=1, VBATE=0:
 *     TEMP    (CH16)  480-cycle sample  (~22.9 µs)
 *     VREFINT (CH17)  480-cycle sample
 *   Pass B  TSVREFE=0, VBATE=1:
 *     VBAT    (CH18)  480-cycle sample
 *
 *   RM0090 §13.3.3 (ADC_CCR) warns: "VBATE and TSVREFE bits cannot be set
 *   at the same time – when both are set, only the VBAT conversion is
 *   performed."  With ADC1 dedicated to internal channels and ADC2 running
 *   external pins independently, toggling TSVREFE/VBATE never disturbs the
 *   external scan.  ADC1 also has no continuous mode and no DMA, making it
 *   straightforward to stop during low-power states.
 *
 * Calibration
 * -----------
 *   VDDA is derived at run-time from the factory VREFINT calibration word
 *   stored at 0x1FFF7A2A (measured at 3.3 V, 30 °C).
 *
 *   Temperature uses the two-point factory calibration:
 *     CAL1 (30 °C)  @ 0x1FFF7A2C
 *     CAL2 (110 °C) @ 0x1FFF7A2E
 *
 *   VBAT: the silicon connects VBAT through a /2 resistor bridge to IN18.
 */

#include "adc_driver.h"

#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_rcc.h"
#include "cmsis_compiler.h"   /* __disable_irq / __enable_irq              */

#include <string.h>           /* memcpy                                    */
#include "printers.h"         /* print / printDec / tabTo / printCr / maybeCr */

/* =========================================================================
 * Private constants
 * ====================================================================== */

/*
 * stm32f4xx_ll_adc.h already defines VREFINT_CAL_ADDR, TEMPSENSOR_CAL1_ADDR,
 * and TEMPSENSOR_CAL2_ADDR.  Guard our definitions so they are only provided
 * if an older or stripped-down LL header omits them.
 */
#ifndef VREFINT_CAL_ADDR
#define VREFINT_CAL_ADDR         ((const uint16_t *)0x1FFF7A2AU)
#endif

/** Voltage at which factory VREFINT calibration was performed (mV). */
#define VREFINT_CAL_VREF_MV      3300U

#ifndef TEMPSENSOR_CAL1_ADDR
#define TEMPSENSOR_CAL1_ADDR     ((const uint16_t *)0x1FFF7A2CU)  /* 30 °C  */
#endif
#ifndef TEMPSENSOR_CAL2_ADDR
#define TEMPSENSOR_CAL2_ADDR     ((const uint16_t *)0x1FFF7A2EU)  /* 110 °C */
#endif
#define TEMPSENSOR_CAL1_TEMP_C   30.0f
#define TEMPSENSOR_CAL2_TEMP_C   110.0f

/**
 * VBAT hardware bridge divider on STM32F405/407 (RM0090 §13).
 * The ADC sees VBAT/2; multiply the raw reading back by 2 to recover VBAT.
 */
#define VBAT_BRIDGE_DIVIDER      2U

/** ADC full-scale value for 12-bit resolution. */
#define ADC_FULL_SCALE           4095.0f

/*
 * ADC clock prescaler.
 *
 * STM32F407 @ 168 MHz: APB2 = 84 MHz.  ADC maximum clock = 36 MHz (RM0090
 * §13.3.1).  DIV4 → 21 MHz is the correct choice for a 168 MHz system.
 *
 * DIV2 → 42 MHz is over-spec: the ADC may appear to work but gives
 * inaccurate readings, particularly for the high-impedance temperature
 * sensor input which needs sufficient sample time to charge the S/H cap.
 *
 * If ADC_PRESCALER is already defined in a project-wide header this
 * fallback is skipped.  Verify it equals LL_ADC_CLOCK_SYNC_PCLK_DIV4.
 */
#ifndef ADC_PRESCALER
#define ADC_PRESCALER            LL_ADC_CLOCK_SYNC_PCLK_DIV4
#endif

/**
 * Number of channels in the ADC2 continuous DMA scan.
 * Only the four external pins feed ADC2; TEMP, VREFINT, and VBAT are read
 * on demand through ADC1 in two separate passes so that TSVREFE and VBATE
 * are never asserted simultaneously (RM0090 §13.3.3).
 */
#define ADC_DMA_CHANNELS         4U

/* =========================================================================
 * DMA result buffer  (volatile – written by DMA hardware)
 * ====================================================================== */

static volatile uint16_t s_dma_buf[ADC_DMA_CHANNELS];

/* =========================================================================
 * Private helper – GPIO
 * ====================================================================== */

static void adc_gpio_init(void)
{
    /* Enable GPIO clocks */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

    LL_GPIO_InitTypeDef gpio_cfg = {
        .Mode      = LL_GPIO_MODE_ANALOG,
        .Pull      = LL_GPIO_PULL_NO,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,  /* ignored in analog mode */
        .Speed      = LL_GPIO_SPEED_FREQ_LOW,
        .Alternate  = LL_GPIO_AF_0
    };

    /* PA3  → IN3 */
    gpio_cfg.Pin = LL_GPIO_PIN_3;
    LL_GPIO_Init(GPIOA, &gpio_cfg);

    /* PB0  → IN8 */
    gpio_cfg.Pin = LL_GPIO_PIN_0;
    LL_GPIO_Init(GPIOB, &gpio_cfg);

    /* PB1  → IN9 */
    gpio_cfg.Pin = LL_GPIO_PIN_1;
    LL_GPIO_Init(GPIOB, &gpio_cfg);

    /* PC2  → IN12 */
    gpio_cfg.Pin = LL_GPIO_PIN_2;
    LL_GPIO_Init(GPIOC, &gpio_cfg);
}

/* =========================================================================
 * Private helper – DMA2 Stream 2, Channel 1  (ADC2 → memory, circular)
 * ====================================================================== */

static void adc_dma_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

    /* Ensure stream is disabled before configuring */
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_2);
    while (LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_2)) {}

    LL_DMA_SetChannelSelection     (DMA2, LL_DMA_STREAM_2, LL_DMA_CHANNEL_1);
    LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetStreamPriorityLevel  (DMA2, LL_DMA_STREAM_2, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode                 (DMA2, LL_DMA_STREAM_2, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode        (DMA2, LL_DMA_STREAM_2, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode        (DMA2, LL_DMA_STREAM_2, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize           (DMA2, LL_DMA_STREAM_2, LL_DMA_PDATAALIGN_HALFWORD);
    LL_DMA_SetMemorySize           (DMA2, LL_DMA_STREAM_2, LL_DMA_MDATAALIGN_HALFWORD);
    LL_DMA_SetDataLength           (DMA2, LL_DMA_STREAM_2, ADC_DMA_CHANNELS);
    LL_DMA_SetPeriphAddress        (DMA2, LL_DMA_STREAM_2, (uint32_t)&ADC2->DR);
    LL_DMA_SetMemoryAddress        (DMA2, LL_DMA_STREAM_2, (uint32_t)s_dma_buf);

    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);
}

/* =========================================================================
 * Private helper – ADC1 (internal channels, single-shot, no DMA)
 * ====================================================================== */

static void adc1_init(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

    /* ----- Common (shared ADC clock) ------------------------------------ */

    /* ADC clock: synchronous, APB2 with chosen prescaler.
     * Must be configured before enabling either ADC instance.             */
    LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1), ADC_PRESCALER);

    /*
     * Enable TSVREFE only – do NOT set VBATE here.
     * RM0090 §13.3.3: "When both VBATE and TSVREFE bits are set, only the
     * VBAT conversion is performed," meaning CH16 receives VBAT instead of
     * the temperature sensor signal.  VBATE is toggled only during the
     * dedicated VBAT read in read_internal_channels().
     */
    LL_ADC_SetCommonPathInternalCh(
        __LL_ADC_COMMON_INSTANCE(ADC1),
        LL_ADC_PATH_INTERNAL_TEMPSENSOR |
        LL_ADC_PATH_INTERNAL_VREFINT);

    /* ----- ADC1 instance ------------------------------------------------ */

    LL_ADC_SetResolution    (ADC1, LL_ADC_RESOLUTION_12B);
    LL_ADC_SetDataAlignment (ADC1, LL_ADC_DATA_ALIGN_RIGHT);

    /* Single-rank mode: one channel at a time, selected per-call.         */
    LL_ADC_SetSequencersScanMode  (ADC1, LL_ADC_SEQ_SCAN_DISABLE);
    LL_ADC_REG_SetTriggerSource   (ADC1, LL_ADC_REG_TRIG_SOFTWARE);
    LL_ADC_REG_SetContinuousMode  (ADC1, LL_ADC_REG_CONV_SINGLE);
    LL_ADC_REG_SetSequencerLength (ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);

    /* No DMA – results are read directly from DR after each conversion.   */
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);

    /* ----- Sample times for internal channels --------------------------- */

    /* 480 cycles @ 21 MHz ≈ 22.9 µs  (≥ 10 µs required by datasheet).   */
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_SAMPLINGTIME_480CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT,    LL_ADC_SAMPLINGTIME_480CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VBAT,       LL_ADC_SAMPLINGTIME_480CYCLES);

    /* ----- Enable ------------------------------------------------------- */

    LL_ADC_Enable(ADC1);

    /* t_STAB stabilisation delay before first conversion.
     * At 168 MHz core, 1000 NOPs ≈ 6 µs.                                 */
    for (volatile uint32_t i = 0U; i < 1000U; i++) { __NOP(); }

    /* No continuous start – conversions are triggered on demand only.     */
}

/* =========================================================================
 * Private helper – ADC2 (external pins, continuous scan + DMA)
 * ====================================================================== */

static void adc2_init(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC2);

    /* Common clock was already programmed in adc1_init().                 */

    /* ----- ADC2 instance ------------------------------------------------ */

    LL_ADC_SetResolution    (ADC2, LL_ADC_RESOLUTION_12B);
    LL_ADC_SetDataAlignment (ADC2, LL_ADC_DATA_ALIGN_RIGHT);

    /* Scan mode: walk all four ranks in sequence on each trigger.         */
    LL_ADC_SetSequencersScanMode  (ADC2, LL_ADC_SEQ_SCAN_ENABLE);
    LL_ADC_REG_SetTriggerSource   (ADC2, LL_ADC_REG_TRIG_SOFTWARE);
    LL_ADC_REG_SetContinuousMode  (ADC2, LL_ADC_REG_CONV_CONTINUOUS);
    LL_ADC_REG_SetSequencerLength (ADC2, LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS);

    /* DMA_UNLIMITED: DMA request is re-issued after every sequence end,
     * keeping the circular s_dma_buf[] perpetually refreshed.             */
    LL_ADC_REG_SetDMATransfer(ADC2, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);

    /* ----- External channel assignment – ranks 1–4 ---------------------- */

    LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_3);
    LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_8);
    LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_9);
    LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_12);

    /* ----- Sample times ------------------------------------------------- */

    /* 56 cycles @ 21 MHz ≈ 2.67 µs per channel.                          */
    LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_3,  LL_ADC_SAMPLINGTIME_56CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_8,  LL_ADC_SAMPLINGTIME_56CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_9,  LL_ADC_SAMPLINGTIME_56CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_12, LL_ADC_SAMPLINGTIME_56CYCLES);

    /* ----- Enable and start continuous scan ----------------------------- */

    LL_ADC_Enable(ADC2);

    for (volatile uint32_t i = 0U; i < 1000U; i++) { __NOP(); }

    LL_ADC_REG_StartConversionSWStart(ADC2);
}

/* =========================================================================
 * Public API
 * ====================================================================== */

void ADC_Driver_Init(void)
{
    adc_gpio_init();   /* PA3, PB0, PB1, PC2 → analog mode              */
    adc_dma_init();    /* DMA2 Stream2 Ch1 → ADC2->DR, circular          */
    adc1_init();       /* ADC1: internal channels, single-shot, no DMA   */
    adc2_init();       /* ADC2: external pins, continuous scan + DMA      */
}

/* =========================================================================
 * Private helper – single-shot regular conversion on ADC1
 *
 * Sets rank-1 to the requested channel, fires a software trigger, waits
 * for EOC, and returns the 12-bit result.  ADC1 has no continuous mode
 * and no DMA, so there is no ongoing conversion to interrupt.
 * ====================================================================== */

static uint16_t adc1_read_channel(uint32_t channel)
{
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, channel);

    /*
     * Clear any stale EOC/EOCS flag before triggering.  If a previous
     * conversion left the flag set (e.g. DR was read by the compiler but
     * the flag persisted due to an optimisation artefact), the poll below
     * would return immediately and read old data.
     *
     * STM32F4 LL naming: the EOC status-register bit is exposed as
     * LL_ADC_IsActiveFlag_EOCS / LL_ADC_ClearFlag_EOCS (the "EOCS" suffix
     * reflects that the bit's meaning is controlled by the EOCS bit in CR2).
     */
    LL_ADC_ClearFlag_EOCS(ADC1);

    LL_ADC_REG_StartConversionSWStart(ADC1);
    while (!LL_ADC_IsActiveFlag_EOCS(ADC1)) {}
    const uint16_t raw = (uint16_t)LL_ADC_REG_ReadConversionData12(ADC1);
    LL_ADC_ClearFlag_EOCS(ADC1);
    return raw;
}

/* =========================================================================
 * Private helper – on-demand reads for all three internal channels
 *
 * Pass A (TSVREFE=1, VBATE=0): TEMP then VREFINT.
 * Pass B (TSVREFE=0, VBATE=1): VBAT.
 *
 * RM0090 §13.3.3: VBATE and TSVREFE must never be set simultaneously –
 * when both are set, only the VBAT conversion is performed (CH16 receives
 * the VBAT signal instead of the temperature sensor).  The two-pass
 * sequence guarantees they are never asserted at the same time.
 *
 * ADC2's continuous DMA scan runs unaffected throughout; the CCR bits
 * only gate internal channel muxes and have no effect on ADC2 channels.
 * ====================================================================== */

static void read_internal_channels(uint16_t *temp_raw,
                                   uint16_t *vref_raw,
                                   uint16_t *vbat_raw)
{
    /* ------------------------------------------------------------------
     * Pass A: TSVREFE=1, VBATE=0  →  TEMP + VREFINT
     * ---------------------------------------------------------------- */
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1),
                                   LL_ADC_PATH_INTERNAL_TEMPSENSOR |
                                   LL_ADC_PATH_INTERNAL_VREFINT);

    /* Allow the internal mux switch to settle (~200 NOPs ≈ 1.2 µs).    */
    for (volatile uint32_t i = 0U; i < 200U; i++) { __NOP(); }

    *temp_raw = adc1_read_channel(LL_ADC_CHANNEL_TEMPSENSOR);
    *vref_raw = adc1_read_channel(LL_ADC_CHANNEL_VREFINT);

    /* ------------------------------------------------------------------
     * Pass B: TSVREFE=0, VBATE=1  →  VBAT
     * ---------------------------------------------------------------- */
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1),
                                   LL_ADC_PATH_INTERNAL_VBAT);

    for (volatile uint32_t i = 0U; i < 200U; i++) { __NOP(); }

    *vbat_raw = adc1_read_channel(LL_ADC_CHANNEL_VBAT);

    /* Restore TSVREFE so the sensor stays powered between calls.        */
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1),
                                   LL_ADC_PATH_INTERNAL_TEMPSENSOR |
                                   LL_ADC_PATH_INTERNAL_VREFINT);
}

/* ------------------------------------------------------------------------- */

void ADC_Driver_Update(ADC_Results_t *results)
{
    /* ------------------------------------------------------------------
     * Step 1: snapshot the 4-slot DMA buffer (external pins only).
     * ---------------------------------------------------------------- */
    uint16_t snap[ADC_DMA_CHANNELS];

    __disable_irq();
    memcpy(snap, (const void *)s_dma_buf, sizeof(snap));
    __enable_irq();

    for (uint32_t i = 0U; i < ADC_DMA_CHANNELS; i++)
    {
        results->raw[i] = snap[i];
    }

    /* ------------------------------------------------------------------
     * Step 2: single-shot reads on ADC1 for TEMP, VREFINT, and VBAT.
     * ---------------------------------------------------------------- */
    read_internal_channels(&results->raw[ADC_IDX_TEMP],
                           &results->raw[ADC_IDX_VREFINT],
                           &results->raw[ADC_IDX_VBAT]);

    /* ------------------------------------------------------------------
     * Step 3: calibrate VDDA from the VREFINT factory cal word.
     *
     *   VDDA_mV = VREFINT_CAL_VREF_MV × VREFINT_CAL / VREFINT_raw
     * ---------------------------------------------------------------- */
    const uint16_t vrefint_cal = *VREFINT_CAL_ADDR;
    const float vdda_mv =
        (float)VREFINT_CAL_VREF_MV * (float)vrefint_cal
        / (float)results->raw[ADC_IDX_VREFINT];

    results->vdda_mv = vdda_mv;

    /* ------------------------------------------------------------------
     * Step 4: external channel voltages.
     *
     *   V_pin_mV = raw × VDDA_mV / 4095
     * ---------------------------------------------------------------- */
    for (int i = ADC_IDX_IN3; i <= ADC_IDX_IN12; i++)
    {
        results->voltage_mv[i] = (float)results->raw[i] * vdda_mv / ADC_FULL_SCALE;
    }

    /* ------------------------------------------------------------------
     * Step 5: VREFINT voltage (sanity check – should be ~1210 mV).
     * ---------------------------------------------------------------- */
    results->voltage_mv[ADC_IDX_VREFINT] =
        (float)results->raw[ADC_IDX_VREFINT] * vdda_mv / ADC_FULL_SCALE;

    /* ------------------------------------------------------------------
     * Step 6: die temperature – two-point factory calibration.
     *
     * The factory cal words (CAL1 @ 30 °C, CAL2 @ 110 °C) were sampled at
     * exactly VDDA = 3300 mV.  The temperature sensor output is an absolute
     * voltage (not ratiometric to VDDA): for the same die temperature, a
     * lower VDDA raises the raw ADC count because the ADC full-scale drops.
     *
     * Normalise raw_temp back to the 3300 mV reference so it is directly
     * comparable to CAL1/CAL2:
     *
     *   ts_scaled = raw_temp × VDDA_mV / VREFINT_CAL_VREF_MV
     *
     * Note the direction: multiply by VDDA/3300 (not 3300/VDDA).
     * Multiplying the wrong way compounds the error instead of cancelling
     * it, and produces wildly high temperatures when VDDA < 3300 mV.
     *
     *   T(°C) = (CAL2_TEMP − CAL1_TEMP) × (ts_scaled − CAL1)
     *           ─────────────────────────────────────────────── + CAL1_TEMP
     *                        (CAL2 − CAL1)
     * ---------------------------------------------------------------- */
    const uint16_t ts_cal1 = *TEMPSENSOR_CAL1_ADDR;
    const uint16_t ts_cal2 = *TEMPSENSOR_CAL2_ADDR;

    const float ts_scaled =
        (float)results->raw[ADC_IDX_TEMP]
        * vdda_mv / (float)VREFINT_CAL_VREF_MV;

    results->temperature_c =
        (TEMPSENSOR_CAL2_TEMP_C - TEMPSENSOR_CAL1_TEMP_C)
        * (ts_scaled - (float)ts_cal1)
        / (float)(ts_cal2 - ts_cal1)
        + TEMPSENSOR_CAL1_TEMP_C;

    results->voltage_mv[ADC_IDX_TEMP] = 0.0f;   /* not a real voltage */

    /* ------------------------------------------------------------------
     * Step 7: VBAT – hardware divides by 2 before the ADC.
     *
     *   VBAT_mV = raw_vbat × VDDA_mV / 4095 × VBAT_BRIDGE_DIVIDER (×2)
     * ---------------------------------------------------------------- */
    const float vbat_mv =
        (float)results->raw[ADC_IDX_VBAT]
        * vdda_mv / ADC_FULL_SCALE
        * (float)VBAT_BRIDGE_DIVIDER;

    results->voltage_mv[ADC_IDX_VBAT] = vbat_mv;
    results->vbat_v = vbat_mv / 1000.0f;
}

/* =========================================================================
 * ADC_Driver_PrintAll  (uses printers.h API)
 * ====================================================================== */

/*
 * Column layout:
 *   Col  0 : channel label
 *   Col 12 : raw 12-bit code
 *   Col 20 : converted value + unit
 */
#define COL_RAW    12
#define COL_VALUE  20

/* Print one row: label | raw code | float value | unit string. */
static void print_row(const char *label,
                      uint32_t    raw,
                      float       value,
                      uint8_t     decimals,
                      const char *unit)
{
    maybeCr();
    print(label);   tabTo(COL_RAW);
    printDec(raw);  tabTo(COL_VALUE);
    printFloat(value, decimals);
    print(unit);
    printCr();
}

/* ------------------------------------------------------------------------- */

void ADC_Driver_PrintAll(const ADC_Results_t *results)
{
    maybeCr();
    print("--- ADC readings (STM32F407) ---");
    printCr();

    /* Header */
    print("Channel");  tabTo(COL_RAW);
    print("Raw");      tabTo(COL_VALUE);
    print("Value");
    printCr();

    /* VDDA – derived from VREFINT, no raw code of its own */
    maybeCr();
    print("VDDA");     tabTo(COL_RAW);
    print("----");     tabTo(COL_VALUE);
    printFloat(results->vdda_mv, 2);
    print(" mV (cal)");
    printCr();

    /* External channels – 2 decimal places in mV */
    print_row("IN3  PA3",  results->raw[ADC_IDX_IN3],     results->voltage_mv[ADC_IDX_IN3],     2, " mV");
    print_row("IN8  PB0",  results->raw[ADC_IDX_IN8],     results->voltage_mv[ADC_IDX_IN8],     2, " mV");
    print_row("IN9  PB1",  results->raw[ADC_IDX_IN9],     results->voltage_mv[ADC_IDX_IN9],     2, " mV");
    print_row("IN12 PC2",  results->raw[ADC_IDX_IN12],    results->voltage_mv[ADC_IDX_IN12],    2, " mV");

    /* Internal channels */
    print_row("TEMP",      results->raw[ADC_IDX_TEMP],    results->temperature_c,               1, " C (die)");
    print_row("VREFINT",   results->raw[ADC_IDX_VREFINT], results->voltage_mv[ADC_IDX_VREFINT], 2, " mV");
    print_row("VBAT",      results->raw[ADC_IDX_VBAT],    results->vbat_v,                      3, " V (x2)");
}

void show_adc() {
    ADC_Results_t adc;
    ADC_Driver_Update(&adc);
    ADC_Driver_PrintAll(&adc);
}

