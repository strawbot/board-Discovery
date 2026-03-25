/**
 * @file    adc_driver.c
 * @brief   Multi-channel ADC driver implementation – STM32F407, LL drivers.
 *
 * Hardware resources used
 * -----------------------
 *   ADC1                – single ADC, scan + continuous mode
 *   DMA2 Stream 0 Ch 0  – circular transfer, ADC1->DR → s_dma_buf[]
 *   GPIOA pin 3         – IN3
 *   GPIOB pins 0, 1     – IN8, IN9
 *   GPIOC pin 2         – IN12
 *   (internal channels need no GPIO)
 *
 * Regular sequence – DMA (external pins only)
 * --------------------------------------------
 *   Rank 1  IN3   (PA3)   56-cycle sample  (~2.7 µs @ 21 MHz ADC clk)
 *   Rank 2  IN8   (PB0)   56-cycle sample
 *   Rank 3  IN9   (PB1)   56-cycle sample
 *   Rank 4  IN12  (PC2)   56-cycle sample
 *
 * Injected sequence – polled on demand in ADC_Driver_Update()
 * ------------------------------------------------------------
 *   Pass A  TSVREFE=1, VBATE=0:
 *     Rank 1  TEMP    (CH16)  480-cycle sample  (~22.9 µs)
 *     Rank 2  VREFINT (CH17)  480-cycle sample
 *   Pass B  TSVREFE=0, VBATE=1:
 *     Rank 1  VBAT    (CH18)  480-cycle sample
 *
 *   RM0090 §13.3.3 (ADC_CCR) warns: "VBATE and TSVREFE bits cannot be set
 *   at the same time – when both are set, only the VBAT conversion is
 *   performed," meaning CH16 receives VBAT voltage instead of the die
 *   temperature.  The two-pass injected approach ensures these bits are
 *   never asserted simultaneously.
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

/**
 * Number of channels in the continuous DMA scan.
 * Only the four external pins are in the regular sequence; TEMP, VREFINT,
 * and VBAT are all read via the injected sequence so that TSVREFE and
 * VBATE are never asserted simultaneously.
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
 * Private helper – DMA2 Stream 0, Channel 0  (ADC1 → memory, circular)
 * ====================================================================== */

static void adc_dma_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

    /* Ensure stream is disabled before configuring */
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);
    while (LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_0)) {}

    LL_DMA_SetChannelSelection  (DMA2, LL_DMA_STREAM_0, LL_DMA_CHANNEL_0);
    LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetStreamPriorityLevel  (DMA2, LL_DMA_STREAM_0, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode                 (DMA2, LL_DMA_STREAM_0, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode        (DMA2, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode        (DMA2, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize           (DMA2, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_HALFWORD);
    LL_DMA_SetMemorySize           (DMA2, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_HALFWORD);
    /*
     * NOTE: STM32F4 LL uses LL_DMA_SetDataLength (not LL_DMA_SetNbDataToTransfer
     * which is the name used on G0/G4/H7/U5 families).
     */
    LL_DMA_SetDataLength           (DMA2, LL_DMA_STREAM_0, ADC_DMA_CHANNELS);
    LL_DMA_SetPeriphAddress        (DMA2, LL_DMA_STREAM_0, (uint32_t)&ADC1->DR);
    LL_DMA_SetMemoryAddress        (DMA2, LL_DMA_STREAM_0, (uint32_t)s_dma_buf);

    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
}

/* =========================================================================
 * Private helper – ADC1 core
 * ====================================================================== */

static void adc_core_init(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

    /* ----- Common (shared ADC registers) -------------------------------- */

    /* ADC clock: synchronous, derived from APB2 with chosen prescaler.    */
    LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1), ADC_PRESCALER);

    /*
     * Enable TSVREFE only – do NOT set VBATE here.
     * RM0090 §13.3.3: "When both VBATE and TSVREFE bits are set, only the
     * VBAT conversion is performed," meaning CH16 receives VBAT instead of
     * the temperature sensor signal.  VBATE is toggled only during the
     * injected VBAT measurement in ADC_Driver_Update().
     */
    LL_ADC_SetCommonPathInternalCh(
        __LL_ADC_COMMON_INSTANCE(ADC1),
        LL_ADC_PATH_INTERNAL_TEMPSENSOR |
        LL_ADC_PATH_INTERNAL_VREFINT);

    /* ----- ADC1 instance ------------------------------------------------ */

    LL_ADC_SetResolution    (ADC1, LL_ADC_RESOLUTION_12B);
    LL_ADC_SetDataAlignment (ADC1, LL_ADC_DATA_ALIGN_RIGHT);

    /* Scan mode: convert all ranks in the regular sequence.               */
    LL_ADC_SetSequencersScanMode(ADC1, LL_ADC_SEQ_SCAN_ENABLE);

    /* ----- Regular (foreground) sequence -------------------------------- */

    LL_ADC_REG_SetTriggerSource   (ADC1, LL_ADC_REG_TRIG_SOFTWARE);
    LL_ADC_REG_SetContinuousMode  (ADC1, LL_ADC_REG_CONV_CONTINUOUS);
    /* 4 ranks: external pins only.  Internal channels are injected. */
    LL_ADC_REG_SetSequencerLength (ADC1, LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS);

    /* DMA_UNLIMITED: the DMA request is re-issued after each sequence end,
     * keeping the circular DMA buffer perpetually refreshed.              */
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);

    /* ----- Regular channel assignment ----------------------------------- */

    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_3);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_8);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_9);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_12);

    /* ----- Sample times ------------------------------------------------- */

    /*
     * External channels: 56 cycles @ 21 MHz ≈ 2.67 µs.
     * Internal channels: 480 cycles @ 21 MHz ≈ 22.9 µs (≥ 10 µs required).
     * Internal sample times are set here once; ranks are assigned
     * dynamically inside read_internal_channels().
     */
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_3,          LL_ADC_SAMPLINGTIME_56CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_8,          LL_ADC_SAMPLINGTIME_56CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_9,          LL_ADC_SAMPLINGTIME_56CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_12,         LL_ADC_SAMPLINGTIME_56CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_SAMPLINGTIME_480CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT,    LL_ADC_SAMPLINGTIME_480CYCLES);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VBAT,       LL_ADC_SAMPLINGTIME_480CYCLES);

    /* ----- Injected sequence: software trigger, ranks set dynamically --- */

    LL_ADC_INJ_SetTriggerSource(ADC1, LL_ADC_INJ_TRIG_SOFTWARE);

    /* ----- Enable and start --------------------------------------------- */

    LL_ADC_Enable(ADC1);

    /*
     * Wait for ADC stabilisation.
     * RM0090 says the ADC needs a stabilisation time t_STAB before the
     * first conversion can be launched.  At 168 MHz, 1000 nops ≈ ~6 µs.
     */
    for (volatile uint32_t i = 0U; i < 1000U; i++) { __NOP(); }

    LL_ADC_REG_StartConversionSWStart(ADC1);
}

/* =========================================================================
 * Public API
 * ====================================================================== */

void ADC_Driver_Init(void)
{
    adc_gpio_init();
    adc_dma_init();
    adc_core_init();
}

/* =========================================================================
 * Private helper – injected reads for all three internal channels
 *
 * Pass A (TSVREFE=1, VBATE=0): 2-rank injected – TEMP then VREFINT.
 * Pass B (TSVREFE=0, VBATE=1): 1-rank injected – VBAT.
 *
 * RM0090 §13.3.3: VBATE and TSVREFE must never be set simultaneously.
 * The regular DMA scan (external pins only) is unaffected and continues
 * throughout.
 * ====================================================================== */

static void read_internal_channels(uint16_t *temp_raw,
                                   uint16_t *vref_raw,
                                   uint16_t *vbat_raw)
{
    /* ------------------------------------------------------------------
     * Pass A: TEMP + VREFINT
     * ---------------------------------------------------------------- */
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1),
                                   LL_ADC_PATH_INTERNAL_TEMPSENSOR |
                                   LL_ADC_PATH_INTERNAL_VREFINT);

    LL_ADC_INJ_SetSequencerLength(ADC1, LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS);
    LL_ADC_INJ_SetSequencerRanks (ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_TEMPSENSOR);
    LL_ADC_INJ_SetSequencerRanks (ADC1, LL_ADC_INJ_RANK_2, LL_ADC_CHANNEL_VREFINT);

    /* Allow internal switch to settle before triggering. */
    for (volatile uint32_t i = 0U; i < 200U; i++) { __NOP(); }

    LL_ADC_INJ_StartConversionSWStart(ADC1);
    while (!LL_ADC_IsActiveFlag_JEOS(ADC1)) {}

    *temp_raw = (uint16_t)LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
    *vref_raw = (uint16_t)LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2);
    LL_ADC_ClearFlag_JEOS(ADC1);

    /* ------------------------------------------------------------------
     * Pass B: VBAT
     * ---------------------------------------------------------------- */
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1),
                                   LL_ADC_PATH_INTERNAL_VBAT);

    LL_ADC_INJ_SetSequencerLength(ADC1, LL_ADC_INJ_SEQ_SCAN_DISABLE);   /* 1 rank */
    LL_ADC_INJ_SetSequencerRanks (ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VBAT);

    for (volatile uint32_t i = 0U; i < 200U; i++) { __NOP(); }

    LL_ADC_INJ_StartConversionSWStart(ADC1);
    while (!LL_ADC_IsActiveFlag_JEOS(ADC1)) {}

    *vbat_raw = (uint16_t)LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
    LL_ADC_ClearFlag_JEOS(ADC1);

    /* Restore TSVREFE so the temperature sensor stays powered between calls. */
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
     * Step 2: injected reads for TEMP, VREFINT, and VBAT.
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
     * Scale the raw reading to the 3.3 V equivalent before interpolating:
     *
     *   ts_scaled = raw_temp × VREFINT_CAL_VREF_MV / VDDA_mV
     *
     *   T(°C) = (CAL2_TEMP − CAL1_TEMP) × (ts_scaled − CAL1)
     *           ─────────────────────────────────────────────── + CAL1_TEMP
     *                        (CAL2 − CAL1)
     * ---------------------------------------------------------------- */
    const uint16_t ts_cal1 = *TEMPSENSOR_CAL1_ADDR;
    const uint16_t ts_cal2 = *TEMPSENSOR_CAL2_ADDR;

    const float ts_scaled =
        (float)results->raw[ADC_IDX_TEMP]
        * (float)VREFINT_CAL_VREF_MV / vdda_mv;

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
    print("--- ADC1 readings (STM32F407) ---");
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

