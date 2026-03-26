/**
 * @file    dac_sine.c
 * @brief   DAC sine wave generator – bare-metal implementation
 *          Target: STM32F407VGT6 (STM32F4DISCOVERY)
 *
 * Signal chain (CPU-free after init)
 * ───────────────────────────────────
 *   TIMx Update event
 *       │  TRGO pulse
 *       ▼
 *   DAC Channel 1  (PA4, DHR12R1 – 12-bit right-aligned)
 *       │  DMA request on each conversion trigger
 *       ▼
 *   DMA1 Stream 5 Channel 7  (circular, memory→peripheral, 16-bit)
 *       │
 *       └──► sine_buf[]  (n_samples × uint16_t, pre-computed each frequency change)
 *
 * Timer selection
 * ───────────────
 *   Define DAC_SINE_TIMER as 6, 7, or 4 (default: 6).
 *   All three timers live on APB1 → timer clock = 84 MHz.
 *
 *   Timer  TSEL[2:0]   RCC bit            MMS reg
 *   ─────  ─────────   ───────────────    ──────────────
 *   TIM6     000       RCC_APB1ENR_TIM6EN  TIM6->CR2
 *   TIM7     010       RCC_APB1ENR_TIM7EN  TIM7->CR2
 *   TIM4     101       RCC_APB1ENR_TIM4EN  TIM4->CR2
 *
 * Frequency / PSC / ARR equations
 * ────────────────────────────────
 *   We want:  TIMx update rate  =  freq_hz × n_samples
 *             TIM_CLK / ((PSC+1)(ARR+1))  =  freq_hz × n_samples
 *
 *   total_ticks = TIM_CLK / (freq_hz × n_samples)
 *   PSC  = ceil(total_ticks / 65536) - 1        (keep ARR ≤ 65535)
 *   ARR  = total_ticks / (PSC+1)  - 1
 *
 *   Example – 1 Hz, 256 samples:
 *     total_ticks = 84 000 000 / 256 = 328 125
 *     PSC = ceil(328125/65536)-1 = 6-1 = 5
 *     ARR = 328125/6 - 1 = 54686    (actual rate = 256.002 Hz, error < 0.001%)
 *
 *   Example – 100 kHz, 10 samples:
 *     total_ticks = 84 000 000 / 1 000 000 = 84
 *     PSC = 0
 *     ARR = 83                       (exact)
 */

#include "dac_sine.h"
#include "stm32f4xx.h"
#include "printers.h"
#include "cli.h"
#include <math.h>

/* =========================================================================
 * Timer selection
 * ========================================================================= */

#ifndef DAC_SINE_TIMER
#  define DAC_SINE_TIMER  6   /* default: TIM6 */
#endif

#if   DAC_SINE_TIMER == 6
#  define DAC_TIMx           TIM6
#  define DAC_TIMx_RCC_EN    RCC_APB1ENR_TIM6EN
#  define DAC_TSEL           0U   /* TSEL[2:0] = 000 → TIM6_TRGO */
#  define DAC_TIMER_NAME     "TIM6  (TSEL=000)"

#elif DAC_SINE_TIMER == 7
#  define DAC_TIMx           TIM7
#  define DAC_TIMx_RCC_EN    RCC_APB1ENR_TIM7EN
#  define DAC_TSEL           2U   /* TSEL[2:0] = 010 → TIM7_TRGO */
#  define DAC_TIMER_NAME     "TIM7  (TSEL=010)"

#elif DAC_SINE_TIMER == 4
#  define DAC_TIMx           TIM4
#  define DAC_TIMx_RCC_EN    RCC_APB1ENR_TIM4EN
#  define DAC_TSEL           5U   /* TSEL[2:0] = 101 → TIM4_TRGO */
#  define DAC_TIMER_NAME     "TIM4  (TSEL=101)"

#else
#  error "DAC_SINE_TIMER must be 4, 6, or 7"
#endif

/* =========================================================================
 * Constants
 * ========================================================================= */

/** APB1 timer clock – 84 MHz with standard 168 MHz system clock */
#define TIM_CLK          84000000UL

/** Max DAC update rate: ~1 MHz (limited by output buffer settling ~1 µs) */
#define DAC_MAX_RATE     1000000UL

#define MAX_SAMPLES      256U   /* sine table size upper bound  */
#define MIN_SAMPLES       10U   /* minimum samples/period (100 kHz @ 1 MHz) */

/* =========================================================================
 * Module state
 * ========================================================================= */

static uint16_t  sine_buf[MAX_SAMPLES];   /* DMA source: one sine period   */
static uint32_t  g_n_samples;             /* samples currently in use       */
static uint32_t  g_psc;                   /* saved PSC (for Start/Stop)     */
static uint32_t  g_arr;                   /* saved ARR                      */

/* =========================================================================
 * Private helpers
 * ========================================================================= */

/** Fill sine_buf[0..n-1] with a 12-bit (0–4095) full-scale sine wave. */
static void sine_fill(uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        float v = sinf(2.0f * 3.14159265358979f * (float)i / (float)n);
        sine_buf[i] = (uint16_t)((v + 1.0f) * 0.5f * 4095.0f + 0.5f);
    }
}

/**
 * Compute n_samples, PSC, and ARR for a given output frequency.
 *
 * Strategy:
 *   1. Maximise n_samples (better waveform), subject to DAC rate ≤ 1 MHz.
 *   2. Compute total_ticks = TIM_CLK / (freq * n).
 *   3. Split into PSC + ARR so both fit in 16 bits.
 */
static void calc_params(uint32_t freq_hz,
                        uint32_t *p_n,
                        uint32_t *p_psc,
                        uint32_t *p_arr)
{
    /* ── Step 1: number of samples ── */
    uint32_t n = DAC_MAX_RATE / freq_hz;
    if (n > MAX_SAMPLES) n = MAX_SAMPLES;
    if (n < MIN_SAMPLES) n = MIN_SAMPLES;

    /* ── Step 2: total timer ticks for one DAC update ── */
    uint32_t total = TIM_CLK / (freq_hz * n);
    if (total < 1U) total = 1U;

    /* ── Step 3: PSC/ARR split for 16-bit timer ──
     *   We need (PSC+1)*(ARR+1) ≈ total, with ARR ≤ 65535.
     *   Minimum PSC such that  total/(PSC+1) ≤ 65536:
     *     PSC+1 = ceil(total / 65536)
     */
    uint32_t psc_plus1 = (total + 65535U) / 65536U;   /* = ceil(total/65536) */
    if (psc_plus1 < 1U) psc_plus1 = 1U;

    uint32_t arr_plus1 = total / psc_plus1;
    if (arr_plus1 < 1U) arr_plus1 = 1U;

    *p_n   = n;
    *p_psc = psc_plus1 - 1U;   /* register value = divisor - 1 */
    *p_arr = arr_plus1 - 1U;
}

/* ─── Peripheral initialisers ─────────────────────────────────────────────── */

/** PA4 → analogue mode (DAC1_OUT; no alternate function required). */
static void gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    __DSB();
    GPIOA->MODER |= (3U << (4U * 2U));   /* PA4: MODER[9:8] = 11 (analogue) */
}

/**
 * DMA1 Stream 5 Channel 7 – memory→peripheral, 16-bit, circular.
 *
 * Peripheral: DAC->DHR12R1  (fixed address)
 * Memory:     sine_buf[]    (incrementing, wraps at n_samples)
 *
 * DMA1 Stream 5 Channel 7 → DAC Channel 1  (RM0090 Table 42)
 */
static void dma_init(uint32_t n_samples)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    __DSB();

    /* Disable stream; hardware clears EN once the ongoing transfer ends */
    DMA1_Stream5->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream5->CR & DMA_SxCR_EN) {}

    /* Clear all Stream-5 interrupt flags (HISR bits: TC, HT, TE, DME, FE) */
    DMA1->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 |
                  DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5 |
                  DMA_HIFCR_CFEIF5;

    DMA1_Stream5->PAR  = (uint32_t)(&DAC->DHR12R1);
    DMA1_Stream5->M0AR = (uint32_t)sine_buf;
    DMA1_Stream5->NDTR = n_samples;
    DMA1_Stream5->FCR  = 0U;   /* direct mode – no FIFO */

    DMA1_Stream5->CR =
          (7U << DMA_SxCR_CHSEL_Pos)   /* Channel 7                       */
        | (2U << DMA_SxCR_PL_Pos)      /* Priority: very high             */
        | (1U << DMA_SxCR_MSIZE_Pos)   /* Memory data size:     16-bit    */
        | (1U << DMA_SxCR_PSIZE_Pos)   /* Peripheral data size: 16-bit    */
        | DMA_SxCR_MINC                /* Memory address increments       */
        | DMA_SxCR_CIRC                /* Circular – NDTR auto-reloads    */
        | (1U << DMA_SxCR_DIR_Pos);    /* Memory → Peripheral             */

    DMA1_Stream5->CR |= DMA_SxCR_EN;
}

/**
 * DAC Channel 1 – hardware trigger from TIMx_TRGO, DMA enabled.
 *
 * TSEL[2:0] selects the trigger source (see DAC_TSEL macro above).
 * Output buffer (BOFF1=0) is ON for low-impedance drive; disable it
 * (set DAC_CR_BOFF1) only if you need the full 0–3.3 V rail and your
 * load is > 50 kΩ / < 50 pF.
 */
static void dac_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;
    __DSB();

    DAC->CR = DAC_CR_EN1
            | DAC_CR_TEN1
            | (DAC_TSEL << DAC_CR_TSEL1_Pos)
            | DAC_CR_DMAEN1;
}

/**
 * Configure and start the selected 16-bit timer (TIM6, TIM7, or TIM4).
 * MMS = 010: Update event → TRGO → triggers DAC conversion.
 *
 * @param psc  Prescaler register value   (divides clock by psc+1)
 * @param arr  Auto-reload register value (period = arr+1 prescaled ticks)
 */
static void timer_init(uint32_t psc, uint32_t arr)
{
    RCC->APB1ENR |= DAC_TIMx_RCC_EN;
    __DSB();

    DAC_TIMx->CR1 = 0U;
    DAC_TIMx->CR2 = (2U << TIM_CR2_MMS_Pos);   /* MMS = 010: Update → TRGO */
    DAC_TIMx->PSC = (uint16_t)psc;
    DAC_TIMx->ARR = (uint16_t)arr;
    DAC_TIMx->EGR = TIM_EGR_UG;                /* load PSC/ARR now         */
    DAC_TIMx->SR  = 0U;                         /* clear update flag        */
    DAC_TIMx->CR1 = TIM_CR1_CEN;               /* start                    */
}

/* =========================================================================
 * Public API
 * ========================================================================= */

void DAC_Sine_Init(uint32_t freq_hz)
{
    if (freq_hz < 1U)      freq_hz = 1U;
    if (freq_hz > 100000U) freq_hz = 100000U;

    calc_params(freq_hz, &g_n_samples, &g_psc, &g_arr);
    sine_fill(g_n_samples);

    gpio_init();
    dma_init(g_n_samples);
    dac_init();
    timer_init(g_psc, g_arr);
}

void DAC_Sine_SetFreq(uint32_t freq_hz)
{
    if (freq_hz < 1U)      freq_hz = 1U;
    if (freq_hz > 100000U) freq_hz = 100000U;

    uint32_t n, psc, arr;
    calc_params(freq_hz, &n, &psc, &arr);

    /* ── 1. Stop timer (no more TRGO pulses → no new DAC triggers) ── */
    DAC_TIMx->CR1 &= ~TIM_CR1_CEN;

    /* ── 2. Stop DMA ── */
    DMA1_Stream5->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream5->CR & DMA_SxCR_EN) {}

    /* ── 3. Rebuild sine table (DMA is off – safe to write) ── */
    sine_fill(n);

    /* ── 4. Reconfigure DMA with new sample count and re-enable ── */
    DMA1->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 |
                  DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5 |
                  DMA_HIFCR_CFEIF5;
    DMA1_Stream5->NDTR = n;
    DMA1_Stream5->CR  |= DMA_SxCR_EN;

    /* ── 5. Load new timer period and restart ── */
    DAC_TIMx->PSC = (uint16_t)psc;
    DAC_TIMx->ARR = (uint16_t)arr;
    DAC_TIMx->EGR = TIM_EGR_UG;
    DAC_TIMx->SR  = 0U;
    DAC_TIMx->CR1 = TIM_CR1_CEN;

    g_n_samples = n;
    g_psc       = psc;
    g_arr       = arr;
}

void cli_DAC_Sine_SetFreq() { DAC_Sine_SetFreq(ret()); }

void DAC_Sine_Stop(void)
{
    DAC_TIMx->CR1 &= ~TIM_CR1_CEN;

    DMA1_Stream5->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream5->CR & DMA_SxCR_EN) {}

    /* Drive output to 0 V via software trigger */
    DAC->CR &= ~(DAC_CR_TEN1 | DAC_CR_DMAEN1);
    DAC->DHR12R1  = 0U;
    DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
}

void DAC_Sine_Start(void)
{
    /* Restore DAC to hardware-trigger + DMA mode */
    DAC->CR |= DAC_CR_TEN1
             | (DAC_TSEL << DAC_CR_TSEL1_Pos)
             | DAC_CR_DMAEN1;

    /* Restart DMA */
    DMA1->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 |
                  DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5 |
                  DMA_HIFCR_CFEIF5;
    DMA1_Stream5->NDTR = g_n_samples;
    DMA1_Stream5->CR  |= DMA_SxCR_EN;

    /* Restart timer */
    DAC_TIMx->PSC = (uint16_t)g_psc;
    DAC_TIMx->ARR = (uint16_t)g_arr;
    DAC_TIMx->EGR = TIM_EGR_UG;
    DAC_TIMx->SR  = 0U;
    DAC_TIMx->CR1 = TIM_CR1_CEN;
}

/* =========================================================================
 * CLI command: show-dac
 * Modelled after show_adc() in adc_driver.c.
 * ========================================================================= */

#define COL_VAL  12   /* column at which values start */

void show_dac(void)
{
    uint32_t dac_rate = TIM_CLK / ((g_psc + 1U) * (g_arr + 1U));
    float    freq     = (float)dac_rate / (float)g_n_samples;

    maybeCr();
    print("--- DAC sine wave (STM32F407) ---");
    printCr();

    maybeCr();
    print("Output");    tabTo(COL_VAL);  print("PA4  (DAC1_OUT, 12-bit)");   printCr();

    maybeCr();
    print("Timer");     tabTo(COL_VAL);  print(DAC_TIMER_NAME);              printCr();

    maybeCr();
    print("DMA");       tabTo(COL_VAL);  print("DMA1 Stream5 Ch7");          printCr();

    maybeCr();
    print("Samples");   tabTo(COL_VAL);  printDec(g_n_samples);              printCr();

    maybeCr();
    print("PSC");       tabTo(COL_VAL);  printDec(g_psc);                    printCr();

    maybeCr();
    print("ARR");       tabTo(COL_VAL);  printDec(g_arr);                    printCr();

    maybeCr();
    print("DAC rate");  tabTo(COL_VAL);  printDec(dac_rate);  print(" Hz");  printCr();

    maybeCr();
    print("Frequency"); tabTo(COL_VAL);  printFloat(freq, 3); print(" Hz");  printCr();
}
