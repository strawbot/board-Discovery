// muscle_wire.c — Nitinol muscle wire PWM driver + resistance monitor
//
// PWM  : TIM3 CH2, PB5 (AF2), 1 kHz, 0.1 % duty resolution
// ADC  : dual simultaneous ADC1+ADC2 single-shot, triggered in the ON phase
//        IN3 (PA3) = supply sense divider
//        IN8 (PB0) = inline sense resistor (FET source → 1 Ω → GND)
// Print: printers.h API (print / printFloat / printDec / printCr / tabTo)
// CLI  : void/void functions; ret() fetches the data-stack argument

#include "muscle_wire.h"

#include <string.h>
#include <math.h>

#include "stm32f4xx.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_tim.h"

#include "adc_driver.h"         // ADC_Driver_Update / ADC_Results_t / ADC_IDX_*
#include "printers.h"           // print / printFloat / printDec / printCr / tabTo
#include "cli.h"                // ret()
#include "tea.h"                // after() / msec() / namedAction()
#include "http_server.h"        // http_mw_live_feed()

// ── Column positions for tabular output ──────────────────────────────────────
#define COL_LABEL   0
#define COL_VALUE   14
#define COL_UNIT    26

// ── Module state ──────────────────────────────────────────────────────────────
static uint8_t             pwm_pct       = 0;

static MW_Sample_t         profile[MW_PROFILE_LEN];
static uint32_t            profile_idx   = 0;
static uint32_t            profile_tick  = 0;
static uint32_t            profile_start = 0;   // HAL_GetTick() at start
static uint8_t             profile_duty  = 0;
static MW_ProfileState_t   profile_state = MW_PROFILE_IDLE;

// ── ADC sample latches ────────────────────────────────────────────────────────
//
// adc_raw_latch — written by the CC4 ISR (trivially small: two LDRH + STRH).
//   Holds raw 12-bit counts captured MW_ADC_SETTLE_TICKS µs into the ON phase.
//   ISR only writes; MW_SampleR (main loop) reads.
//
// adc_latch — written by MW_SampleR in main-loop context after converting the
//   raw counts to millivolts using the cached VDDA.  All other code reads this.
//   Always fresh within one MW_SampleR period (100 ms).
static volatile struct {
    uint16_t raw0;      // IN3 — supply sense
    uint16_t raw1;      // IN8 — inline sense resistor
} adc_raw_latch = { 0u, 0u };

static volatile struct {
    float vsup_mv;
    float vsense_mv;
} adc_latch = { 0.0f, 0.0f };

// IIR filter state — initialised to 0; MW_SampleR seeds from first reading.
static float iir_vsup_mv   = 0.0f;
static float iir_vsense_mv = 0.0f;

// ── Closed-loop controller state ──────────────────────────────────────────────
static float cl_target_pct = -1.0f;    // < 0 = disabled
static float cl_integral   =  0.0f;

// ── Calibration state ─────────────────────────────────────────────────────────
static MW_CalState_t  cal_state   = MW_CAL_IDLE;
static float          cal_r_max   = MW_R_WIRE_RELAXED;     // updated by cal-wire
static float          cal_r_min   = MW_R_WIRE_CONTRACTED;  // updated by cal-wire
static bool           cal_valid   = false;                  // true once cal-wire succeeds
static uint32_t       cal_tick_ms = 0;                      // ms elapsed in current phase
static float          cal_win[CAL_WINDOW];                  // rolling sample window
static uint8_t        cal_win_idx = 0;
static uint8_t        cal_win_cnt = 0;
// Per-phase IIR on the resistance value.  Filters R directly so correlated noise
// in vsup/vsense cancels through the R formula.  Holds its last value when an
// individual raw reading is momentarily invalid, keeping cal_win rotating.
// Reset to 0 at the start of each sampling phase by cal_reset_window().
static float          cal_r_filt  = 0.0f;

// ═════════════════════════════════════════════════════════════════════════════
// Init — TIM3 CH2 / PB5 PWM, 1 kHz
// ═════════════════════════════════════════════════════════════════════════════

void MW_Init(void)
{
    // ── GPIO: PB5 → AF2 (TIM3_CH2) ──────────────────────────────────────────
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

    LL_GPIO_SetPinMode       (GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7     (GPIOB, LL_GPIO_PIN_5, LL_GPIO_AF_2);
    LL_GPIO_SetPinOutputType (GPIOB, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinSpeed      (GPIOB, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinPull       (GPIOB, LL_GPIO_PIN_5, LL_GPIO_PULL_NO);

    // ── TIM3 ─────────────────────────────────────────────────────────────────
    // APB1 timer clock = 84 MHz
    // PSC = 83 → tick = 1 µs
    // ARR = 999 → period = 1 ms (1 kHz)
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

    LL_TIM_SetPrescaler  (TIM3, MW_PWM_PSC);
    LL_TIM_SetAutoReload (TIM3, MW_PWM_ARR);
    LL_TIM_SetCounterMode(TIM3, LL_TIM_COUNTERMODE_UP);
    LL_TIM_EnableARRPreload(TIM3);

    // CH2 — PWM Mode 1: output HIGH while CNT < CCR (FET conducts during HIGH)
    LL_TIM_OC_SetMode      (TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetCompareCH2(TIM3, 0u);
    LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);

    // ── CC4 one-shot compare — ADC sample trigger ─────────────────────────────
    // CH4 output pin is NOT enabled; only the CC4 interrupt is used.
    // FROZEN mode: compare match does not affect any output — interrupt only.
    // CCR4 is written dynamically by the CC2 ISR on each ON→OFF edge.
    LL_TIM_OC_SetMode      (TIM3, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_FROZEN);
    LL_TIM_OC_SetCompareCH4(TIM3, 0u);
    // CH4 output NOT enabled — LL_TIM_CC_EnableChannel intentionally omitted

    // Load preloaded values, then start
    LL_TIM_GenerateEvent_UPDATE(TIM3);
    LL_TIM_EnableCounter(TIM3);

    // ── TIM3 IRQ — CC4 only (CC2 not used) ───────────────────────────────────
    // CC4 is armed by MW_SampleR (100 ms tea action) before each wanted sample.
    // The ISR is two uint16 reads + disable — no blocking, no ADC1 activity.
    LL_TIM_ClearFlag_CC4(TIM3);
    NVIC_SetPriority(TIM3_IRQn, 8u);
    NVIC_EnableIRQ(TIM3_IRQn);

    // ── Dual simultaneous ADC ─────────────────────────────────────────────
    // Retire the ADC2 continuous DMA scan; configure ADC1+ADC2 for dual
    // regular simultaneous mode so vsup (IN3) and vsense (IN8) are captured
    // at exactly the same instant.  Supply noise cancels in the R ratio.
    ADC_SimInit();

    // Register the HTTP feed action and start the 200 ms self-reschedule chain.
    namedAction(MW_HttpFeed);
    after(msec(200), MW_HttpFeed);

    // MW_SampleR: arms CC4 and converts the captured raw values → adc_latch.
    // Runs at 100 ms so adc_latch is always within one period of fresh.
    namedAction(MW_SampleR);
    after(msec(100), MW_SampleR);

    // Register calibration tick so the scheduler can find it by name.
    // MW_CLI_Calibrate() starts it; it stops itself when done.
    namedAction(MW_CalTick);

    // Register closed-loop tick so the scheduler can find it by name.
    // MW_SetTarget() starts it; it stops itself when disabled.
    namedAction(MW_CLTick);

    // ── Initial VDDA calibration ──────────────────────────────────────────
    // s_vdda_mv defaults to 3300 mV.  On this board VDDA is ~2928 mV; using
    // the wrong value causes all millivolt conversions to read ~13 % high.
    // Run one full ADC_Driver_Update() now so s_vdda_mv is correct before
    // the first MW_SampleR fires.  The OVR cleanup inside ADC_Driver_Update()
    // handles the piggyback ADC2 conversions from the internal-channel passes.
    {
        ADC_Results_t vdda_cal;
        ADC_Driver_Update(&vdda_cal);
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// PWM control
// ═════════════════════════════════════════════════════════════════════════════

void MW_SetPWM(uint8_t percent)
{
    if (percent > MW_PWM_MAX_PCT)
        percent = MW_PWM_MAX_PCT;
    pwm_pct = percent;
    // ARR = 999 → 1 % = 10 counts → 0.1 % resolution across 0–999
    LL_TIM_OC_SetCompareCH2(TIM3, (uint32_t)percent * 10u);
}

uint8_t MW_GetPWM(void)
{
    return pwm_pct;
}

// ═════════════════════════════════════════════════════════════════════════════
// Phase detection
//
// PWM Mode 1: output HIGH (FET gate driven) while CNT < CCR.
// ON phase  → CNT < CCR  → FET conducting → current in sense R → R_wire valid.
// OFF phase → CNT ≥ CCR  → FET off, no current, sense node ≈ 0 V.
// ═════════════════════════════════════════════════════════════════════════════

bool MW_IsOnPhase(void)
{
    return (LL_TIM_GetCounter(TIM3) < LL_TIM_OC_GetCompareCH2(TIM3));
}

// ═════════════════════════════════════════════════════════════════════════════
// TIM3 IRQ — CC4 one-shot fires MW_ADC_SETTLE_TICKS µs into the ON phase
//
// MW_SampleR (100 ms tea action) arms CC4 at CCR4 = min(MW_ADC_SETTLE_TICKS,
// CCR2/2) so the ISR fires during the settled ON window regardless of duty.
// When CC4 fires the ISR triggers a dual regular simultaneous single-shot on
// ADC1+ADC2: vsup (IN3/PA3) and vsense (IN8/PB0) captured at the same cycle.
//
// Measuring during ON phase: current flows through the inline sense resistor,
// so vsense = I × R_SENSE gives a direct measure of wire current.
// Supply-rail noise cancels in the R_wire ratio.
// ADC conversion: ~714 ns (15 cycles @ 21 MHz); spin-wait is bounded.
//
// Call from TIM3_IRQHandler in stm32f4xx_it.c:
//   void TIM3_IRQHandler(void) { MW_TIM3_IRQHandler(); }
// ═════════════════════════════════════════════════════════════════════════════

void MW_TIM3_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_CC4(TIM3))
    {
        LL_TIM_ClearFlag_CC4(TIM3);
        LL_TIM_DisableIT_CC4(TIM3);     // one-shot: MW_SampleR re-arms next time

        // FET has been ON for MW_ADC_SETTLE_TICKS µs — sample now.
        // ADC_SimBurstRead takes MW_ADC_SAMPLES simultaneous IN3+IN8 pairs
        // back-to-back (28-cycle sample time, 21 MHz clock):
        //   1 pair  ≈ 1.93 µs   MW_ADC_SAMPLES = 4 → ~7.7 µs total
        // Averaged result written directly into adc_raw_latch.
        // Scope strobe pulses HIGH for the entire burst window.
        ADC_SimBurstRead(&adc_raw_latch.raw0, &adc_raw_latch.raw1, MW_ADC_SAMPLES);
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// ADC readings  (from shared adc_driver DMA buffer)
// ═════════════════════════════════════════════════════════════════════════════

// ═════════════════════════════════════════════════════════════════════════════
// MW_SampleR — 100 ms tea action: take ON-phase sample → update adc_latch
//
// When PWM > 0:
//   1. Convert adc_raw_latch (filled by CC4 ISR MW_ADC_SETTLE_TICKS µs into
//      the ON phase) to millivolts using the cached VDDA.
//   2. Arm CC4 for the settled point inside the next ON phase.
//      CCR4 = min(MW_ADC_SETTLE_TICKS, CCR2/2) ensures CC4 always fires while
//      the FET is still conducting, even at low duty cycles.
//
// When PWM = 0:
//   FET is off — no current path, so the sense node reads ~0 V.
//   A brief direct-drive pulse forces the FET ON for ~150 µs so that a valid
//   current-based resistance reading can be taken:
//     1. Set CCR2 = MW_ADC_SETTLE_TICKS + 50; apply via GenerateEvent_UPDATE.
//        Timer resets to 0; output goes HIGH (FET turns on).
//     2. Spin on TIM3 counter until CNT ≥ MW_ADC_SETTLE_TICKS (100 µs).
//     3. Trigger and read both ADC channels.
//     4. Restore CCR2 = 0; apply via GenerateEvent_UPDATE. FET turns off.
//   pwm_pct stays 0 — only the timer hardware is briefly manipulated.
//
// adc_latch is updated every 100 ms regardless of PWM state, so all
// consumers (adc_snap, CalTick, HttpFeed) always see a recent ON-phase reading.
// ═════════════════════════════════════════════════════════════════════════════

void MW_SampleR(void)
{
    const float vdda = ADC_GetVDDA_mv();    // last calibrated VDDA (mV)
    float new_vsup, new_vsense;

    if (pwm_pct == 0u)
    {
        // ── Brief force-on pulse for 0 % duty ───────────────────────────────
        // Set a short ON pulse (CCR2 = SETTLE + 50 µs) and apply immediately
        // via GenerateEvent_UPDATE (which resets the counter to 0 and transfers
        // the preloaded CCR2 shadow → active).  The output goes HIGH at once
        // since CNT(0) < CCR2_active.
        const uint32_t pulse_ccr = (uint32_t)(MW_ADC_SETTLE_TICKS + 50u);
        LL_TIM_OC_SetCompareCH2(TIM3, pulse_ccr);
        LL_TIM_GenerateEvent_UPDATE(TIM3);   // applies CCR2, resets CNT → 0

        // Spin on the hardware counter — no NOPs, no HAL delay.
        while (LL_TIM_GetCounter(TIM3) < MW_ADC_SETTLE_TICKS) {}

        // Sample both channels simultaneously (FET settled for SETTLE_TICKS µs).
        // MW_ADC_SAMPLES burst pairs averaged — same noise reduction as ISR path.
        uint16_t raw0, raw1;
        ADC_SimBurstRead(&raw0, &raw1, MW_ADC_SAMPLES);

        // Immediately restore 0 % — turn FET off.
        LL_TIM_OC_SetCompareCH2(TIM3, 0u);
        LL_TIM_GenerateEvent_UPDATE(TIM3);   // applies CCR2=0, resets CNT → 0

        new_vsup   = (float)raw0 * vdda / 4095.0f * MW_SCALE_CH0;
        new_vsense = (float)raw1 * vdda / 4095.0f * MW_SCALE_CH1;
    }
    else
    {
        // ── Convert raw values latched by the CC4 ISR ───────────────────────
        new_vsup   = (float)adc_raw_latch.raw0 * vdda / 4095.0f * MW_SCALE_CH0;
        new_vsense = (float)adc_raw_latch.raw1 * vdda / 4095.0f * MW_SCALE_CH1;

        // ── Re-arm CC4 for the settled point in the next ON phase ───────────
        // CCR4 fires MW_ADC_SETTLE_TICKS µs after the rising edge (timer wrap).
        // Cap at half the ON-phase duration so low duty cycles still sample
        // while the FET is conducting.
        uint32_t ccr2 = LL_TIM_OC_GetCompareCH2(TIM3);
        uint32_t ccr4 = MW_ADC_SETTLE_TICKS;
        if (ccr4 >= ccr2) ccr4 = ccr2 / 2u;
        if (ccr4 < 2u)    ccr4 = 2u;        // absolute minimum: 2 µs
        LL_TIM_OC_SetCompareCH4(TIM3, ccr4);
        LL_TIM_ClearFlag_CC4(TIM3);
        LL_TIM_EnableIT_CC4(TIM3);
    }

    // ── IIR low-pass filter ──────────────────────────────────────────────────
    // Seed the filter on first call (both states are 0.0 at startup).
    if (iir_vsup_mv == 0.0f)
    {
        iir_vsup_mv   = new_vsup;
        iir_vsense_mv = new_vsense;
    }
    else
    {
        iir_vsup_mv   = MW_IIR_ALPHA * new_vsup   + (1.0f - MW_IIR_ALPHA) * iir_vsup_mv;
        iir_vsense_mv = MW_IIR_ALPHA * new_vsense + (1.0f - MW_IIR_ALPHA) * iir_vsense_mv;
    }

    adc_latch.vsup_mv   = iir_vsup_mv;
    adc_latch.vsense_mv = iir_vsense_mv;

    after(msec(100), MW_SampleR);
}

// adc_snap — return the most recent ON-phase reading from adc_latch.
// Always valid: updated by MW_SampleR every 100 ms for both PWM states
// (0 % uses a brief force-on pulse; >0 % uses CC4 in the ON phase).
static void adc_snap(float *vsup_mv, float *vsense_mv)
{
    *vsup_mv   = adc_latch.vsup_mv;
    *vsense_mv = adc_latch.vsense_mv;
}

float MW_GetVsupply_mv(void)
{
    float vsup, vsense;
    adc_snap(&vsup, &vsense);
    return vsup;
}

float MW_GetVsense_mv(void)
{
    float vsup, vsense;
    adc_snap(&vsup, &vsense);
    return vsense;
}

// R_wire = MW_R_SENSE × (V_supply − V_sense) / V_sense
//
// Valid only when FET is conducting (current through sense resistor).
// MW_SampleR always takes readings during ON phase, so adc_latch.vsense_mv
// reflects a real current measurement.  Returns 0 on open-circuit or error.
float MW_GetResistance(void)
{
    float vsup, vsense;
    adc_snap(&vsup, &vsense);

    if (vsense < 5.0f)      return 0.0f;  // no current — FET off or wire open
    if (vsense >= vsup)     return 0.0f;  // implausible: sense ≥ supply

    return MW_R_SENSE * (vsup - vsense) / vsense;
}

// Positive = wire shorter than relaxed (0–100 % of calibrated travel).
// Uses measured R_max / R_min when cal-wire has completed; falls back to
// compile-time defaults MW_R_WIRE_RELAXED / MW_R_WIRE_CONTRACTED otherwise.
float MW_GetContraction(void)
{
    float r = MW_GetResistance();
    if (r <= 0.0f) return 0.0f;
    float r_top = cal_valid ? cal_r_max : MW_R_WIRE_RELAXED;
    float r_bot = cal_valid ? cal_r_min : MW_R_WIRE_CONTRACTED;
    float span  = r_top - r_bot;
    if (span <= 0.0f) return 0.0f;
    float c = (r_top - r) / span * 100.0f;
    if (c < 0.0f)   c = 0.0f;
    if (c > 100.0f) c = 100.0f;
    return c;
}

// ═════════════════════════════════════════════════════════════════════════════
// Profile capture
//
// Records one reading every MW_PROFILE_PERIOD_MS ms.
// Samples come from adc_latch (filled by CC4 ISR at the settled point in the
// OFF phase) so every entry is at the same deterministic position in the cycle.
// MW_ProfileTick() must be called from HAL_SYSTICK_Callback (1 ms cadence).
// ═════════════════════════════════════════════════════════════════════════════

void MW_StartProfile(uint8_t duty_pct)
{
    memset(profile, 0, sizeof(profile));
    profile_idx   = 0;
    profile_tick  = 0;
    profile_duty  = duty_pct;
    profile_start = HAL_GetTick();
    profile_state = MW_PROFILE_RUNNING;

    MW_SetPWM(duty_pct);

    print("profile start  duty=");
    printDec(duty_pct);
    print("%  samples=");
    printDec(MW_PROFILE_LEN);
    print("  interval=");
    printDec(MW_PROFILE_PERIOD_MS);
    print("ms");
    printCr();
}

MW_ProfileState_t MW_GetProfileState(void) { return profile_state; }
uint32_t          MW_GetProfileCount(void)  { return profile_idx;   }

// Fast path — called every 1 ms from SysTick; returns immediately when idle.
void MW_ProfileTick(void)
{
    if (profile_state != MW_PROFILE_RUNNING) return;

    if (++profile_tick < MW_PROFILE_PERIOD_MS) return;
    profile_tick = 0;

    // adc_latch is always an ON-phase reading (maintained by MW_SampleR every
    // 100 ms).  Use it directly so the profile entries are consistent with
    // the live resistance readings.
    MW_Sample_t *s  = &profile[profile_idx];
    s->time_ms      = HAL_GetTick() - profile_start;
    s->raw_ch0      = adc_raw_latch.raw0;
    s->raw_ch1      = adc_raw_latch.raw1;
    s->v_supply_mv  = adc_latch.vsup_mv;
    s->v_sense_mv   = adc_latch.vsense_mv;

    if (s->v_sense_mv > 5.0f && s->v_sense_mv < s->v_supply_mv)
        s->r_wire = MW_R_SENSE * (s->v_supply_mv - s->v_sense_mv) / s->v_sense_mv;
    else
        s->r_wire = 0.0f;

    s->contraction = (s->r_wire > 0.0f)
                     ? 100.0f * (1.0f - s->r_wire / MW_R_WIRE_RELAXED)
                     : 0.0f;

    if (++profile_idx >= MW_PROFILE_LEN)
    {
        profile_state = MW_PROFILE_DONE;
        print("profile done");
        printCr();
        // PWM left running — use  setpwm 0  to stop.
    }
}

// Emit CSV to CLI output.  Paste into a spreadsheet or plot tool.
void MW_DumpProfile(void)
{
    if (profile_state == MW_PROFILE_RUNNING)
    {
        print("profile running  ");
        printDec(profile_idx);
        print("/");
        printDec(MW_PROFILE_LEN);
        printCr();
        return;
    }
    if (profile_idx == 0)
    {
        print("no profile data — use start-profile");
        printCr();
        return;
    }

    print("# muscle wire profile  duty=");
    printDec(profile_duty);
    print("%  relaxed=");
    printFloat(MW_R_WIRE_RELAXED, 1);
    print(" ohm");
    printCr();

    print("time_ms,raw_ch0,raw_ch1,vsup_mv,vsense_mv,r_wire,contraction_pct");
    printCr();

    for (uint32_t i = 0; i < profile_idx; i++)
    {
        MW_Sample_t *s = &profile[i];
        printDec(s->time_ms);   print(",");
        printDec(s->raw_ch0);   print(",");
        printDec(s->raw_ch1);   print(",");
        printFloat(s->v_supply_mv, 1);  print(",");
        printFloat(s->v_sense_mv,  3);  print(",");
        printFloat(s->r_wire,      2);  print(",");
        printFloat(s->contraction, 2);
        printCr();
    }
    print("# end");
    printCr();
}

// ═════════════════════════════════════════════════════════════════════════════
// HTTP live feed — pushes data to /mw_stream every 200 ms
//
// Self-rescheduling TimbreOS action started by MW_Init().
// Sends one {"live":[Vsup_V, R_wire_ohm, PWM_pct]} SSE frame.
// R_wire is only valid during the OFF phase; 0.0 is sent otherwise so the
// graph shows a clean gap rather than garbage — the browser auto-scales
// around the valid data.
// ═════════════════════════════════════════════════════════════════════════════

void MW_HttpFeed(void)
{
    float vsup, vsense;
    adc_snap(&vsup, &vsense);

    // adc_latch is always an ON-phase reading; R is valid whenever vsense > 0.
    float r = 0.0f;
    if (vsense > 5.0f && vsense < vsup)
        r = MW_R_SENSE * (vsup - vsense) / vsense;

    http_mw_live_feed(vsup / 1000.0f,   // mV → V
                      r,                // Ω
                      (float)pwm_pct);  // 0–70 %

    after(msec(200), MW_HttpFeed);      // reschedule — 5 Hz continuous feed
}

// ═════════════════════════════════════════════════════════════════════════════
// Self-calibration
//
// Procedure (triggered by cal-wire):
//   RMAX_SETTLE  hold 0% PWM for CAL_RMAX_SETTLE_MS — wire cools, fully extends.
//   RMAX_SAMPLE  collect OFF-phase readings until CAL_WINDOW samples span
//                less than CAL_STABLE_BAND Ω; record mean as R_max.
//   RMIN_HEAT    ramp to MW_PWM_MAX_PCT, wait CAL_RMIN_HEAT_MS for contraction.
//   RMIN_SAMPLE  same stability test; record mean as R_min.  PWM → 0.
//
// Spike filter: R_min samples outside [0.5×R_max … R_max] are discarded.
// After a successful run MW_GetContraction() uses the measured limits.
// ═════════════════════════════════════════════════════════════════════════════

static void cal_win_push(float r)
{
    cal_win[cal_win_idx] = r;
    cal_win_idx = (uint8_t)((cal_win_idx + 1u) % CAL_WINDOW);
    if (cal_win_cnt < CAL_WINDOW) cal_win_cnt++;
}

static bool cal_is_stable(float *mean_out)
{
    if (cal_win_cnt < CAL_WINDOW) return false;
    float lo = cal_win[0], hi = cal_win[0], sum = 0.0f;
    for (uint8_t i = 0; i < CAL_WINDOW; i++) {
        if (cal_win[i] < lo) lo = cal_win[i];
        if (cal_win[i] > hi) hi = cal_win[i];
        sum += cal_win[i];
    }
    *mean_out = sum / (float)CAL_WINDOW;
    return (hi - lo) < CAL_STABLE_BAND;
}

static void cal_reset_window(void)
{
    cal_win_idx = 0;
    cal_win_cnt = 0;
    cal_tick_ms = 0;
    cal_r_filt  = 0.0f;     // reset per-phase IIR so each phase seeds fresh
    // Zero the array so stale values from a previous phase are not visible
    // in printLast debug output.  The stability check ignores unwritten slots
    // via cal_win_cnt, but zeroed values make the debug printout unambiguous.
    for (uint8_t i = 0; i < CAL_WINDOW; i++) cal_win[i] = 0.0f;
}

static void printLast(float r_raw) {
    float vsup, vsense;
    adc_snap(&vsup, &vsense);
    print("cal dbg  vsup=");   printFloat(vsup   / 1000.0f, 3);
    print("V  vsense=");       printFloat(vsense / 1000.0f, 3);
    print("V  r_raw=");        printFloat(r_raw, 2);
    print("  r_filt=");        printFloat(cal_r_filt, 2);
    printCr();
    print("cal_win: ");
    for (uint8_t i = 0; i < CAL_WINDOW; i++) {
        printFloat(cal_win[i], 4);
        print(" ");
    }
    printCr();
}

void MW_CalTick(void)
{
    cal_tick_ms += 200u;

    // adc_latch is always an ON-phase reading (updated by MW_SampleR every 100 ms).
    float r = 0.0f;
    {
        float vsup, vsense;
        adc_snap(&vsup, &vsense);
        if (vsense > 5.0f && vsense < vsup)
            r = MW_R_SENSE * (vsup - vsense) / vsense;
    }

    if (cal_tick_ms % 1000u == 0u) printLast(r);

    switch (cal_state) {

        case MW_CAL_RMAX_SETTLE:
            if (cal_tick_ms >= CAL_RMAX_SETTLE_MS) {
                cal_reset_window();
                cal_state = MW_CAL_RMAX_SAMPLE;
                print("cal: sampling R_max..."); printCr();
            }
            break;

        case MW_CAL_RMAX_SAMPLE: {
            // Update resistance IIR when the raw reading is valid.
            // When r_raw is momentarily 0 (vsense glitch) the IIR simply holds its
            // last value — cal_r_filt stays non-zero and cal_win keeps rotating.
            if (r > 0.0f) {
                cal_r_filt = (cal_r_filt == 0.0f)
                             ? r
                             : (CAL_R_ALPHA * r + (1.0f - CAL_R_ALPHA) * cal_r_filt);
            }
            if (cal_r_filt > 0.0f) cal_win_push(cal_r_filt);
            float mean;
            if (cal_is_stable(&mean)) {
                cal_r_max = mean;
                print("cal: R_max = "); printFloat(cal_r_max, 2); print(" ohm"); printCr();
                MW_SetPWM(MW_PWM_MAX_PCT);
                cal_reset_window();
                cal_state = MW_CAL_RMIN_HEAT;
                print("cal: heating to "); printDec(MW_PWM_MAX_PCT); print("%..."); printCr();
                break;
            }
            if (cal_tick_ms >= CAL_TIMEOUT_MS) {
                cal_state = MW_CAL_TIMEOUT;
                print("cal: timeout — R_max unstable"); printCr();
                printLast(r);
                return;
            }
            break;
        }

        case MW_CAL_RMIN_HEAT:
            if (cal_tick_ms >= CAL_RMIN_HEAT_MS) {
                cal_reset_window();
                cal_state = MW_CAL_RMIN_SAMPLE;
                print("cal: sampling R_min..."); printCr();
            }
            break;

        case MW_CAL_RMIN_SAMPLE: {
            // Spike filter: accept any positive R reading.
            // With simultaneous ADC sampling the noise largely cancels in the
            // ratio, so we don't need a tight range gate.  The IIR (cal_r_filt)
            // provides the smoothing; cal_is_stable() enforces the ±0.15 Ω
            // convergence criterion before accepting the result.
            // The old upper bound (r < cal_r_max) caused cal_r_filt to stay
            // zero when the heated wire hadn't contracted far enough yet, or
            // when IIR residual from the RMAX phase briefly pushed r above
            // cal_r_max by a tiny margin.
            if (r > 0.0f) {
                cal_r_filt = (cal_r_filt == 0.0f)
                             ? r
                             : (CAL_R_ALPHA * r + (1.0f - CAL_R_ALPHA) * cal_r_filt);
            }
            if (cal_r_filt > 0.0f) cal_win_push(cal_r_filt);
            float mean;
            if (cal_is_stable(&mean)) {
                cal_r_min  = mean;
                cal_valid  = true;
                MW_SetPWM(0);
                cal_state  = MW_CAL_DONE;
                float span = cal_r_max - cal_r_min;
                print("cal: R_min = "); printFloat(cal_r_min, 2); print(" ohm"); printCr();
                print("cal: span  = "); printFloat(span, 2);
                print(" ohm  ("); printFloat(100.0f * span / cal_r_max, 1);
                print("% dR/R)"); printCr();

                // Quality check: warn if the span looks implausibly narrow.
                //
                // A healthy calibration should capture at least 30 % dR/R
                // (≈ 6 Ω on a wire with R_max ≈ 20 Ω).  A small span almost
                // always means the wire was still warm when R_max was sampled —
                // the 5-second settle window was not long enough.
                //
                // The two thresholds are intentionally independent so the
                // message is specific:
                //   R_max < 12 Ω → wire was partially contracted at start.
                //   span  < 2 Ω  → travel range too small for reliable control.
                bool r_max_suspect = (cal_r_max < 12.0f);
                bool span_suspect  = (span < 2.0f);
                if (r_max_suspect || span_suspect) {
                    print("cal: WARNING — calibration quality is poor:"); printCr();
                    if (r_max_suspect) {
                        print("  R_max "); printFloat(cal_r_max, 2);
                        print(" ohm is below 12 ohm; wire was not fully relaxed."); printCr();
                        print("  Run cal-wire again after the wire has cooled"); printCr();
                        print("  (show-pwm R wire should be near 20 ohm first)."); printCr();
                    }
                    if (span_suspect) {
                        print("  span "); printFloat(span, 2);
                        print(" ohm is too small; contraction % will be noisy."); printCr();
                    }
                    print("  Contraction readings marked (cal) are unreliable."); printCr();
                }
                return;     // done — do not reschedule
            }
            if (cal_tick_ms >= CAL_TIMEOUT_MS) {
                MW_SetPWM(0);
                cal_state = MW_CAL_TIMEOUT;
                print("cal: timeout — R_min unstable"); printCr();
                printLast(r);
                return;
            }
            break;
        }

        default:
            return;
    }

    after(msec(200), MW_CalTick);
}

MW_CalState_t  MW_GetCalState(void) { return cal_state; }
bool           MW_IsCalValid(void)  { return cal_valid;  }
float          MW_GetCalRmax(void)  { return cal_r_max;  }
float          MW_GetCalRmin(void)  { return cal_r_min;  }

// ═════════════════════════════════════════════════════════════════════════════
// CLI handlers
// ═════════════════════════════════════════════════════════════════════════════

// show-pwm — duty cycle, voltages, resistance, contraction.
// Reads from adc_latch (always an ON-phase snapshot kept fresh by MW_SampleR).
void MW_CLI_ShowPWM(void)
{
    float vsup, vsense;
    adc_snap(&vsup, &vsense);

    float r_wire = 0.0f;
    float pct    = 0.0f;

    if (vsense > 5.0f && vsense < vsup)
    {
        r_wire      = MW_R_SENSE * (vsup - vsense) / vsense;
        float r_top = cal_valid ? cal_r_max : MW_R_WIRE_RELAXED;
        float r_bot = cal_valid ? cal_r_min : MW_R_WIRE_CONTRACTED;
        float span  = r_top - r_bot;
        if (span > 0.0f) {
            pct = (r_top - r_wire) / span * 100.0f;
            if (pct < 0.0f)   pct = 0.0f;
            if (pct > 100.0f) pct = 100.0f;
        }
    }

    maybeCr();
    print("--- muscle wire ---");
    printCr();

    print("PWM duty");   tabTo(COL_VALUE);
    printDec(pwm_pct);  print("%");
    print("  ceiling ");
    printDec(MW_PWM_MAX_PCT); print("%");
    printCr();

    print("phase");      tabTo(COL_VALUE);
    print(MW_IsOnPhase() ? "ON  (valid)" : "OFF (no current)");
    printCr();

    print("V supply");   tabTo(COL_VALUE);
    printFloat(vsup   / 1000.0f, 2);  tabTo(COL_UNIT); print("V");
    printCr();

    print("V sense");    tabTo(COL_VALUE);
    printFloat(vsense / 1000.0f, 3);  tabTo(COL_UNIT); print("V");
    print("  (I × 1 Ω)");
    printCr();

    print("R wire");     tabTo(COL_VALUE);
    if (r_wire > 0.0f)
    {
        printFloat(r_wire, 1); tabTo(COL_UNIT); print("ohm");
    }
    else
    {
        print("--");
    }
    printCr();

    print("contraction"); tabTo(COL_VALUE);
    if (r_wire > 0.0f)
    {
        printFloat(pct, 1); tabTo(COL_UNIT); print("%");
        print(cal_valid ? "  (cal)" : "  (est)");
    }
    else
    {
        print("--");
    }
    printCr();

    if (profile_state == MW_PROFILE_RUNNING)
    {
        print("profile");  tabTo(COL_VALUE);
        printDec(profile_idx); print("/"); printDec(MW_PROFILE_LEN);
        printCr();
    }
    else if (profile_state == MW_PROFILE_DONE)
    {
        print("profile");  tabTo(COL_VALUE);
        printDec(profile_idx); print(" samples ready");
        printCr();
    }

    print("cl target");  tabTo(COL_VALUE);
    if (cl_target_pct >= 0.0f) {
        printFloat(cl_target_pct, 1); tabTo(COL_UNIT); print("%  active");
    } else {
        print("off");
    }
    printCr();

    print("---");
    printCr();
}

// setpwm ( n ) — set duty cycle n = 0..100
// Uses ret() to fetch the argument from the data stack, matching
// the accel_regs_read / CLI convention used throughout the project.
void MW_CLI_SetPWM(void)
{
    uint8_t pct = (uint8_t)ret();
    uint8_t prev = pwm_pct;
    MW_SetTarget(-1.0f);    // manual override — disable closed loop
    MW_SetPWM(pct);

    printDec(prev); print("% -> "); printDec(pwm_pct); print("%");
    if (pct > MW_PWM_MAX_PCT)
    {
        print("  (clamped from ");
        printDec(pct);
        print("%)");
    }
    printCr();
}

// start-profile ( n ) — apply n% duty and begin a 25.6 s profile capture.
// Requires MW_ProfileTick() wired into HAL_SYSTICK_Callback.
void MW_CLI_StartProfile(void)
{
    uint8_t pct = (uint8_t)ret();
    MW_StartProfile(pct);
}

// cal-wire — run the two-point self-calibration procedure.
// Sets PWM to 0, waits for thermal settle, samples R_max, then ramps to
// MW_PWM_MAX_PCT, waits for contraction, samples R_min.  Takes ~20–30 s.
// Progress messages are printed as each phase completes.
void MW_CLI_Calibrate(void)
{
    // Refuse if a run is already in progress
    if (cal_state == MW_CAL_RMAX_SETTLE ||
        cal_state == MW_CAL_RMAX_SAMPLE ||
        cal_state == MW_CAL_RMIN_HEAT   ||
        cal_state == MW_CAL_RMIN_SAMPLE)
    {
        print("cal: already running — wait for done/timeout"); printCr();
        return;
    }

    MW_SetTarget(-1.0f);    // disable closed loop during calibration
    MW_SetPWM(0);
    cal_reset_window();
    cal_state = MW_CAL_RMAX_SETTLE;

    print("cal: settling ");
    printDec(CAL_RMAX_SETTLE_MS / 1000u);
    print("s at 0%...");
    printCr();

    after(msec(200), MW_CalTick);
}

// show-cal — display current calibration state and limits.
void MW_CLI_ShowCal(void)
{
    static const char * const s_state[] = {
        "idle", "settling", "R_max sample", "heating", "R_min sample", "done", "timeout"
    };

    maybeCr();
    print("--- calibration ---"); printCr();

    print("state");  tabTo(COL_VALUE);
    print(s_state[cal_state]); printCr();

    print("R_max");  tabTo(COL_VALUE);
    printFloat(cal_r_max, 2); tabTo(COL_UNIT); print("ohm");
    print(cal_valid ? "" : "  (default)"); printCr();

    print("R_min");  tabTo(COL_VALUE);
    printFloat(cal_r_min, 2); tabTo(COL_UNIT); print("ohm");
    print(cal_valid ? "" : "  (default)"); printCr();

    if (cal_valid) {
        float span = cal_r_max - cal_r_min;
        print("span");   tabTo(COL_VALUE);
        printFloat(span, 2); tabTo(COL_UNIT); print("ohm"); printCr();

        print("travel"); tabTo(COL_VALUE);
        printFloat(100.0f * span / cal_r_max, 1); tabTo(COL_UNIT); print("% dR/R"); printCr();

        print("quality"); tabTo(COL_VALUE);
        if (cal_r_max < 12.0f)
            print("POOR — R_max too low (wire was warm at calibration start)");
        else if (span < 2.0f)
            print("POOR — span too small for reliable contraction control");
        else
            print("ok");
        printCr();
    }

    print("---"); printCr();
}

// ═════════════════════════════════════════════════════════════════════════════
// Closed-loop contraction controller
//
// PI controller: every CL_PERIOD_MS ms the tick samples the current contraction,
// computes an error against the target, and adjusts PWM accordingly.
//
// Three non-linearities are handled explicitly:
//
//   1. Deadband  — no action within ±CL_DEADBAND_PCT of target; integral
//                  is zeroed here to prevent windup at rest.
//
//   2. Gain scheduling — Kp is scaled down in the low-contraction (wire cold,
//                  sluggish response) and high-contraction (near thermal limit)
//                  regions to reduce overshoot and thermal stress.
//
//   3. Asymmetric cooling — if the wire overshoots the target by more than
//                  CL_FAST_COOL_THRESH %, PWM is zeroed immediately.  Cooling
//                  is passive so gradual reduction is pointless; zeroing lets
//                  the wire cool as fast as physics allows.
//
// The loop is disabled automatically when setpwm or cal-wire takes control.
// ═════════════════════════════════════════════════════════════════════════════

void MW_SetTarget(float pct)
{
    if (pct < 0.0f)
    {
        cl_target_pct = -1.0f;
        cl_integral   =  0.0f;
        return;
    }
    if (pct > 100.0f) pct = 100.0f;

    bool was_off  = (cl_target_pct < 0.0f);
    cl_target_pct = pct;
    cl_integral   = 0.0f;    // reset integral on every new target

    if (was_off)
        after(msec(CL_PERIOD_MS), MW_CLTick);   // start the tick chain
}

float MW_GetTarget(void)     { return cl_target_pct; }
bool  MW_IsClosedLoop(void)  { return cl_target_pct >= 0.0f; }

// ── MW_CLTick — 500 ms self-rescheduling PI control step ──────────────────
void MW_CLTick(void)
{
    if (cl_target_pct < 0.0f) return;   // disabled — do not reschedule

    float measured = MW_GetContraction();
    float error    = cl_target_pct - measured;

    // ── 1. Deadband: hold current PWM when close enough ──────────────────
    if (fabsf(error) <= CL_DEADBAND_PCT)
    {
        cl_integral = 0.0f;             // bleed off integral at rest
        after(msec(CL_PERIOD_MS), MW_CLTick);
        return;
    }

    // ── 2. Asymmetric cooling: big overshoot → kill power, let wire cool ─
    if (error < -CL_FAST_COOL_THRESH)
    {
        MW_SetPWM(0);
        cl_integral = 0.0f;
        after(msec(CL_PERIOD_MS), MW_CLTick);
        return;
    }

    // ── 3. Gain scheduling: reduce Kp at the extremes ────────────────────
    float kp_scale = (cl_target_pct < CL_LOW_THRESHOLD ||
                      cl_target_pct > CL_HIGH_THRESHOLD)
                     ? CL_EDGE_GAIN : 1.0f;

    // ── 4. PI update ──────────────────────────────────────────────────────
    cl_integral += error;

    // Anti-windup: clamp so the integral's PWM contribution stays ≤ CL_I_LIMIT
    float i_limit = CL_I_LIMIT / (CL_KI > 0.0f ? CL_KI : 1.0f);
    if (cl_integral >  i_limit) cl_integral =  i_limit;
    if (cl_integral < -i_limit) cl_integral = -i_limit;

    float adjustment = CL_KP * kp_scale * error + CL_KI * cl_integral;

    // Apply delta to current PWM; clamp to valid range
    int32_t new_pwm = (int32_t)pwm_pct
                    + (int32_t)(adjustment >= 0.0f
                                ? adjustment + 0.5f
                                : adjustment - 0.5f);
    if (new_pwm < 0)                       new_pwm = 0;
    if (new_pwm > (int32_t)MW_PWM_MAX_PCT) new_pwm = (int32_t)MW_PWM_MAX_PCT;

    MW_SetPWM((uint8_t)new_pwm);
    after(msec(CL_PERIOD_MS), MW_CLTick);
}

// ── set-per ( n ) — set closed-loop contraction target ────────────────────
void MW_CLI_SetPercent(void)
{
    int32_t n = (int32_t)(uint8_t)ret();   // 0–255 from stack; values >100 invalid
    if (n < 0 || n > 100)
    {
        print("set-per: argument must be 0–100"); printCr();
        return;
    }
    MW_SetTarget((float)n);
    print("target "); printDec((uint32_t)n); print("%  closed-loop on"); printCr();
}

// ── cl-off — disable closed-loop controller ───────────────────────────────
void MW_CLI_CLOff(void)
{
    MW_SetTarget(-1.0f);
    print("closed-loop off"); printCr();
}

// ── show-cl — show closed-loop controller state ───────────────────────────
void MW_CLI_ShowCL(void)
{
    maybeCr();
    print("--- closed loop ---"); printCr();

    print("state");      tabTo(COL_VALUE);
    print(cl_target_pct >= 0.0f ? "active" : "off"); printCr();

    if (cl_target_pct >= 0.0f)
    {
        float measured = MW_GetContraction();
        float error    = cl_target_pct - measured;

        print("target");   tabTo(COL_VALUE);
        printFloat(cl_target_pct, 1); tabTo(COL_UNIT); print("%"); printCr();

        print("measured");  tabTo(COL_VALUE);
        if (measured > 0.0f) { printFloat(measured, 1); tabTo(COL_UNIT); print("%"); }
        else                   print("--");
        printCr();

        print("error");    tabTo(COL_VALUE);
        if (measured > 0.0f)
        {
            if (error >= 0.0f) print("+");
            printFloat(error, 1); tabTo(COL_UNIT); print("%");
            if (fabsf(error) <= CL_DEADBAND_PCT) print("  (in band)");
        }
        else { print("--"); }
        printCr();

        print("integral"); tabTo(COL_VALUE);
        printFloat(cl_integral, 1); printCr();

        print("PWM");      tabTo(COL_VALUE);
        printDec(pwm_pct); print("%"); printCr();

        print("region");   tabTo(COL_VALUE);
        if      (cl_target_pct < CL_LOW_THRESHOLD)  print("low  (reduced gain)");
        else if (cl_target_pct > CL_HIGH_THRESHOLD) print("high (reduced gain)");
        else                                         print("mid");
        printCr();
    }

    print("---"); printCr();
}

void print_float() {
    Byte n = ret();
    union{ Cell c; float f; } u = {.c=ret()};
    printFloat(u.f, n);
}