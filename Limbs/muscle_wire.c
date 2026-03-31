// muscle_wire.c — Nitinol muscle wire PWM driver + resistance monitor
//
// PWM  : TIM3 CH2, PB5 (AF2), 1 kHz, 0.1 % duty resolution
// ADC  : uses shared adc_driver (ADC2, continuous DMA) — no separate DMA
//        IN3 (PA3) = supply sense, IN8 (PB0) = Node_A sense
// Print: printers.h API (print / printFloat / printDec / printCr / tabTo)
// CLI  : void/void functions; ret() fetches the data-stack argument

#include "muscle_wire.h"

#include <string.h>

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
//   Holds raw 12-bit DMA buffer values captured MW_ADC_SETTLE_TICKS µs after
//   the PWM falling edge.  ISR only writes; MW_SampleR (main loop) reads.
//
// adc_latch — written by MW_SampleR in main-loop context after converting the
//   raw counts to millivolts using the cached VDDA.  All other code reads this.
//   Always fresh within one MW_SampleR period (100 ms).
static volatile struct {
    uint16_t raw0;      // IN3 — supply sense
    uint16_t raw1;      // IN8 — node sense
} adc_raw_latch = { 0u, 0u };

static volatile struct {
    float vsup_mv;
    float vnode_mv;
} adc_latch = { 0.0f, 0.0f };

// IIR filter state — initialised to 0; MW_SampleR seeds from first reading.
static float iir_vsup_mv  = 0.0f;
static float iir_vnode_mv = 0.0f;

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
// in vsup/vnode cancels through the R formula.  Holds its last value when an
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

    // CH2 — PWM Mode 1: output HIGH while CNT < CCR (ULN conducts during HIGH)
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
    // regular simultaneous mode so vsup (IN3) and vnode (IN8) are captured
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
// PWM Mode 1: output HIGH (ULN conducting) while CNT < CCR.
// OFF phase → CNT ≥ CCR → ULN collector high-Z → Node_A valid for R_wire.
// ═════════════════════════════════════════════════════════════════════════════

bool MW_IsOffPhase(void)
{
    return (LL_TIM_GetCounter(TIM3) >= LL_TIM_OC_GetCompareCH2(TIM3));
}

// ═════════════════════════════════════════════════════════════════════════════
// TIM3 IRQ — CC4 one-shot fires MW_ADC_SETTLE_TICKS µs after the PWM edge
//
// MW_SampleR (100 ms tea action) arms CC4 before each wanted sample.
// When CC4 fires (100 µs into the OFF phase), the ISR triggers a dual
// regular simultaneous single-shot on ADC1+ADC2 so that vsup (IN3/PA3) and
// vnode (IN8/PB0) are captured at exactly the same clock cycle.
//
// Supply-rail noise common to both nodes cancels in the R_wire ratio, so no
// DMA oversampling window is needed.  The ADC conversion takes ~714 ns
// (15 cycles @ 21 MHz); the spin-wait is bounded and negligible relative to
// the 1 ms PWM period.
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

        // Trigger simultaneous single-shot: IN3 (vsup) and IN8 (vnode)
        // captured at exactly the same clock cycle.  Both channels sample
        // the same supply-noise phase so the noise cancels in the R ratio.
        // Spin-wait: 3 sample + 12 convert = 15 ADC cycles @ 21 MHz ≈ 714 ns.
        // This is bounded and far shorter than the PWM period (1 ms).
        ADC_SimTrigger();
        while (!ADC_SimReady()) {}
        ADC_SimRead(&adc_raw_latch.raw0, &adc_raw_latch.raw1);
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// ADC readings  (from shared adc_driver DMA buffer)
// ═════════════════════════════════════════════════════════════════════════════

// ═════════════════════════════════════════════════════════════════════════════
// MW_SampleR — 100 ms tea action: convert raw latch → float, re-arm CC4
//
// When PWM > 0:
//   1. Convert adc_raw_latch (filled by CC4 ISR at the settled OFF-phase point)
//      to millivolts using the cached VDDA.
//   2. Arm CC4 for the next cycle: CCR4 = CCR2 + MW_ADC_SETTLE_TICKS.
//      On the next PWM falling edge + SETTLE_TICKS µs, CC4 fires and the ISR
//      takes a fresh simultaneous ADC1+ADC2 shot into adc_raw_latch.
//
// When PWM = 0:
//   The entire period is OFF phase; take a fresh simultaneous single-shot
//   directly — no CC4 needed.
//
// adc_latch is updated every 100 ms regardless of PWM state, so all
// consumers (adc_snap, CalTick, HttpFeed) always see a recent reading.
// ═════════════════════════════════════════════════════════════════════════════

void MW_SampleR(void)
{
    const float vdda = ADC_GetVDDA_mv();    // last calibrated VDDA (mV)
    float new_vsup, new_vnode;

    if (pwm_pct == 0u)
    {
        // No PWM — the entire period is the OFF phase; take a fresh
        // simultaneous single-shot now.  No need to wait for CC4.
        ADC_SimTrigger();
        while (!ADC_SimReady()) {}
        uint16_t raw0, raw1;
        ADC_SimRead(&raw0, &raw1);
        new_vsup  = (float)raw0 * vdda / 4095.0f * MW_SCALE_CH0;
        new_vnode = (float)raw1 * vdda / 4095.0f * MW_SCALE_CH1;
    }
    else
    {
        // Convert raw values captured by the CC4 ISR at the settled point.
        new_vsup  = (float)adc_raw_latch.raw0 * vdda / 4095.0f * MW_SCALE_CH0;
        new_vnode = (float)adc_raw_latch.raw1 * vdda / 4095.0f * MW_SCALE_CH1;

        // Re-arm CC4 for the settled point in the next PWM cycle.
        uint32_t ccr4 = (LL_TIM_OC_GetCompareCH2(TIM3) + MW_ADC_SETTLE_TICKS)
                        % (MW_PWM_ARR + 1u);
        LL_TIM_OC_SetCompareCH4(TIM3, ccr4);
        LL_TIM_ClearFlag_CC4(TIM3);
        LL_TIM_EnableIT_CC4(TIM3);
    }

    // ── Stage 2: IIR low-pass filter ────────────────────────────────────────
    // Seed the filter on first call (both states are 0.0 at startup).
    if (iir_vsup_mv == 0.0f)
    {
        iir_vsup_mv  = new_vsup;
        iir_vnode_mv = new_vnode;
    }
    else
    {
        iir_vsup_mv  = MW_IIR_ALPHA * new_vsup  + (1.0f - MW_IIR_ALPHA) * iir_vsup_mv;
        iir_vnode_mv = MW_IIR_ALPHA * new_vnode + (1.0f - MW_IIR_ALPHA) * iir_vnode_mv;
    }

    adc_latch.vsup_mv  = iir_vsup_mv;
    adc_latch.vnode_mv = iir_vnode_mv;

    after(msec(100), MW_SampleR);
}

// adc_snap — return the most recent settled reading from adc_latch.
// Always valid: updated by MW_SampleR every 100 ms for both PWM states.
static void adc_snap(float *vsup_mv, float *vnode_mv)
{
    *vsup_mv  = adc_latch.vsup_mv;
    *vnode_mv = adc_latch.vnode_mv;
}

float MW_GetVsupply_mv(void)
{
    float vsup, vnode;
    adc_snap(&vsup, &vnode);
    return vsup;
}

float MW_GetVnode_mv(void)
{
    float vsup, vnode;
    adc_snap(&vsup, &vnode);
    return vnode;
}

// R_wire = R_SENSE × (Vsupply − V_node) / V_node
// Valid only during OFF phase; returns 0 if sense circuit looks open/shorted.
float MW_GetResistance(void)
{
    float vsup, vnode;
    adc_snap(&vsup, &vnode);

    if (vnode < 10.0f)    return 0.0f;    // sense circuit open / not connected
    if (vnode >= vsup)    return 0.0f;    // something wrong — avoid negative R

    return MW_R_SENSE * (vsup - vnode) / vnode;
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

    // No phase check needed: adc_latch is filled by the CC4 ISR at the settled
    // point in every PWM cycle, so the values here are always from the OFF phase.
    // When PWM = 0 adc_snap() reads directly (entire period is OFF phase).
    MW_Sample_t *s  = &profile[profile_idx];
    s->time_ms      = HAL_GetTick() - profile_start;

    // Capture latch directly so raw_ch0/ch1 and scaled voltages are consistent.
    ADC_Results_t r;
    ADC_Driver_Update(&r);
    s->raw_ch0      = r.raw[ADC_IDX_IN3];
    s->raw_ch1      = r.raw[ADC_IDX_IN8];
    s->v_supply_mv  = (pwm_pct == 0u) ? (r.voltage_mv[ADC_IDX_IN3] * MW_SCALE_CH0)
                                       : adc_latch.vsup_mv;
    s->v_node_mv    = (pwm_pct == 0u) ? (r.voltage_mv[ADC_IDX_IN8] * MW_SCALE_CH1)
                                       : adc_latch.vnode_mv;

    if (s->v_node_mv > 10.0f && s->v_node_mv < s->v_supply_mv)
        s->r_wire = MW_R_SENSE * (s->v_supply_mv - s->v_node_mv) / s->v_node_mv;
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

    print("time_ms,raw_ch0,raw_ch1,vsup_mv,vnode_mv,r_wire,contraction_pct");
    printCr();

    for (uint32_t i = 0; i < profile_idx; i++)
    {
        MW_Sample_t *s = &profile[i];
        printDec(s->time_ms);   print(",");
        printDec(s->raw_ch0);   print(",");
        printDec(s->raw_ch1);   print(",");
        printFloat(s->v_supply_mv, 1);  print(",");
        printFloat(s->v_node_mv,   1);  print(",");
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
    float vsup, vnode;
    adc_snap(&vsup, &vnode);

    float r = 0.0f;
    if (MW_IsOffPhase() && vnode > 10.0f && vnode < vsup)
        r = MW_R_SENSE * (vsup - vnode) / vnode;

    http_mw_live_feed(vsup / 1000.0f,   // mV → V
                      r,                // Ω (0 during ON phase)
                      (float)pwm_pct);  // 0–70 %

    after(msec(200), MW_HttpFeed);        // reschedule — 5 Hz continuous feed
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
}

static void printLast(float r_raw) {
    float vsup, vnode;
    adc_snap(&vsup, &vnode);
    print("cal dbg  vsup=");  printFloat(vsup / 1000.0f, 3);
    print("V  vnode=");       printFloat(vnode / 1000.0f, 3);
    print("V  r_raw=");       printFloat(r_raw, 2);
    print("  r_filt=");       printFloat(cal_r_filt, 2);
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

    // adc_latch is always a settled reading (updated by MW_SampleR every 100 ms).
    float r = 0.0f;
    {
        float vsup, vnode;
        adc_snap(&vsup, &vnode);
        if (vnode > 10.0f && vnode < vsup)
            r = MW_R_SENSE * (vsup - vnode) / vnode;
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
            // When r_raw is momentarily 0 (vnode glitch) the IIR simply holds its
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
// Takes a consistent snapshot so all values relate to the same conversion.
void MW_CLI_ShowPWM(void)
{
    bool off = MW_IsOffPhase();

    ADC_Results_t r;
    ADC_Driver_Update(&r);

    float vsup  = r.voltage_mv[ADC_IDX_IN3] * MW_SCALE_CH0;
    float vnode = r.voltage_mv[ADC_IDX_IN8] * MW_SCALE_CH1;
    float r_wire = 0.0f;
    float pct    = 0.0f;

    if (off && vnode > 10.0f && vnode < vsup)
    {
        r_wire      = MW_R_SENSE * (vsup - vnode) / vnode;
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
    print(off ? "OFF (valid)" : "ON  (R not valid)");
    printCr();

    print("V supply");   tabTo(COL_VALUE);
    printFloat(vsup  / 1000.0f, 2);  tabTo(COL_UNIT); print("V");
    printCr();

    print("V node");     tabTo(COL_VALUE);
    printFloat(vnode / 1000.0f, 3);  tabTo(COL_UNIT); print("V");
    print(off ? "" : "  (ON-phase)");
    printCr();

    print("R wire");     tabTo(COL_VALUE);
    if (off && r_wire > 0.0f)
    {
        printFloat(r_wire, 1); tabTo(COL_UNIT); print("ohm");
    }
    else
    {
        print("--");
    }
    printCr();

    print("contraction"); tabTo(COL_VALUE);
    if (off && r_wire > 0.0f)
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
    }

    print("---"); printCr();
}
