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

    // Load preloaded values, then start
    LL_TIM_GenerateEvent_UPDATE(TIM3);
    LL_TIM_EnableCounter(TIM3);

    // Register the HTTP feed action and start the 200 ms self-reschedule chain.
    namedAction(MW_HttpFeed);
    after(msec(200), MW_HttpFeed);
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
// ADC readings  (from shared adc_driver DMA buffer)
// ═════════════════════════════════════════════════════════════════════════════

// Snapshot both channels in one call to keep them consistent.
static void adc_snap(float *vsup_mv, float *vnode_mv)
{
    ADC_Results_t r;
    ADC_Driver_Update(&r);
    // voltage_mv[] is already scaled to actual pin voltage (0 – VDDA range).
    // Apply the external resistor divider ratios to get real voltages.
    *vsup_mv  = r.voltage_mv[ADC_IDX_IN3] * MW_SCALE_CH0;
    *vnode_mv = r.voltage_mv[ADC_IDX_IN8] * MW_SCALE_CH1;
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

// Positive = wire shorter than relaxed. Nitinol drops ~10–15 % at full stroke.
float MW_GetContraction(void)
{
    float r = MW_GetResistance();
    if (r <= 0.0f) return 0.0f;
    return 100.0f * (1.0f - r / MW_R_WIRE_RELAXED);
}

// ═════════════════════════════════════════════════════════════════════════════
// Profile capture
//
// Records one OFF-phase reading every MW_PROFILE_PERIOD_MS ms.
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

    // Skip if currently in ON phase — retry next period.
    // At 50 ms / 1 ms PWM period this almost never fires during transition.
    if (!MW_IsOffPhase()) return;

    ADC_Results_t r;
    ADC_Driver_Update(&r);

    MW_Sample_t *s  = &profile[profile_idx];
    s->time_ms      = HAL_GetTick() - profile_start;
    s->raw_ch0      = r.raw[ADC_IDX_IN3];
    s->raw_ch1      = r.raw[ADC_IDX_IN8];
    s->v_supply_mv  = r.voltage_mv[ADC_IDX_IN3] * MW_SCALE_CH0;
    s->v_node_mv    = r.voltage_mv[ADC_IDX_IN8] * MW_SCALE_CH1;

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
        r_wire = MW_R_SENSE * (vsup - vnode) / vnode;
        pct    = 100.0f * (1.0f - r_wire / MW_R_WIRE_RELAXED);
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
