// muscle_wire.h — Nitinol muscle wire PWM driver + resistance monitor
//
// Hardware (STM32F4DISCOVERY)
// ───────────────────────────
//   PWM output : TIM3 CH2  →  PB5 (AF2)  →  MOSFET gate
//
//   ADC readings come from the shared adc_driver (dual simultaneous ADC1+ADC2).
//     ADC_IDX_IN3  (PA3) = CH0 — supply voltage sense divider
//     ADC_IDX_IN8  (PB0) = CH1 — sense resistor (FET source to GND)
//   No separate DMA is set up here; call MW_Init() after ADC_Driver_Init().
//
// Timer clock path
// ────────────────
//   System = 168 MHz,  APB1 prescaler /4 → APB1 = 42 MHz
//   TIM3 clock = 2 × APB1 = 84 MHz
//   PSC = 83  →  tick = 1 µs
//   ARR = 999 →  period = 1 ms  (1 kHz)
//   CCR = duty_pct × 10          (0–100 %, 0.1 % resolution)
//
// Sense circuit — ON-phase measurement
// ─────────────────────────────────────
//   5V ──[R_wire]──[FET drain]──[FET source]──[MW_R_SENSE = 1 Ω]── GND
//                                           ^
//                                     ADC CH1 (PB0)
//
//   During ON  phase: FET conducts; current flows through R_wire + FET + R_SENSE.
//                     V_sense = I × R_SENSE = I × 1 Ω  (current-proportional).
//                     R is valid — sample here.
//   During OFF phase: FET off; no current path; ADC CH1 ≈ 0 V.  R not valid.
//
//   Assuming FET fully-on (V_ds ≈ 0):
//     V_sense = I × 1 Ω
//     V_wire  = V_supply − V_sense          (≈ V_supply − I)
//     R_wire  = V_wire / I = MW_R_SENSE × (V_supply − V_sense) / V_sense
//
//   ADC_CH0 (supply) divider : R1 = 100 kΩ, R2 = 20 kΩ  → scale = 6.0
//   ADC_CH1 (sense)  : direct connection to sense resistor → scale = 1.0
//   Adjust MW_SCALE_CH0 to match actual fitted resistor values.
//
// CLI bindings (discoverywords.txt)
// ──────────────────────────────────
//   setpwm   MW_CLI_SetPWM    // ( n ) set duty to n percent
//   show-pwm MW_CLI_ShowPWM   // show duty, voltages, resistance
//
// Profile capture
// ───────────────
//   start-profile sets PWM and begins recording one ON-phase ADC sample
//   every MW_PROFILE_PERIOD_MS milliseconds for MW_PROFILE_LEN samples.
//   MW_ProfileTick() must be called from the 1 ms HAL SysTick callback:
//       void HAL_SYSTICK_Callback(void) { MW_ProfileTick(); }
//   When complete use dump-profile to emit CSV to the CLI output.

#ifndef MUSCLE_WIRE_H_
#define MUSCLE_WIRE_H_

#include <stdint.h>
#include <stdbool.h>

// ── PWM ───────────────────────────────────────────────────────────────────────
#define MW_PWM_PSC              83u         // 84 MHz → 1 µs tick
#define MW_PWM_ARR              999u        // 1 ms period → 1 kHz
// CCR = duty_pct × 10

// Safety ceiling — set-pwm clamps to this value.
// Lower if the wire runs hot; raise only after thermal characterisation.
#define MW_PWM_MAX_PCT          70u

// ── Sense circuit ─────────────────────────────────────────────────────────────
// Tune MW_SCALE_CH0 to match actual supply-divider resistors once fitted.
// MW_R_SENSE must match the actual sense resistor (1 Ω inline with FET source).
#define MW_R_SENSE              1.0f        // Ω — inline sense resistor (FET source)
#define MW_SCALE_CH0            6.00f       // (R1+R2)/R2 for supply divider
#define MW_SCALE_CH1            1.0f        // direct connection — no divider on sense node

// Wire physical constants (8-inch Flexinol, Teflon-sleeved)
#define MW_R_WIRE_RELAXED       8.9f       // Ω  cold, no load
#define MW_R_WIRE_CONTRACTED    7.7f       // Ω  estimated; refine from profile

// ── Profile capture ───────────────────────────────────────────────────────────
#define MW_PROFILE_LEN          512u        // total samples per run
#define MW_PROFILE_PERIOD_MS    50u         // one sample every N ms
// Total window = 512 × 50 ms = 25.6 seconds

// ── ADC filtering ─────────────────────────────────────────────────────────────
// Single-stage IIR low-pass (on converted float values, at 100 ms intervals):
//   y[n] = α·x[n] + (1−α)·y[n−1]
//   At 10 Hz: time constant τ ≈ −1 / (f_s · ln(1−α))
//   MW_IIR_ALPHA = 0.20 → τ ≈ 450 ms  (tracks wire thermal changes)
//
// Increase MW_IIR_ALPHA (toward 1.0) for faster but noisier response.
// Decrease it for heavier smoothing.
#define MW_IIR_ALPHA            0.20f

// ── ADC sample timing ─────────────────────────────────────────────────────────
// At the minimum PWM setting (2 % = 20 µs ON time) the sampling window is
// 10 µs – 17.5 µs, leaving margin on both sides of the FET transition.
//
// CC4 fires MW_ADC_SETTLE_TICKS µs after the rising edge (timer wrap).
// The CC4 ISR then runs MW_ADC_SAMPLES simultaneous ADC1+ADC2 pairs
// back-to-back, accumulates the results, and averages before storing.
//
// One simultaneous pair at 28-cycle sample time:
//   (28 + 12.5) / 21 MHz ≈ 1.93 µs
// Four pairs: 4 × 1.93 µs = 7.72 µs
//
// At 2 % duty (CCR4 = min(SETTLE, CCR2/2) = min(10,10) = 10 µs):
//   Sample window: 10.0 – 17.7 µs  ✓  within the 20 µs ON phase
//
// For pwm_pct > 0: CC4 armed at CCR4 = min(SETTLE_TICKS, CCR2/2).
// For pwm_pct = 0: MW_SampleR forces FET ON via a brief direct CCR2 pulse,
//   spins SETTLE_TICKS µs on the TIM3 counter, runs the burst, then restores
//   CCR2 = 0.
//
// Wire MW_TIM3_IRQHandler() into TIM3_IRQHandler in stm32f4xx_it.c.
#define MW_ADC_SETTLE_TICKS     10u     // µs into ON phase before first sample
#define MW_ADC_SAMPLES          4u      // simultaneous pairs per reading (√4 = 2× noise)

// ── Self-calibration ──────────────────────────────────────────────────────────
// cal-wire procedure: hold 0% until stable (R_max), then max% until stable (R_min).
// MW_CalTick() self-reschedules at 200 ms; started by MW_CLI_Calibrate().
#define CAL_WINDOW          8u          // samples in stability window (8 × 200 ms = 1.6 s)
#define CAL_STABLE_BAND     0.10f       // Ω — max window spread to declare stability
// IIR applied to R_wire inside MW_CalTick before pushing to the stability window.
// α = 0.15 → τ ≈ 1.2 s at the 200 ms CalTick rate.
#define CAL_R_ALPHA         0.15f
#define CAL_RMAX_SETTLE_MS  5000u       // ms at 0% PWM before R_max sampling begins
#define CAL_RMIN_HEAT_MS    3000u       // ms at max PWM before R_min sampling begins
#define CAL_TIMEOUT_MS      25000u      // ms before aborting a sampling phase

// ── Closed-loop contraction controller ───────────────────────────────────────
// Fuzzy rule-based controller driving PWM to hold a target contraction %.
// MW_CLTick() self-reschedules at CL_PERIOD_MS; started by MW_SetTarget().
// Calling MW_SetTarget(-1) or setpwm / cal-wire disables the loop.
//
// Design rationale
// ────────────────
//   With a 1.2 Ω span, 1 % contraction = 0.012 Ω.  ADC noise is ~0.1–0.2 Ω,
//   which maps to ±8–16 % contraction noise.  A PI controller with a tight
//   deadband fights that noise continuously and oscillates.  Instead, the
//   fuzzy rules use a deadband wider than the noise floor, absolute PWM
//   levels rather than incremental deltas, and a rate term (current minus
//   previous reading) to anticipate overshoot before it occurs.
//
// Operating regions
// ─────────────────
//   |error| > FZ_LARGE_ERR  : heat hard / cut power hard
//   FZ_SMALL_ERR < |error| ≤ FZ_LARGE_ERR : moderate action; rate modulates
//   FZ_DEAD_PCT  < |error| ≤ FZ_SMALL_ERR : gentle action; rate modulates
//   |error| ≤ FZ_DEAD_PCT   : deadband — maintenance current only
//
// Tuning notes
// ────────────
//   FZ_HOLD_PWM  is the key knob: raise it if the wire slowly relaxes at
//                target; lower it if it slowly over-contracts.
//   FZ_DEAD_PCT  must exceed the measurement noise floor (≈ ±8 %).
//   FZ_LARGE_ERR / FZ_SMALL_ERR set the transition boundaries.
//   FZ_RATE_THRESH controls how aggressively the anticipatory term fires.
#define CL_PERIOD_MS         500u    // control update interval (ms)
#define FZ_DEAD_PCT          8.0f    // ±% deadband — sized to exceed noise floor
#define FZ_SMALL_ERR        12.0f    // small/medium error boundary (%)
#define FZ_LARGE_ERR        25.0f    // medium/large error boundary (%)
#define FZ_HIGH_PWM         75u      // PWM% applied for large positive error
#define FZ_MED_PWM          45u      // PWM% applied for medium positive error
#define FZ_LOW_PWM          25u      // PWM% applied for small positive error
#define FZ_HOLD_PWM         18u      // maintenance PWM in/near deadband
#define FZ_RATE_THRESH       4.0f    // %/tick rate that triggers anticipation

typedef enum {
    MW_CAL_IDLE        = 0,
    MW_CAL_RMAX_SETTLE,     // waiting for wire to cool at 0% PWM
    MW_CAL_RMAX_SAMPLE,     // collecting stable R_max window (0% PWM, force-on pulses)
    MW_CAL_RMIN_HEAT,       // holding max PWM, waiting for wire to contract
    MW_CAL_RMIN_SAMPLE,     // collecting stable R_min window
    MW_CAL_DONE,
    MW_CAL_TIMEOUT,
} MW_CalState_t;

// ── Types ─────────────────────────────────────────────────────────────────────
typedef struct {
    uint32_t time_ms;       // ms elapsed since profile start
    uint16_t raw_ch0;       // ADC raw — supply
    uint16_t raw_ch1;       // ADC raw — sense resistor
    float    v_supply_mv;   // mV scaled
    float    v_sense_mv;    // mV across sense resistor (= I × R_SENSE)
    float    r_wire;        // Ω calculated
    float    contraction;   // % shortening from relaxed (positive = shorter)
} MW_Sample_t;

typedef enum {
    MW_PROFILE_IDLE    = 0,
    MW_PROFILE_RUNNING = 1,
    MW_PROFILE_DONE    = 2,
} MW_ProfileState_t;

// ── Public API ────────────────────────────────────────────────────────────────

// Call once after ADC_Driver_Init() — sets up TIM3 / PB5, starts PWM at 0 %.
void    MW_Init(void);

// PWM control
void    MW_SetPWM(uint8_t percent);         // 0 – MW_PWM_MAX_PCT (clamped)
uint8_t MW_GetPWM(void);

// Live readings derived from adc_latch (updated by MW_SampleR every 100 ms).
// R_wire and contraction require ON-phase sampling to be meaningful.
float   MW_GetVsupply_mv(void);
float   MW_GetVsense_mv(void);      // mV across inline sense resistor (≈ I × 1 Ω)
float   MW_GetResistance(void);
float   MW_GetContraction(void);
bool    MW_IsOnPhase(void);         // true while CNT < CCR2 (FET conducting)

// Profile (call MW_ProfileTick from HAL_SYSTICK_Callback)
void               MW_StartProfile(uint8_t duty_pct);
MW_ProfileState_t  MW_GetProfileState(void);
uint32_t           MW_GetProfileCount(void);
void               MW_ProfileTick(void);    // fast no-op when idle
void               MW_DumpProfile(void);    // CSV to CLI output

// HTTP live feed — self-rescheduling action; started automatically by MW_Init().
// Pushes one {"live":[Vsup_V, R_wire_ohm, PWM_pct]} frame to any browser
// client subscribed to /mw_stream.  Runs at 5 Hz (every 200 ms).
void    MW_HttpFeed(void);

// Self-calibration — driven by MW_CalTick() at 200 ms intervals.
// After cal-wire completes, MW_GetContraction() uses measured limits instead
// of the compile-time defaults.
MW_CalState_t  MW_GetCalState(void);
bool           MW_IsCalValid(void);
float          MW_GetCalRmax(void);
float          MW_GetCalRmin(void);
void           MW_CalTick(void);            // self-rescheduling; started by MW_CLI_Calibrate
void           MW_SampleR(void);            // 100 ms self-rescheduling: take ON-phase sample
void           MW_TIM3_IRQHandler(void);    // call from TIM3_IRQHandler in stm32f4xx_it.c

// Closed-loop contraction controller
// MW_SetTarget() starts the loop; MW_CLTick() must be registered via MW_Init().
// The loop stops itself when MW_SetTarget(-1) is called or when setpwm / cal-wire
// takes manual control.
void    MW_SetTarget(float pct);    // set target % (0–100) and enable; < 0 = disable
float   MW_GetTarget(void);         // current target, or < 0 if disabled
bool    MW_IsClosedLoop(void);      // true while loop is active
void    MW_CLTick(void);            // 500 ms self-rescheduling; registered by MW_Init()

// CLI handlers — bind via discoverywords.txt
void    MW_CLI_ShowPWM(void);               // show-pwm
void    MW_CLI_SetPWM(void);                // setpwm        ( n )
void    MW_CLI_StartProfile(void);          // start-profile  ( n )
void    MW_CLI_Calibrate(void);             // cal-wire
void    MW_CLI_ShowCal(void);               // show-cal
void    MW_CLI_SetPercent(void);            // set-per       ( n )
void    MW_CLI_ShowCL(void);                // show-cl
void    MW_CLI_CLOff(void);                 // cl-off

#endif // MUSCLE_WIRE_H_
