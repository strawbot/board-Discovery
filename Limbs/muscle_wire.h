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
//   PSC = 0   →  tick = 1/84 MHz ≈ 11.9 ns
//   ARR = 839 →  period = 840 ticks = 10 µs  (100 kHz)
//   CCR = duty_pct × (ARR+1) / 100  =  duty_pct × 840 / 100
//
// Sense circuit — capacitor hold approach (required at 100 kHz)
// ─────────────────────────────────────────────────────────────
//   5V ──[R_wire]──[FET drain]──[FET source]──[MW_R_SENSE = 1 Ω]──[C_hold = 100 nF]── GND
//                                                                ^
//                                                          ADC CH1 (PB0)
//
//   At 100 kHz the ON phase is only 0.18–3.5 µs at normal PWM settings —
//   far too short to settle, arm a compare match, and run a burst ADC read.
//   Instead a 100 nF capacitor across R_SENSE acts as a sample-and-hold:
//
//   RC time constants (R_SENSE = 1 Ω, C = 100 nF, R_wire ≈ 9 Ω):
//     τ_charge  = (R_wire ∥ R_SENSE) × C  ≈ 0.9 Ω × 100 nF = 90 ns
//     τ_hold    = R_ADC_in × C            ≈ 1 MΩ × 100 nF  = 100 ms
//
//   During ON  phase:  FET conducts; cap charges to V_ON in ~5τ = 450 ns.
//                      At 10 % duty (1 µs ON) the cap reaches > 99.9 % of V_ON.
//   During OFF phase:  FET off; cap holds V_ON (droop < 0.001 % per 10 µs cycle).
//
//   Net result: ADC CH1 reads ≈ V_ON continuously, regardless of PWM phase.
//   No CC4 compare or ISR synchronisation is required.
//   The same R_wire formula remains valid (V_sense ≈ V_ON).
//
//   For pwm_pct = 0 (no PWM running):
//     MW_SampleR forces FET ON briefly (99.9 % duty → cap charges in < 500 ns),
//     runs the ADC burst, then restores 0 % duty.
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
#define MW_PWM_PSC              0u          // 84 MHz clock, no prescaler; tick ≈ 11.9 ns
#define MW_PWM_ARR              839u        // period = 840 ticks = 10 µs → 100 kHz
// CCR = duty_pct × (MW_PWM_ARR + 1u) / 100u  =  duty_pct × 840 / 100

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
// At 100 kHz the ON phase is ≤ 3.5 µs at normal PWM settings.
// The ADC burst (4 pairs × 1.93 µs = 7.72 µs) cannot fit inside a single
// ON phase, so the CC4 one-shot approach used at 1 kHz is abandoned.
//
// Instead the 100 nF cap across R_SENSE holds V_ON between pulses (τ_hold ≈ 100 ms).
// MW_SampleR calls ADC_SimBurstRead directly — no timer synchronisation needed.
//
// For pwm_pct = 0: MW_SampleR briefly forces 99.9 % duty (CCR2 = MW_PWM_ARR)
//   via GenerateEvent_UPDATE, waits MW_ADC_SETTLE_TICKS timer counts for the cap
//   to charge, runs the burst, then restores CCR2 = 0.
//
// MW_ADC_SETTLE_TICKS is now in 84 MHz timer ticks (≈ 11.9 ns each):
//   5 × τ_charge = 5 × 90 ns ≈ 450 ns → 38 ticks
//
// MW_TIM3_IRQHandler is retained as a no-op stub so stm32f4xx_it.c compiles
// without changes.  The NVIC for TIM3 is no longer enabled.
#define MW_ADC_SETTLE_TICKS     38u     // timer ticks (84 MHz) to charge cap before sample
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
// Fuzzy controller with piecewise-linear interpolation driving PWM to hold
// a target contraction %.  MW_CLTick() self-reschedules at CL_PERIOD_MS.
// Calling MW_SetTarget(-1) or setpwm / cal-wire disables the loop.
//
// Design rationale
// ────────────────
//   With a 1.2 Ω span, 1 % contraction = 0.012 Ω.  Measurement noise maps
//   to ±10–15 % contraction noise.  Output is computed by interpolating
//   smoothly between control points rather than stepping between discrete
//   levels — so small noise-driven error changes produce small PWM changes,
//   not sudden cuts that let the spring snap the wire back.
//
// Output mapping — piecewise linear interpolation
// ────────────────────────────────────────────────
//   Positive error (needs more contraction), heating ramp:
//     0 → DEAD_PCT : flat at HOLD_PWM  (deadband)
//     DEAD → SMALL : interpolate LOW_PWM → MED_PWM
//     SMALL → LARGE: interpolate MED_PWM → HIGH_PWM
//     > LARGE      : flat at HIGH_PWM
//
//   Negative error (above target), cooling ramp:
//     0 → DEAD_PCT : flat at HOLD_PWM  (deadband)
//     DEAD → SMALL : interpolate HOLD_PWM → SOFT_COOL_PWM
//     SMALL → LARGE: interpolate SOFT_COOL_PWM → 0 %
//     > LARGE      : flat at 0 %
//
//   Rate anticipation: proportional blend toward HOLD_PWM when wire is
//   already moving fast toward target — damps overshoot without cutting off.
//
// Tuning notes
// ────────────
//   FZ_HOLD_PWM  is the key knob: raise it if the wire relaxes at target;
//                lower it if it creeps up.
//   FZ_SOFT_COOL_PWM must stay below the activation threshold (~15–18 %).
//   FZ_DEAD_PCT  must exceed the measurement noise floor (≈ ±10–12 %).
//
// Tuning basis (from char-wire step-response data):
//   15% PWM → no sustained contraction (below activation threshold)
//   20% PWM → ~65% steady-state contraction (25 s slow creep), cool ~4 s
//   25% PWM → ~65% steady-state contraction (15 s slow creep), cool ~1.5 s
//   35% PWM → ~100% steady-state contraction (10 s creep),     cool ~3 s
//
//   The wire has a phase-transition plateau near 65%: both 20% and 25% PWM
//   converge there.  Exceeding 65% requires ≥35% PWM.  The activation
//   threshold (below which the spring wins) is ~15–18% PWM.
//
//   Practical controllable range: ~25–80% contraction.
//   Noise in the plateau zone: ±10–15% — deadband must exceed this.
//   τ_heat (fast phase): ~1.5–2 s.  τ_cool (spring-assisted): ~0.7–1.5 s.
//
#define CL_PERIOD_MS         300u    // 300 ms — catches 1.5 s fast rise in 5 ticks
#define FZ_DEAD_PCT         12.0f    // ±% deadband — exceeds ±10% plateau noise
#define FZ_SMALL_ERR        20.0f    // small/medium error boundary (%)
#define FZ_LARGE_ERR        35.0f    // medium/large error boundary (%)
#define FZ_HIGH_PWM          35u     // PWM% for large error — full activation power
#define FZ_MED_PWM           25u     // PWM% for medium error — plateau-level power
#define FZ_LOW_PWM           20u     // PWM% for small error — just above threshold
#define FZ_HOLD_PWM          18u     // maintenance PWM — just above activation threshold
                                     // raise if wire relaxes at target; lower if it creeps up
#define FZ_SOFT_COOL_PWM     10u     // sub-threshold cool — wire drifts back slowly, no spring snap
                                     // used when slightly above target; prevents hard 0% cuts
#define FZ_MIN_HOLD_TARGET   20.0f   // targets below this use 0% in the deadband — the spring
                                     // returns the wire without help; avoids heating near zero
#define FZ_RATE_THRESH      10.0f    // %/tick — fast initial rise is ~15–20%/tick

// ── Wire characterisation profiler ───────────────────────────────────────────
// char-wire ( n ) runs a three-phase automated step-response test:
//
//   Phase 0 — pre-cool : 0% PWM until contraction stabilises (up to CHAR_PRECOOL_MS)
//   Phase 1 — heat     : n% PWM; record until stable or CHAR_HEAT_MS timeout
//   Phase 2 — cool     : 0% PWM; record until stable or CHAR_COOL_MS timeout
//
// Samples are taken every CHAR_SAMPLE_MS via the tea scheduler; no SysTick
// wiring is required.  dump-char emits CSV for all three phases.
//
// Paste the CSV into a spreadsheet and plot contraction vs. time to read:
//   τ_heat  — time to reach 63 % of steady-state contraction after step-on
//   τ_cool  — time to fall to 37 % of peak after step-off
//   c_ss    — steady-state contraction at char_pwm % duty
//   spring  — initial contraction drop in the first 1–2 samples after power-off
//             (the mechanical restoring force the system applies when unloaded)
//
// Stability criterion: spread across CHAR_STABLE_COUNT consecutive readings
// (each already IIR-filtered) < CHAR_STABLE_PCT %.  Set wider than the noise
// floor so the trigger fires when the thermal transient has settled, not when
// measurement noise happens to be quiet.
#define CHAR_SAMPLE_MS        200u   // sample interval (ms) — matches MW_SampleR cadence
#define CHAR_PRECOOL_MS     15000u   // max pre-cool phase (ms)
#define CHAR_HEAT_MS        30000u   // max heat phase (ms)
#define CHAR_COOL_MS        30000u   // max cool phase (ms)
#define CHAR_MAX_SAMPLES      512u   // sample buffer depth (512 × 200 ms = 102 s)
#define CHAR_STABLE_PCT       5.0f   // % spread to declare stable — wider than noise floor
#define CHAR_STABLE_COUNT      10u   // samples in stability window (10 × 200 ms = 2 s)

typedef enum {
    MW_CHAR_IDLE     = 0,
    MW_CHAR_PRECOOL,        // waiting for wire to cool at 0%
    MW_CHAR_HEATING,        // PWM step applied; recording rise
    MW_CHAR_COOLING,        // PWM removed; recording fall
    MW_CHAR_DONE,
    MW_CHAR_ABORTED,
} MW_CharState_t;

typedef struct {
    uint32_t time_ms;       // ms since char-wire started
    uint8_t  phase;         // 0=precool  1=heat  2=cool
    uint8_t  pwm_pct;       // PWM duty at this sample
    float    r_wire;        // Ω
    float    contraction;   // % (cal-based if valid, defaults otherwise)
} MW_CharSample_t;

MW_CharState_t  MW_GetCharState(void);
void            MW_CharTick(void);          // tea action — started by MW_CLI_CharWire
void            MW_CLI_CharWire(void);      // char-wire  ( n ) — n = heat PWM %
void            MW_CLI_DumpChar(void);      // dump-char

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
