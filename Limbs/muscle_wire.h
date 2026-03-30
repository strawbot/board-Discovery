// muscle_wire.h — Nitinol muscle wire PWM driver + resistance monitor
//
// Hardware (STM32F4DISCOVERY)
// ───────────────────────────
//   PWM output : TIM3 CH2  →  PB5 (AF2)  →  ULN2803A input
//     PA6 is NOT used — it is SPI1_MISO (accelerometer).
//     PB0/PB1 are NOT used — they are ADC2 IN8/IN9.
//
//   ADC readings come from the shared adc_driver (ADC2, continuous DMA).
//     ADC_IDX_IN3  (PA3) = CH0 — supply voltage sense divider
//     ADC_IDX_IN8  (PB0) = CH1 — Node_A / wire sense divider
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
// Sense circuit — R_SENSE is permanently wired, no switch needed
// ──────────────────────────────────────────────────────────────
//   Vsupply → [R_wire] → Node_A ──┬── ULN2803A collector → emitter → GND
//                                  └── [MW_R_SENSE] ──────────────────> GND
//
//   During ON  phase: Node_A ≈ Vce_sat (~1 V).  R_sense draws ~4.5 mA —
//                     negligible against wire current. Reading discarded in SW.
//   During OFF phase: ULN collector is high-Z. R_sense forms the lower leg
//                     of the divider with R_wire. Node_A is valid for R calc.
//
//   V_node_A = Vsupply × R_SENSE / (R_wire + R_SENSE)
//   → R_wire = R_SENSE × (Vsupply − V_node) / V_node
//
//   ADC_CH0 (supply) divider : R1 = 100 kΩ, R2 = 20 kΩ  → scale = 6.0
//   ADC_CH1 (node)   divider : Rtop = 82 kΩ, Rbot = 22 kΩ → scale ≈ 4.73
//   Adjust MW_SCALE_CH0 / MW_SCALE_CH1 to match actual fitted values.
//
// CLI bindings (discoverywords.txt)
// ──────────────────────────────────
//   setpwm   MW_CLI_SetPWM    // ( n ) set duty to n percent
//   show-pwm MW_CLI_ShowPWM   // show duty, voltages, resistance
//
// Profile capture
// ───────────────
//   start-profile sets PWM and begins recording one OFF-phase ADC sample
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
// Tune these to match the actual resistor values once fitted.
#define MW_R_SENSE              220.0f      // Ω — pull-down / sense resistor
#define MW_SCALE_CH0            6.00f       // (R1+R2)/R2 for supply divider
#define MW_SCALE_CH1            4.727f      // (Rtop+Rbot)/Rbot for node divider

// Wire physical constants (8-inch Flexinol, Teflon-sleeved)
#define MW_R_WIRE_RELAXED       45.0f       // Ω  cold, no load
#define MW_R_WIRE_CONTRACTED    38.0f       // Ω  estimated; refine from profile

// ── Profile capture ───────────────────────────────────────────────────────────
#define MW_PROFILE_LEN          512u        // total samples per run
#define MW_PROFILE_PERIOD_MS    50u         // one sample every N ms
// Total window = 512 × 50 ms = 25.6 seconds

// ── Types ─────────────────────────────────────────────────────────────────────
typedef struct {
    uint32_t time_ms;       // ms elapsed since profile start
    uint16_t raw_ch0;       // ADC raw — supply
    uint16_t raw_ch1;       // ADC raw — node
    float    v_supply_mv;   // mV scaled
    float    v_node_mv;     // mV scaled
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

// Live readings from the shared ADC DMA buffer.
// R_wire and contraction are only meaningful during the OFF phase.
float   MW_GetVsupply_mv(void);
float   MW_GetVnode_mv(void);
float   MW_GetResistance(void);
float   MW_GetContraction(void);
bool    MW_IsOffPhase(void);

// Profile (call MW_ProfileTick from HAL_SYSTICK_Callback)
void               MW_StartProfile(uint8_t duty_pct);
MW_ProfileState_t  MW_GetProfileState(void);
uint32_t           MW_GetProfileCount(void);
void               MW_ProfileTick(void);    // fast no-op when idle
void               MW_DumpProfile(void);    // CSV to CLI output

// HTTP live feed — self-rescheduling action; started automatically by MW_Init().
// Pushes one {"live":[Vsup_V, R_wire_ohm, PWM_pct]} frame to any browser
// client subscribed to /mw_stream.  Runs at 5 Hz (every 200 ms).
// Register as namedAction in your init file if you need to reference it
// from a different translation unit; MW_Init() already does this.
void    MW_HttpFeed(void);

// CLI handlers — bind via discoverywords.txt
void    MW_CLI_ShowPWM(void);               // show-pwm
void    MW_CLI_SetPWM(void);                // setpwm       ( n )
void    MW_CLI_StartProfile(void);          // start-profile ( n )

#endif // MUSCLE_WIRE_H_
