/**
 * @file    adc_sensors.h
 * @brief   Analog sensor drivers – MAP, IAT, CLT, TPS, wideband O2
 *
 * All sensors are read via the RP2350 ADC in round-robin DMA mode.
 * The DMA fills a circular buffer; the control loop samples the latest
 * averaged value from that buffer at 2 kHz.
 *
 * Sensor transfer functions
 * ─────────────────────────
 *
 *  MAP (e.g. Bosch 0 261 230 006  or MPX4250A)
 *    Output: 0.2 V … 4.8 V  →  0 … 250 kPa absolute
 *    V_out = 0.004 × P_kPa + 0.04   (datasheet approximation)
 *
 *  IAT / CLT  (NTC thermistor, e.g. GM 10kΩ / 2.5kΩ pull-up)
 *    Steinhart–Hart equation:
 *      1/T = A + B·ln(R) + C·(ln(R))³
 *    Simplified β-model:
 *      T = T₀ × T_ref / (T_ref × ln(R/R_ref) + T₀)
 *    where R is derived from the voltage divider.
 *
 *  TPS  (0-5 V potentiometer → 0-3.3 V via resistor divider)
 *    Linear: TPS% = (V_adc - V_min) / (V_max - V_min) × 100
 *
 *  O2 Wideband (Bosch LSU 4.9 via LC-2 controller  0-5 V = AFR 7.35-22.39)
 *    λ = AFR / 14.7
 *    AFR = V × (22.39 - 7.35) / 5.0 + 7.35
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ── ADC channel order in round-robin DMA ────────────────────
// RP2350 ADC channels 0-3 (GP26-GP29)
#define ADC_CH_MAP   0
#define ADC_CH_TPS   1
#define ADC_CH_IAT   2
#define ADC_CH_CLT   3
#define ADC_NUM_CH   4   // number of channels muxed

// ── NTC thermistor constants (GM-style 10 kΩ @ 25°C sensor) ─
#define NTC_R_REF_OHM   10000.0f   // R at T_ref
#define NTC_T_REF_K     298.15f    // 25°C in Kelvin
#define NTC_BETA_IAT    3977.0f    // β for IAT sensor
#define NTC_BETA_CLT    3977.0f    // β for CLT sensor
#define NTC_R_PULLUP    2490.0f    // pull-up resistor [Ω]

// ── MAP sensor constants (MPX4250A-style) ───────────────────
#define MAP_V_MIN_V     0.2f       // V at 0 kPa absolute
#define MAP_V_MAX_V     4.8f       // V at 250 kPa absolute
#define MAP_P_MAX_KPA   250.0f

// ── TPS calibration ─────────────────────────────────────────
#define TPS_V_CLOSED    0.50f      // V at closed throttle
#define TPS_V_OPEN      4.50f      // V at WOT (before divider)
// After 3.3 V divider (2:3 ratio):
#define TPS_ADC_MIN     (TPS_V_CLOSED * (3.3f / 5.0f))
#define TPS_ADC_MAX     (TPS_V_OPEN   * (3.3f / 5.0f))

// ── O2 sensor (Innovate LC-2 0-5 V = AFR 7.35-22.39) ───────
#define O2_AFR_MIN      7.35f
#define O2_AFR_MAX      22.39f
#define O2_V_MAX        5.0f      // full-scale voltage (before divider)

// ── Init / sample API ────────────────────────────────────────

/**
 * Initialise ADC hardware and DMA round-robin.
 * Call once during startup before the main loop.
 */
void adc_sensors_init(void);

/**
 * Process latest DMA buffer → compute physical values.
 * Call at Control Loop rate (2 kHz) from Core 0.
 * Results are placed into the output structure below.
 */
typedef struct {
    float map_kpa;      // manifold absolute pressure [kPa]
    float iat_degC;     // intake air temperature [°C]
    float clt_degC;     // coolant temperature [°C]
    float tps_pct;      // throttle position [0-100 %]
    float lambda;       // λ from wideband O2
} SensorReadings_t;

void adc_sensors_read(SensorReadings_t *out);

#ifdef __cplusplus
}
#endif
