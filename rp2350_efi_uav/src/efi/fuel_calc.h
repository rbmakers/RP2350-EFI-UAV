/**
 * @file    fuel_calc.h
 * @brief   UAV-EFI fuel calculation
 *
 * Automotive → UAV modifications
 * ────────────────────────────────
 *
 *  1. Altitude-corrected IPW
 *     ─────────────────────
 *     The air mass already includes altitude via the KF/ideal-gas path.
 *     No additional σ multiplier is needed in the nominal path.
 *     In failsafe (MAP invalid), density_ratio scales the reference mass.
 *
 *  2. Alpha-N / Speed-Density blending
 *     ──────────────────────────────────
 *     At high altitude and during rapid throttle transients, MAP sensor
 *     lag causes Speed-Density to over-fuel. Alpha-N (TPS-based fuel map)
 *     is blended in based on RPM and the rate of MAP change (map_kpa_dot).
 *
 *     blend_factor = f(rpm)  ×  f(map_dot)
 *     IPW_final = blend × IPW_alpha_n + (1-blend) × IPW_speed_density
 *
 *  3. EGT enrichment
 *     ───────────────
 *     EGT > EGT_WARN_DEGC → add EGT_ENRICH_PCT_PER_50C % per 50°C above threshold.
 *     EGT > EGT_CRITICAL_DEGC → maximum enrichment + alert GCS.
 *
 *  4. Fail-Functional (not Limp) strategy
 *     ─────────────────────────────────────
 *     MAP fail  → substitute BAP × partial-load factor; flag FF_MAP_FIXED.
 *     IAT fail  → substitute 20°C; flag FF_IAT_FIXED.
 *     Both fail → use density_ratio × fixed reference; flag FF_EMERGENCY.
 *     Engine kill ONLY if EGT critical AND no enrichment response possible.
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "../ecu_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VE_RPM_BINS   16
#define VE_LOAD_BINS  16

// Alpha-N table: rows = RPM, cols = TPS%
#define AN_TPS_BINS   16
extern const float AN_TPS_AXIS[AN_TPS_BINS];   // 0-100 %

extern const float VE_TABLE [VE_RPM_BINS][VE_LOAD_BINS];
extern const float AFR_TABLE[VE_RPM_BINS][VE_LOAD_BINS];
extern const float AN_TABLE [VE_RPM_BINS][AN_TPS_BINS];  // [UAV] Alpha-N IPW table
extern const float VE_RPM_AXIS [VE_RPM_BINS];
extern const float VE_LOAD_AXIS[VE_LOAD_BINS];

// ── Input / output structures ─────────────────────────────────

typedef struct {
    float rpm;
    float map_kpa;          // KF-estimated MAP [kPa]
    float map_kpa_dot;      // dMAP/dt from KF [kPa/s]
    float bap_kpa;          // [UAV] KF-estimated BAP [kPa]
    float density_ratio;    // [UAV] σ = ρ/ρ₀
    float iat_degC;
    float cht_degC;
    float egt_degC;         // [UAV] KF-estimated EGT [°C]
    float tps_pct;
    float tps_dot;          // [%/s]
    float lambda_meas;
    bool  closed_loop;
    bool  map_sensor_valid;  // [UAV] false → activate FF_MAP_FIXED
    bool  iat_sensor_valid;  // [UAV] false → activate FF_IAT_FIXED
    bool  egt_sensor_valid;  // [UAV]
    uint16_t fault_flags;
} FuelInput_t;

typedef struct {
    float ve_pct;
    float afr_target;
    float air_mass_mg;
    float ipw_sd_us;        // Speed-Density path [µs]
    float ipw_an_us;        // Alpha-N path [µs]
    float alpha_n_blend;    // 0 = SD, 1 = AN
    float ipw_blended_us;   // SD/AN blend result
    float ipw_warmup_us;    // after cold-start enrichment
    float ipw_accel_us;     // after accel enrichment
    float ipw_egt_us;       // [UAV] after EGT enrichment
    float egt_enrich_pct;   // [UAV] EGT enrichment percent applied
    float lambda_trim;
    float ipw_final_us;
    FailMode_t fail_mode;
} FuelOutput_t;

void fuel_calc_init(void);
void fuel_calc_run (const FuelInput_t *in, FuelOutput_t *out);

float table_lookup_2d(const float table[VE_RPM_BINS][VE_LOAD_BINS],
                      float rpm, float load);
float table_lookup_an(const float table[VE_RPM_BINS][AN_TPS_BINS],
                      float rpm, float tps_pct);

#ifdef __cplusplus
}
#endif
