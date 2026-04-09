/**
 * @file    kalman_filter.h
 * @brief   UAV-EFI Kalman Filters – MAP/BAP fusion, IAT, air mass
 *
 * UAV additions vs. automotive baseline
 * ─────────────────────────────────────
 *
 *  KF #1 – MAP + BAP fused pressure estimator (3-state)
 *  ┌──────────────────────────────────────────────────────────┐
 *  │ State: x = [ P_man, dP/dt, P_baro ]ᵀ                    │
 *  │                                                          │
 *  │  P_man  – manifold absolute pressure [kPa]               │
 *  │  dP/dt  – rate of change (accel-enrichment trigger) [kPa/s]│
 *  │  P_baro – barometric reference (slowly varying) [kPa]   │
 *  │                                                          │
 *  │ Measurement vector: z = [ P_map_adc, P_bap_ms5611 ]ᵀ    │
 *  │                                                          │
 *  │ The BAP state evolves as a near-constant (baro changes   │
 *  │ slowly with altitude). This lets the filter self-correct │
 *  │ the MAP reading using the altitude reference.            │
 *  │                                                          │
 *  │ MAP-relative = P_man - P_baro → true manifold depression │
 *  └──────────────────────────────────────────────────────────┘
 *
 *  KF #2 – IAT 1-state (unchanged from automotive)
 *
 *  KF #3 – EGT 1-state (new)
 *  ┌──────────────────────────────────────────────────────────┐
 *  │ EGT changes faster than ambient temperature but slower   │
 *  │ than MAP. A 1-state KF removes thermocouple amplifier    │
 *  │ noise (~3°C RMS) without adding meaningful lag.          │
 *  └──────────────────────────────────────────────────────────┘
 *
 *  Air mass computation (altitude-corrected)
 *  ─────────────────────────────────────────
 *    The automotive formula uses MAP only. For UAV we must
 *    account for reduced ambient density at altitude:
 *
 *      ρ_air  = P_man_pa / (R_air × T_air_K)          [kg/m³]
 *      m_air  = ρ_air × V_swept × VE%
 *
 *    Because P_man_pa already encodes the altitude-reduced
 *    intake pressure, no explicit density_ratio correction is
 *    needed IF P_man is measured correctly. However, when MAP
 *    sensor fails (FF_MAP_FIXED), we substitute BAP × partial-
 *    load factor, so the fallback must use density_ratio:
 *
 *      m_air_ff = σ × m_air_sea_level_estimate
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ─────────────────────────────────────────────────────────────
 *  KF #1 – MAP + BAP fused (3-state)
 * ───────────────────────────────────────────────────────────── */

typedef struct {
    float    x[3];      // [ P_man, dP/dt, P_baro ] in kPa / kPa/s / kPa
    float    P[9];      // 3×3 covariance (row-major)
    float    Q[3];      // diagonal process noise
    float    R_map;     // MAP measurement noise variance [kPa²]
    float    R_bap;     // BAP measurement noise variance [kPa²]
    uint64_t last_us;
    bool     init;
    bool     bap_valid; // false → treat BAP as unavailable, collapse to 2-state
} MapBapKalman_t;

void  map_bap_kf_init  (MapBapKalman_t *kf, float init_map_kpa, float init_bap_kpa);
void  map_bap_kf_update(MapBapKalman_t *kf,
                         float map_meas_kpa,    // ADC MAP reading (NaN if invalid)
                         float bap_meas_kpa,    // MS5611 BAP reading (NaN if invalid)
                         uint64_t now_us);

static inline float map_bap_kf_get_map  (const MapBapKalman_t *kf) { return kf->x[0]; }
static inline float map_bap_kf_get_rate (const MapBapKalman_t *kf) { return kf->x[1]; }
static inline float map_bap_kf_get_bap  (const MapBapKalman_t *kf) { return kf->x[2]; }
static inline float map_bap_kf_get_rel  (const MapBapKalman_t *kf) { return kf->x[0] - kf->x[2]; }


/* ─────────────────────────────────────────────────────────────
 *  KF #2 – IAT  (1-state, unchanged)
 * ───────────────────────────────────────────────────────────── */

typedef struct {
    float x;   // temperature estimate [°C]
    float P;   // error variance
    float Q;   // process noise
    float R;   // measurement noise
    bool  init;
} IatKalman_t;

void  iat_kf_init  (IatKalman_t *kf, float init_degC);
void  iat_kf_update(IatKalman_t *kf, float measured_degC);
static inline float iat_kf_get_temp(const IatKalman_t *kf) { return kf->x; }


/* ─────────────────────────────────────────────────────────────
 *  KF #3 – EGT  (1-state) [UAV NEW]
 * ───────────────────────────────────────────────────────────── */

typedef struct {
    float x;   // EGT estimate [°C]
    float P;
    float Q;   // larger Q than IAT – EGT can change 100°C/s
    float R;   // MAX31855 noise ~3°C
    bool  init;
} EgtKalman_t;

void  egt_kf_init  (EgtKalman_t *kf, float init_degC);
void  egt_kf_update(EgtKalman_t *kf, float measured_degC);
static inline float egt_kf_get_temp(const EgtKalman_t *kf) { return kf->x; }


/* ─────────────────────────────────────────────────────────────
 *  Air mass  (altitude-corrected)
 * ───────────────────────────────────────────────────────────── */

/**
 * Compute air mass per cylinder per intake event [mg].
 *
 * Nominal path (MAP sensor valid):
 *   m = (P_man_pa / (R_air × T_air_K)) × V_swept × VE%
 *
 * Failsafe path (MAP sensor failed, use BAP + density_ratio):
 *   m = density_ratio × VE% × m_sea_level_full_load
 *
 * @param map_kpa       KF-estimated MAP [kPa]
 * @param iat_degC      KF-estimated IAT [°C]
 * @param ve_pct        VE from lookup table [%]
 * @param density_ratio σ = ρ/ρ₀ (from compute_density_ratio)
 * @param map_valid     true = use full ideal-gas formula; false = failsafe
 */
float compute_air_mass_mg(float map_kpa, float iat_degC,
                           float ve_pct, float density_ratio,
                           bool map_valid);

#ifdef __cplusplus
}
#endif
