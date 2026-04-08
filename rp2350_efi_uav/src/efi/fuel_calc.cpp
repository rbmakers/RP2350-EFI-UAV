/**
 *
 * 火箭鳥創客倉庫 製作
 *
 * @file    fuel_calc.cpp
 * @brief   UAV-EFI fuel calculation – Alpha-N blend, EGT enrichment,
 *          fail-functional, altitude-corrected Speed-Density
 */

#include "fuel_calc.h"
#include "../ecu_config.h"
#include "../fusion/kalman_filter.h"
#include <math.h>
#include <string.h>

// ── RPM axis (shared for VE, AFR, Alpha-N) ──────────────────
const float VE_RPM_AXIS[VE_RPM_BINS] = {
     800, 1000, 1200, 1500, 1800, 2200, 2600, 3000,
    3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000
};

// MAP load axis [kPa] – now extends below atmospheric for UAV altitude ops
const float VE_LOAD_AXIS[VE_LOAD_BINS] = {
    20, 25, 30, 35, 40, 50, 60, 70,
    75, 80, 85, 88, 90, 92, 96, 100
};

// [UAV] Alpha-N TPS axis [%]
const float AN_TPS_AXIS[AN_TPS_BINS] = {
    0, 5, 10, 15, 20, 25, 30, 40,
    50, 60, 65, 70, 75, 80, 90, 100
};

// VE table – same as automotive (tuned for specific engine)
const float VE_TABLE[VE_RPM_BINS][VE_LOAD_BINS] = {
    /* 800  */ { 58,60,62,64,66,68,70,72,73,74,75,76,77,78,79,80 },
    /* 1000 */ { 60,62,64,66,68,70,72,74,75,76,77,78,79,80,81,82 },
    /* 1200 */ { 62,64,66,68,70,72,74,76,77,78,79,80,81,82,83,84 },
    /* 1500 */ { 64,66,68,70,72,74,76,78,79,80,81,82,83,84,85,86 },
    /* 1800 */ { 65,67,69,71,73,75,77,79,80,81,82,83,84,85,86,87 },
    /* 2200 */ { 66,68,70,72,74,76,78,80,81,82,83,84,85,86,87,88 },
    /* 2600 */ { 67,69,71,73,75,77,79,81,82,83,84,85,86,87,88,89 },
    /* 3000 */ { 68,70,72,74,76,78,80,82,83,84,85,86,87,88,89,90 },
    /* 3500 */ { 69,71,73,75,77,79,81,83,84,85,86,87,88,89,90,91 },
    /* 4000 */ { 70,72,74,76,78,80,82,84,85,86,87,88,89,90,91,92 },
    /* 4500 */ { 69,71,73,75,77,79,81,83,84,85,86,87,88,89,90,91 },
    /* 5000 */ { 68,70,72,74,76,78,80,82,83,84,85,86,87,88,89,90 },
    /* 5500 */ { 66,68,70,72,74,76,78,80,81,82,83,84,85,86,87,88 },
    /* 6000 */ { 64,66,68,70,72,74,76,78,79,80,81,82,83,84,85,86 },
    /* 6500 */ { 62,64,66,68,70,72,74,76,77,78,79,80,81,82,83,84 },
    /* 7000 */ { 60,62,64,66,68,70,72,74,75,76,77,78,79,80,81,82 },
};

// Target AFR table
const float AFR_TABLE[VE_RPM_BINS][VE_LOAD_BINS] = {
    /* 800  */{14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.5f,14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.5f},
    /* 1000 */{14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.5f,14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.5f},
    /* 1200 */{14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.5f,14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.5f},
    /* 1500 */{14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.5f,14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.5f},
    /* 1800 */{14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.5f,14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.5f},
    /* 2200 */{14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.5f,14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.5f},
    /* 2600 */{14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.5f,14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.5f},
    /* 3000 */{14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.7f,14.5f,14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.5f,12.2f},
    /* 3500 */{14.7f,14.7f,14.7f,14.7f,14.7f,14.5f,14.2f,14.0f,13.8f,13.5f,13.2f,13.0f,12.8f,12.6f,12.4f,12.2f},
    /* 4000 */{14.7f,14.7f,14.7f,14.7f,14.5f,14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.6f,12.4f,12.2f,12.1f,12.0f},
    /* 4500 */{14.7f,14.7f,14.7f,14.5f,14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.6f,12.4f,12.2f,12.1f,12.0f,11.9f},
    /* 5000 */{14.7f,14.7f,14.5f,14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.6f,12.4f,12.2f,12.1f,12.0f,11.9f,11.8f},
    /* 5500 */{14.7f,14.5f,14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.6f,12.4f,12.2f,12.1f,12.0f,11.9f,11.8f,11.7f},
    /* 6000 */{14.5f,14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.6f,12.4f,12.2f,12.1f,12.0f,11.9f,11.8f,11.7f,11.6f},
    /* 6500 */{14.2f,13.8f,13.5f,13.2f,13.0f,12.8f,12.6f,12.4f,12.2f,12.1f,12.0f,11.9f,11.8f,11.7f,11.6f,11.5f},
    /* 7000 */{13.8f,13.5f,13.2f,13.0f,12.8f,12.6f,12.4f,12.2f,12.1f,12.0f,11.9f,11.8f,11.7f,11.6f,11.5f,11.4f},
};

// [UAV] Alpha-N table: IPW in µs directly indexed by RPM × TPS%
// Tuned to deliver correct fuelling independent of MAP pressure
// (critical at altitude where MAP may not reflect load correctly)
const float AN_TABLE[VE_RPM_BINS][AN_TPS_BINS] = {
    /* 800  */ { 500, 700, 900,1100,1300,1500,1700,1900,2100,2200,2300,2400,2500,2600,2800,3000},
    /* 1000 */ { 550, 750, 950,1150,1350,1550,1750,1950,2150,2250,2350,2450,2550,2650,2850,3050},
    /* 1200 */ { 600, 800,1000,1200,1400,1600,1800,2000,2200,2300,2400,2500,2600,2700,2900,3100},
    /* 1500 */ { 650, 850,1050,1250,1450,1650,1850,2050,2250,2350,2450,2550,2650,2750,2950,3150},
    /* 1800 */ { 700, 900,1100,1300,1500,1700,1900,2100,2300,2400,2500,2600,2700,2800,3000,3200},
    /* 2200 */ { 750, 950,1150,1350,1550,1750,1950,2150,2350,2450,2550,2650,2750,2850,3050,3250},
    /* 2600 */ { 800,1000,1200,1400,1600,1800,2000,2200,2400,2500,2600,2700,2800,2900,3100,3300},
    /* 3000 */ { 850,1050,1250,1450,1650,1850,2050,2250,2450,2550,2650,2750,2850,2950,3150,3350},
    /* 3500 */ { 900,1100,1300,1500,1700,1900,2100,2300,2500,2600,2700,2800,2900,3000,3200,3400},
    /* 4000 */ { 850,1050,1250,1450,1650,1850,2050,2250,2450,2550,2650,2750,2850,2950,3150,3350},
    /* 4500 */ { 800,1000,1200,1400,1600,1800,2000,2200,2400,2500,2600,2700,2800,2900,3100,3300},
    /* 5000 */ { 750, 950,1150,1350,1550,1750,1950,2150,2350,2450,2550,2650,2750,2850,3050,3250},
    /* 5500 */ { 700, 900,1100,1300,1500,1700,1900,2100,2300,2400,2500,2600,2700,2800,3000,3200},
    /* 6000 */ { 650, 850,1050,1250,1450,1650,1850,2050,2250,2350,2450,2550,2650,2750,2950,3150},
    /* 6500 */ { 600, 800,1000,1200,1400,1600,1800,2000,2200,2300,2400,2500,2600,2700,2900,3100},
    /* 7000 */ { 550, 750, 950,1150,1350,1550,1750,1950,2150,2250,2350,2450,2550,2650,2850,3050},
};

// ── Injector capacity ─────────────────────────────────────────
static float s_inj_mg_per_us;

// ── Closed-loop state ─────────────────────────────────────────
static float s_lambda_integrator = 0.0f;
static constexpr float LAMBDA_KI  = 0.001f;
static constexpr float LAMBDA_MAX = 0.25f;

// ── Accel enrichment state ────────────────────────────────────
static float s_accel_decay_us = 0.0f;
static constexpr float ACCEL_DECAY   = 0.85f;
static constexpr float ACCEL_GAIN    = 0.5f;    // µs per (%/s)

// ── Table interpolation ──────────────────────────────────────

float table_lookup_2d(const float table[VE_RPM_BINS][VE_LOAD_BINS],
                      float rpm, float load) {
    int ri = 0;
    for (int i = 0; i < VE_RPM_BINS-1; i++) if (rpm  >= VE_RPM_AXIS[i])  ri = i;
    int li = 0;
    for (int i = 0; i < VE_LOAD_BINS-1; i++) if (load >= VE_LOAD_AXIS[i]) li = i;
    if (ri >= VE_RPM_BINS-1)  ri = VE_RPM_BINS-2;
    if (li >= VE_LOAD_BINS-1) li = VE_LOAD_BINS-2;

    float rf = (rpm  - VE_RPM_AXIS[ri])  / (VE_RPM_AXIS[ri+1]  - VE_RPM_AXIS[ri]);
    float lf = (load - VE_LOAD_AXIS[li]) / (VE_LOAD_AXIS[li+1] - VE_LOAD_AXIS[li]);
    rf = rf < 0.0f ? 0.0f : (rf > 1.0f ? 1.0f : rf);
    lf = lf < 0.0f ? 0.0f : (lf > 1.0f ? 1.0f : lf);

    return table[ri][li]*(1-rf)*(1-lf) + table[ri+1][li]*rf*(1-lf)
         + table[ri][li+1]*(1-rf)*lf   + table[ri+1][li+1]*rf*lf;
}

float table_lookup_an(const float table[VE_RPM_BINS][AN_TPS_BINS],
                      float rpm, float tps_pct) {
    int ri = 0;
    for (int i = 0; i < VE_RPM_BINS-1; i++)  if (rpm     >= VE_RPM_AXIS[i])  ri = i;
    int ti = 0;
    for (int i = 0; i < AN_TPS_BINS-1; i++)  if (tps_pct >= AN_TPS_AXIS[i])  ti = i;
    if (ri >= VE_RPM_BINS-1)  ri = VE_RPM_BINS-2;
    if (ti >= AN_TPS_BINS-1)  ti = AN_TPS_BINS-2;

    float rf = (rpm     - VE_RPM_AXIS[ri])  / (VE_RPM_AXIS[ri+1]  - VE_RPM_AXIS[ri]);
    float tf = (tps_pct - AN_TPS_AXIS[ti])  / (AN_TPS_AXIS[ti+1]  - AN_TPS_AXIS[ti]);
    rf = rf < 0.0f ? 0.0f : (rf > 1.0f ? 1.0f : rf);
    tf = tf < 0.0f ? 0.0f : (tf > 1.0f ? 1.0f : tf);

    return table[ri][ti]*(1-rf)*(1-tf) + table[ri+1][ti]*rf*(1-tf)
         + table[ri][ti+1]*(1-rf)*tf   + table[ri+1][ti+1]*rf*tf;
}

// ── Warm-up factor ────────────────────────────────────────────
static float warmup_factor(float cht_degC) {
    if (cht_degC >= 80.0f) return 1.0f;
    if (cht_degC <= -40.0f) return 2.0f;
    return 1.0f + (80.0f - cht_degC) / 120.0f;
}

// ── Alpha-N blend factor ──────────────────────────────────────
// Returns 0 (full Speed-Density) to 1 (full Alpha-N)
// Blend toward Alpha-N when:
//   a) RPM below threshold (transient-heavy regime)
//   b) MAP changing rapidly (sensor lagging reality)
static float compute_an_blend(float rpm, float map_dot_kpa_s) {
    // RPM-based blend
    float rpm_blend;
    if (rpm <= ALPHA_N_RPM_LOW)  rpm_blend = 1.0f;
    else if (rpm >= ALPHA_N_RPM_HIGH) rpm_blend = 0.0f;
    else rpm_blend = (ALPHA_N_RPM_HIGH - rpm) / (ALPHA_N_RPM_HIGH - ALPHA_N_RPM_LOW);

    // MAP-dot blend: if |dP/dt| > 20 kPa/s → prefer Alpha-N
    float dot_abs = fabsf(map_dot_kpa_s);
    float dot_blend = (dot_abs > 20.0f) ? 1.0f : (dot_abs / 20.0f);

    // Take the maximum of the two blend signals
    return (rpm_blend > dot_blend) ? rpm_blend : dot_blend;
}

// ── EGT enrichment ────────────────────────────────────────────
// Returns extra fuel factor (0.0 = no extra)
static float egt_enrichment_factor(float egt_degC, bool egt_valid,
                                   float *enrich_pct_out) {
    *enrich_pct_out = 0.0f;
    if (!egt_valid || egt_degC < EGT_WARN_DEGC) return 1.0f;

    float excess  = egt_degC - EGT_WARN_DEGC;           // °C above warning
    float enrich  = (excess / 50.0f) * EGT_ENRICH_PCT_PER_50C;  // % extra
    if (enrich > 30.0f) enrich = 30.0f;                 // cap at 30 % extra
    *enrich_pct_out = enrich;
    return 1.0f + enrich / 100.0f;
}

// ── Init ──────────────────────────────────────────────────────
void fuel_calc_init(void) {
    s_inj_mg_per_us     = (INJ_FLOW_RATE_CC_MIN * 740.0f) / (60.0f * 1.0e6f);
    s_lambda_integrator = 0.0f;
    s_accel_decay_us    = 0.0f;
}

// ── Main fuel calculation ──────────────────────────────────────
void fuel_calc_run(const FuelInput_t *in, FuelOutput_t *out) {
    out->fail_mode = FF_NORMAL;

    // ── 1. Determine effective MAP and IAT (with failsafe) ────
    float effective_map_kpa = in->map_kpa;
    float effective_iat_degC = in->iat_degC;
    bool  map_valid = in->map_sensor_valid;

    if (!map_valid) {
        // MAP sensor failed → substitute BAP × partial-load factor
        // At altitude, BAP reflects ambient; a natural-aspirated engine
        // at mid-throttle sees ~70% of ambient as manifold depression.
        effective_map_kpa = in->bap_kpa * 0.75f;
        out->fail_mode = FF_MAP_FIXED;
    }
    if (!in->iat_sensor_valid) {
        effective_iat_degC = FF_IAT_DEGC;
        if (out->fail_mode == FF_MAP_FIXED) out->fail_mode = FF_EMERGENCY;
        else out->fail_mode = FF_IAT_FIXED;
    }

    // ── 2. VE / AFR lookup ────────────────────────────────────
    out->ve_pct     = table_lookup_2d(VE_TABLE,  in->rpm, effective_map_kpa);
    out->afr_target = table_lookup_2d(AFR_TABLE, in->rpm, effective_map_kpa);

    // ── 3. Speed-Density air mass ─────────────────────────────
    out->air_mass_mg = compute_air_mass_mg(effective_map_kpa, effective_iat_degC,
                                           out->ve_pct, in->density_ratio,
                                           map_valid);

    // ── 4. Speed-Density base IPW ─────────────────────────────
    float fuel_mg    = out->air_mass_mg / out->afr_target;
    out->ipw_sd_us   = fuel_mg / s_inj_mg_per_us;

    // ── 5. Alpha-N IPW  [UAV] ─────────────────────────────────
    out->ipw_an_us   = table_lookup_an(AN_TABLE, in->rpm, in->tps_pct);

    // ── 6. Alpha-N / Speed-Density blend  [UAV] ──────────────
    out->alpha_n_blend  = compute_an_blend(in->rpm, in->map_kpa_dot);
    float blend         = out->alpha_n_blend;
    out->ipw_blended_us = blend * out->ipw_an_us + (1.0f - blend) * out->ipw_sd_us;

    // ── 7. Warm-up enrichment ─────────────────────────────────
    out->ipw_warmup_us  = out->ipw_blended_us * warmup_factor(in->cht_degC);

    // ── 8. Accel enrichment ───────────────────────────────────
    if (in->tps_dot > 50.0f) {
        float new_accel = in->tps_dot * ACCEL_GAIN;
        if (new_accel > s_accel_decay_us) s_accel_decay_us = new_accel;
    }
    out->ipw_accel_us = out->ipw_warmup_us + s_accel_decay_us;
    s_accel_decay_us *= ACCEL_DECAY;
    if (s_accel_decay_us < 0.5f) s_accel_decay_us = 0.0f;

    // ── 9. EGT enrichment  [UAV] ─────────────────────────────
    float egt_factor = egt_enrichment_factor(in->egt_degC, in->egt_sensor_valid,
                                              &out->egt_enrich_pct);
    out->ipw_egt_us  = out->ipw_accel_us * egt_factor;

    // Update fail_mode for EGT emergency
    if (in->egt_degC >= EGT_CRITICAL_DEGC && in->egt_sensor_valid) {
        out->fail_mode = FF_EMERGENCY;
    } else if (in->egt_degC >= EGT_WARN_DEGC && in->egt_sensor_valid
               && out->fail_mode == FF_NORMAL) {
        out->fail_mode = FF_EGT_ENRICH;
    }

    // ── 10. Closed-loop lambda trim ───────────────────────────
    // Disable closed-loop during EGT enrichment (open-loop EGT priority)
    bool cl_ok = in->closed_loop && (out->fail_mode == FF_NORMAL || out->fail_mode == FF_IAT_FIXED);
    if (cl_ok && in->lambda_meas > 0.5f && in->lambda_meas < 2.0f) {
        float lambda_target = out->afr_target / STOICH_AFR;
        float error = lambda_target - in->lambda_meas;
        s_lambda_integrator += error * LAMBDA_KI;
        if (s_lambda_integrator >  LAMBDA_MAX) s_lambda_integrator =  LAMBDA_MAX;
        if (s_lambda_integrator < -LAMBDA_MAX) s_lambda_integrator = -LAMBDA_MAX;
    }
    out->lambda_trim = s_lambda_integrator;

    // ── 11. Apply lambda trim + dead time ────────────────────
    float ipw = out->ipw_egt_us * (1.0f + out->lambda_trim) + INJ_DEAD_TIME_US;

    // Emergency override: if all sensors failed, use fixed safe IPW
    if (out->fail_mode == FF_EMERGENCY &&
        !in->map_sensor_valid && !in->iat_sensor_valid) {
        ipw = FF_IPW_FIXED_US + INJ_DEAD_TIME_US;
    }

    // ── 12. Final limits ──────────────────────────────────────
    if (ipw <     0.0f) ipw =     0.0f;
    if (ipw > 20000.0f) ipw = 20000.0f;
    out->ipw_final_us = ipw;
}
