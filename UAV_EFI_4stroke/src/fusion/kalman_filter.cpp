/**
 * @file    kalman_filter.cpp
 * @brief   UAV-EFI Kalman Filter implementations
 *
 * KF #1 – MAP + BAP fused (3-state constant-velocity + baro model)
 * ─────────────────────────────────────────────────────────────────
 *
 *  Transition matrix F (dt in seconds):
 *
 *    F = | 1  dt  0 |    P_man  evolves with velocity
 *        | 0   1  0 |    dP/dt  nearly constant
 *        | 0   0  1 |    P_baro nearly constant (changes 0.12 kPa/m)
 *
 *  Measurement matrix H (2 measurements: MAP ADC, BAP sensor):
 *
 *    H = | 1  0  0 |    z1 = P_man  (MAP ADC measures manifold)
 *        | 0  0  1 |    z2 = P_baro (MS5611 measures ambient)
 *
 *  Predict: x_pred = F·x;   P_pred = F·P·Fᵀ + Q
 *  Update:  y = z - H·x;    S = H·P·Hᵀ + R;   K = P·Hᵀ·S⁻¹
 *           x = x + K·y;    P = (I-K·H)·P
 *
 *  For a 3-state / 2-measurement KF, S is 2×2 (invertible analytically).
 *  We use the explicit 2×2 inverse to avoid a general matrix library.
 */

#include "kalman_filter.h"
#include "../ecu_config.h"

#include "hardware/timer.h"
#include <math.h>
#include <string.h>

static constexpr float R_AIR = 287.05f;  // J/(kg·K)

// ─────────────────────────────────────────────────────────────
//  KF #1 – MAP + BAP fused  (3-state)
// ─────────────────────────────────────────────────────────────

// P[i][j] accessors using flat row-major array
#define P3(kf,r,c)  ((kf)->P[(r)*3+(c)])

void map_bap_kf_init(MapBapKalman_t *kf,
                     float init_map_kpa, float init_bap_kpa) {
    kf->x[0] = init_map_kpa;
    kf->x[1] = 0.0f;           // assume zero rate initially
    kf->x[2] = init_bap_kpa;

    // Initial covariance (uncertain)
    memset(kf->P, 0, sizeof(kf->P));
    P3(kf,0,0) = 100.0f;   // var P_man
    P3(kf,1,1) =  50.0f;   // var dP/dt
    P3(kf,2,2) =  25.0f;   // var P_baro  (MS5611 is quite accurate)

    // Process noise (diagonal Q)
    kf->Q[0] = 0.2f;    // MAP changes up to ~2 kPa/s² (throttle transients)
    kf->Q[1] = 2.0f;    // dP/dt changes up to ~20 kPa/s² (snap throttle)
    kf->Q[2] = 0.001f;  // BAP changes ~0.0012 kPa/s at 10 m/s climb

    kf->R_map   = 4.0f;     // MAP ADC RMS noise ~2 kPa → R = 4
    kf->R_bap   = 0.09f;    // MS5611 RMS noise ~0.3 kPa → R = 0.09
    kf->last_us = time_us_64();
    kf->init    = true;
    kf->bap_valid = (init_bap_kpa > 10.0f);
}

void map_bap_kf_update(MapBapKalman_t *kf,
                        float map_meas_kpa,
                        float bap_meas_kpa,
                        uint64_t now_us) {
    if (!kf->init) {
        map_bap_kf_init(kf, map_meas_kpa,
                        (bap_meas_kpa > 10.0f) ? bap_meas_kpa : ISA_P0_PA / 1000.0f);
        return;
    }

    float dt = (float)(now_us - kf->last_us) * 1.0e-6f;
    kf->last_us = now_us;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.0005f;

    bool map_ok = (!isnan(map_meas_kpa) && map_meas_kpa > 5.0f && map_meas_kpa < 200.0f);
    bool bap_ok = (!isnan(bap_meas_kpa) && bap_meas_kpa > 40.0f && bap_meas_kpa < 110.0f);
    kf->bap_valid = bap_ok;

    // ── Predict ────────────────────────────────────────────────
    float xp0 = kf->x[0] + kf->x[1] * dt;
    float xp1 = kf->x[1];
    float xp2 = kf->x[2];

    // P_pred = F·P·Fᵀ + Q  (with F = [[1,dt,0],[0,1,0],[0,0,1]])
    // Row 0:
    float pp00 = P3(kf,0,0) + dt*(P3(kf,1,0)+P3(kf,0,1)) + dt*dt*P3(kf,1,1) + kf->Q[0];
    float pp01 = P3(kf,0,1) + dt*P3(kf,1,1);
    float pp02 = P3(kf,0,2) + dt*P3(kf,1,2);
    // Row 1:
    float pp10 = P3(kf,1,0) + dt*P3(kf,1,1);
    float pp11 = P3(kf,1,1) + kf->Q[1];
    float pp12 = P3(kf,1,2);
    // Row 2:
    float pp20 = P3(kf,2,0) + dt*P3(kf,2,1);
    float pp21 = P3(kf,2,1);
    float pp22 = P3(kf,2,2) + kf->Q[2];

    if (!map_ok && !bap_ok) {
        // No measurements – just propagate prediction
        kf->x[0] = xp0; kf->x[1] = xp1; kf->x[2] = xp2;
        P3(kf,0,0)=pp00; P3(kf,0,1)=pp01; P3(kf,0,2)=pp02;
        P3(kf,1,0)=pp10; P3(kf,1,1)=pp11; P3(kf,1,2)=pp12;
        P3(kf,2,0)=pp20; P3(kf,2,1)=pp21; P3(kf,2,2)=pp22;
        if (kf->x[0] < 5.0f)   kf->x[0] = FF_MAP_KPA;
        return;
    }

    // ── Update (scalar updates applied sequentially for simplicity) ──
    // Sequential scalar updates are mathematically equivalent to
    // a joint update when measurements are independent.

    float x0=xp0, x1=xp1, x2=xp2;
    float P00=pp00,P01=pp01,P02=pp02;
    float P10=pp10,P11=pp11,P12=pp12;
    float P20=pp20,P21=pp21,P22=pp22;

    // --- Update with MAP (z1 = x[0], H1 = [1,0,0]) ---
    if (map_ok) {
        float S1 = P00 + kf->R_map;
        float K0 = P00/S1, K1 = P10/S1, K2 = P20/S1;
        float y1 = map_meas_kpa - x0;
        x0 += K0*y1; x1 += K1*y1; x2 += K2*y1;
        float p00n = (1-K0)*P00; float p01n = (1-K0)*P01; float p02n = (1-K0)*P02;
        float p10n = P10 - K1*P00; float p11n = P11 - K1*P01; float p12n = P12 - K1*P02;
        float p20n = P20 - K2*P00; float p21n = P21 - K2*P01; float p22n = P22 - K2*P02;
        P00=p00n;P01=p01n;P02=p02n;P10=p10n;P11=p11n;P12=p12n;P20=p20n;P21=p21n;P22=p22n;
    }

    // --- Update with BAP (z2 = x[2], H2 = [0,0,1]) ---
    if (bap_ok) {
        float S2 = P22 + kf->R_bap;
        float K0b = P02/S2, K1b = P12/S2, K2b = P22/S2;
        float y2 = bap_meas_kpa - x2;
        x0 += K0b*y2; x1 += K1b*y2; x2 += K2b*y2;
        P00-=K0b*P20; P01-=K0b*P21; P02-=K0b*P22;
        P10-=K1b*P20; P11-=K1b*P21; P12-=K1b*P22;
        P20-=K2b*P20; P21-=K2b*P21; P22-=K2b*P22;
    }

    kf->x[0]=x0; kf->x[1]=x1; kf->x[2]=x2;
    P3(kf,0,0)=P00; P3(kf,0,1)=P01; P3(kf,0,2)=P02;
    P3(kf,1,0)=P10; P3(kf,1,1)=P11; P3(kf,1,2)=P12;
    P3(kf,2,0)=P20; P3(kf,2,1)=P21; P3(kf,2,2)=P22;

    // Physical limits
    if (kf->x[0] < 5.0f)   kf->x[0] = 5.0f;
    if (kf->x[0] > 200.0f) kf->x[0] = 200.0f;
    if (kf->x[2] < 40.0f)  kf->x[2] = 40.0f;
    if (kf->x[2] > 106.0f) kf->x[2] = 106.0f;
}

// ─────────────────────────────────────────────────────────────
//  KF #2 – IAT  (1-state)
// ─────────────────────────────────────────────────────────────

void iat_kf_init(IatKalman_t *kf, float init_degC) {
    kf->x = init_degC;
    kf->P = 100.0f;
    kf->Q = 0.01f;
    kf->R = 4.0f;     // NTC noise ~2°C RMS
    kf->init = true;
}

void iat_kf_update(IatKalman_t *kf, float measured_degC) {
    if (!kf->init) { iat_kf_init(kf, measured_degC); return; }
    float Pp = kf->P + kf->Q;
    float K  = Pp / (Pp + kf->R);
    kf->x = kf->x + K * (measured_degC - kf->x);
    kf->P = (1.0f - K) * Pp;
    if (kf->x < -40.0f) kf->x = -40.0f;
    if (kf->x > 120.0f) kf->x = 120.0f;
}

// ─────────────────────────────────────────────────────────────
//  KF #3 – EGT  (1-state)
// ─────────────────────────────────────────────────────────────

void egt_kf_init(EgtKalman_t *kf, float init_degC) {
    kf->x = (init_degC > -50.0f) ? init_degC : 20.0f;
    kf->P = 500.0f;
    kf->Q = 100.0f;   // EGT can swing 100°C/s on throttle changes
    kf->R = 9.0f;     // MAX31855 noise ~3°C RMS → R = 9
    kf->init = true;
}

void egt_kf_update(EgtKalman_t *kf, float measured_degC) {
    if (!kf->init) { egt_kf_init(kf, measured_degC); return; }
    if (measured_degC < EGT_SENSOR_FAIL_LO) return;  // sensor fault → no update
    float Pp = kf->P + kf->Q;
    float K  = Pp / (Pp + kf->R);
    kf->x = kf->x + K * (measured_degC - kf->x);
    kf->P = (1.0f - K) * Pp;
    if (kf->x <   0.0f) kf->x =   0.0f;
    if (kf->x > 1200.0f) kf->x = 1200.0f;
}

// ─────────────────────────────────────────────────────────────
//  Air mass  (altitude-corrected)
// ─────────────────────────────────────────────────────────────

float compute_air_mass_mg(float map_kpa, float iat_degC,
                           float ve_pct, float density_ratio,
                           bool map_valid) {
    float V_swept_m3 = (ENGINE_DISPLACEMENT_CC / ENGINE_CYL) * 1.0e-6f;

    if (map_valid) {
        // Full ideal-gas formula – P_man already encodes altitude effect
        float P_pa = map_kpa * 1000.0f;
        float T_K  = iat_degC + 273.15f;
        if (T_K < 200.0f) T_K = 200.0f;
        float rho  = P_pa / (R_AIR * T_K);
        return rho * V_swept_m3 * (ve_pct / 100.0f) * 1.0e6f;  // kg → mg
    } else {
        // Failsafe: scale sea-level full-load estimate by density_ratio
        // m_sea_ref ≈ ρ₀ × V_swept × VE% @ 100 kPa ISA sea level
        static constexpr float RHO0_KG_M3 = 1.225f;  // ISA sea-level density
        float m_ref = RHO0_KG_M3 * V_swept_m3 * (ve_pct / 100.0f) * 1.0e6f;
        return m_ref * density_ratio;
    }
}
