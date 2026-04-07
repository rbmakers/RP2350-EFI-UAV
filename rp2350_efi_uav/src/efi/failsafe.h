/**
 * @file    failsafe.h
 * @brief   UAV EFI Fail-Functional Safety Module
 *
 * Automotive vs UAV failure philosophy
 * ─────────────────────────────────────
 *
 *  Automotive (Limp-Home / Limp-Mode):
 *    A failed sensor triggers a reduced-performance "safe state".
 *    The car slows down and pulls over. Consequence: inconvenience.
 *
 *  UAV Fail-Functional:
 *    The engine MUST keep running at best achievable power.
 *    A failed sensor is substituted with a plausible fixed value.
 *    The vehicle lands safely under degraded (but functional) control.
 *    Consequence of engine stop: crash, loss of vehicle, injury.
 *
 * Sensor Substitution Table (used when a sensor is flagged invalid)
 * ──────────────────────────────────────────────────────────────────
 *
 *  Sensor  Substitution value    Rationale
 *  ──────────────────────────────────────────────────────────────
 *  MAP     BAP × 0.85            Assume WOT manifold depression ~85% of BAP
 *  BAP     101.325 kPa (ISA SL)  Assume sea level; over-fuels → safe
 *  IAT     25.0°C                Nominal ambient; conservative
 *  CLT     80.0°C                Assume warm; disables cold-start enrichment
 *  TPS     MAP-derived estimate  Estimate from MAP derivative
 *  O2/λ    1.0 (stoichiometric)  Disable closed-loop; open-loop operation
 *  EGT     EGT_WARN_DEGC         Conservative: apply warning enrichment
 *
 * Fault Flags (fault_flags bitmask in EcuState_t)
 * ─────────────────────────────────────────────────
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "../ecu_config.h"

// Fault flag bitmask definitions
#define FAULT_NONE          0x00
#define FAULT_MAP           (1 << 0)   // MAP sensor implausible
#define FAULT_BAP           (1 << 1)   // BAP sensor failed
#define FAULT_IAT           (1 << 2)   // IAT sensor out of range
#define FAULT_CLT           (1 << 3)   // CLT sensor out of range
#define FAULT_EGT           (1 << 4)   // EGT sensor fault or open circuit
#define FAULT_O2            (1 << 5)   // O2/lambda sensor implausible
#define FAULT_CKP           (1 << 6)   // Crank signal lost > 1 s
#define FAULT_ALTITUDE      (1 << 7)   // Density ratio below minimum

#ifdef __cplusplus
extern "C" {
#endif

// ── Sensor validity limits ────────────────────────────────────
// Values outside these bounds → sensor flagged invalid → substitution
typedef struct {
    float map_kpa_min,    map_kpa_max;     // [kPa]
    float bap_kpa_min,    bap_kpa_max;     // [kPa]
    float iat_degC_min,   iat_degC_max;    // [°C]
    float clt_degC_min,   clt_degC_max;    // [°C]
    float lambda_min,     lambda_max;      // [dimensionless]
    float rpm_max_cont;                    // max continuous RPM
    float rpm_max_abs;                     // absolute redline
} FailsafeLimits_t;

// Default limits for a UAV gasoline engine
extern const FailsafeLimits_t FAILSAFE_DEFAULTS;

// ── Sensor snapshot with validity flags ──────────────────────
typedef struct {
    float map_kpa;
    float bap_kpa;
    float iat_degC;
    float clt_degC;
    float tps_pct;
    float lambda;
    float egt_degC;
    float rpm;
    bool  ckp_sync;
    float density_ratio;
} RawSensors_t;

typedef struct {
    float map_kpa;      // validated or substituted
    float bap_kpa;      // validated or substituted
    float iat_degC;     // validated or substituted
    float clt_degC;     // validated or substituted
    float tps_pct;      // validated or substituted
    float lambda;       // validated or substituted
    float egt_degC;     // validated or substituted
    uint8_t fault_flags;
    bool  fail_functional;  // true if any substitution is active
} ValidatedSensors_t;

/**
 * Validate all sensor readings against physical limits.
 * Replace invalid readings with safe substitution values.
 * Set fault_flags bits for each failed channel.
 *
 * @param raw     raw sensor readings from drivers
 * @param limits  validity limits (pass &FAILSAFE_DEFAULTS normally)
 * @param out     output with validated/substituted values + fault flags
 */
void failsafe_validate(const RawSensors_t *raw,
                       const FailsafeLimits_t *limits,
                       ValidatedSensors_t *out);

/**
 * Check if an RPM limit override is needed due to EGT or altitude fault.
 * Returns the effective RPM ceiling (e.g. 80% of rated on EGT_CRITICAL).
 */
float failsafe_rpm_ceiling(uint8_t fault_flags, float egt_enrich_factor,
                           float density_ratio, float rated_rpm);

/**
 * Generate a human-readable fault string for serial/telemetry output.
 * @param fault_flags  bitmask from ValidatedSensors_t
 * @param buf          output buffer
 * @param buf_len      buffer length
 */
void failsafe_fault_string(uint8_t fault_flags, char *buf, int buf_len);

#ifdef __cplusplus
}
#endif


/* ─── IMPLEMENTATION ─────────────────────────────────────────── */
#ifdef FAILSAFE_IMPL

#include <string.h>
#include <stdio.h>
#include <math.h>

// Default sensor validity envelope for UAV gasoline engines
const FailsafeLimits_t FAILSAFE_DEFAULTS = {
    .map_kpa_min    =  10.0f, .map_kpa_max    = 120.0f,
    .bap_kpa_min    =  55.0f, .bap_kpa_max    = 108.0f,
    .iat_degC_min   = -40.0f, .iat_degC_max   = 100.0f,
    .clt_degC_min   = -40.0f, .clt_degC_max   = 130.0f,
    .lambda_min     =   0.6f, .lambda_max     =   1.8f,
    .rpm_max_cont   = 6500.0f,
    .rpm_max_abs    = 7200.0f,
};

// ── Substitution values ───────────────────────────────────────
static constexpr float SUB_IAT_DEGC = 25.0f;
static constexpr float SUB_CLT_DEGC = 80.0f;
static constexpr float SUB_LAMBDA   = 1.00f;
static constexpr float SUB_EGT_DEGC = EGT_WARN_DEGC;  // conservative
static constexpr float SUB_BAP_KPA  = ISA_P0_KPA;     // sea level

#define CHECK(field, min_val, max_val, flag, sub_val) \
    if ((raw->field) < (min_val) || (raw->field) > (max_val)) { \
        out->field = (sub_val); out->fault_flags |= (flag); \
        out->fail_functional = true; \
    } else { \
        out->field = raw->field; \
    }

void failsafe_validate(const RawSensors_t *raw,
                       const FailsafeLimits_t *limits,
                       ValidatedSensors_t *out) {
    out->fault_flags     = FAULT_NONE;
    out->fail_functional = false;

    // BAP checked first – used as MAP substitution base
    CHECK(bap_kpa, limits->bap_kpa_min, limits->bap_kpa_max,
          FAULT_BAP, SUB_BAP_KPA)

    // MAP: if invalid, substitute BAP × 0.85 (assumes near-WOT)
    float map_sub = out->bap_kpa * 0.85f;
    CHECK(map_kpa, limits->map_kpa_min, limits->map_kpa_max,
          FAULT_MAP, map_sub)

    // MAP vs BAP cross-check: MAP must be ≤ BAP (impossible otherwise)
    if (out->map_kpa > out->bap_kpa + 2.0f) {
        out->map_kpa = out->bap_kpa * 0.85f;
        out->fault_flags |= FAULT_MAP;
        out->fail_functional = true;
    }

    CHECK(iat_degC, limits->iat_degC_min, limits->iat_degC_max,
          FAULT_IAT, SUB_IAT_DEGC)
    CHECK(clt_degC, limits->clt_degC_min, limits->clt_degC_max,
          FAULT_CLT, SUB_CLT_DEGC)
    CHECK(lambda, limits->lambda_min, limits->lambda_max,
          FAULT_O2, SUB_LAMBDA)

    // TPS: pass-through (potentiometer fault detected separately by ADC range)
    out->tps_pct = raw->tps_pct;
    if (out->tps_pct < 0.0f || out->tps_pct > 105.0f) {
        out->tps_pct = 50.0f;   // mid-throttle assumption on TPS fault
    }

    // EGT: invalid → substitute warning value (conservative enrichment)
    if (!isfinite(raw->egt_degC) || raw->egt_degC < -50.0f ||
        raw->egt_degC > EGT_INVALID_DEGC) {
        out->egt_degC = SUB_EGT_DEGC;
        out->fault_flags |= FAULT_EGT;
        out->fail_functional = true;
    } else {
        out->egt_degC = raw->egt_degC;
    }

    // CKP sync lost
    if (!raw->ckp_sync) {
        out->fault_flags |= FAULT_CKP;
    }

    // Density ratio too low → altitude ceiling exceeded
    if (raw->density_ratio < UAV_DENSITY_RATIO_MIN) {
        out->fault_flags |= FAULT_ALTITUDE;
    }
}

float failsafe_rpm_ceiling(uint8_t fault_flags, float egt_enrich_factor,
                           float density_ratio, float rated_rpm) {
    float ceiling = rated_rpm;
    // EGT critical → limit to 80% RPM
    if (egt_enrich_factor >= 1.20f) ceiling = rated_rpm * 0.80f;
    // Altitude limit → reduce proportionally to density ratio
    if (fault_flags & FAULT_ALTITUDE) {
        float alt_factor = density_ratio / UAV_DENSITY_RATIO_MIN;
        if (alt_factor < 0.5f) alt_factor = 0.5f;
        ceiling = fminf(ceiling, rated_rpm * alt_factor);
    }
    return ceiling;
}

void failsafe_fault_string(uint8_t f, char *buf, int buf_len) {
    buf[0] = '\0';
    if (f == FAULT_NONE) { snprintf(buf, buf_len, "OK"); return; }
    if (f & FAULT_MAP)      strncat(buf, "MAP ", buf_len-1);
    if (f & FAULT_BAP)      strncat(buf, "BAP ", buf_len-1);
    if (f & FAULT_IAT)      strncat(buf, "IAT ", buf_len-1);
    if (f & FAULT_CLT)      strncat(buf, "CLT ", buf_len-1);
    if (f & FAULT_EGT)      strncat(buf, "EGT ", buf_len-1);
    if (f & FAULT_O2)       strncat(buf, "O2 ",  buf_len-1);
    if (f & FAULT_CKP)      strncat(buf, "CKP ", buf_len-1);
    if (f & FAULT_ALTITUDE) strncat(buf, "ALT ", buf_len-1);
}

#endif /* FAILSAFE_IMPL */
