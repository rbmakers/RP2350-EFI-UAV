/**
 * @file    fuel_calc.h  [4-STROKE]
 * @brief   Fuel calculation: Speed-Density + Alpha-N blend, enrichments,
 *          EGT protection, closed-loop O2 trim, fail-functional fallbacks.
 *
 * IPW cascade:
 *   base (SD or AN blend) → warm-up × → accel + → EGT × → lambda × → + dead_time
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "ecu_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VE_RPM_BINS   16
#define VE_LOAD_BINS  16
#define AN_TPS_BINS   16

extern const float VE_RPM_AXIS [VE_RPM_BINS];
extern const float VE_LOAD_AXIS[VE_LOAD_BINS];
extern const float AN_TPS_AXIS [AN_TPS_BINS];
extern const float VE_TABLE [VE_RPM_BINS][VE_LOAD_BINS];
extern const float AFR_TABLE[VE_RPM_BINS][VE_LOAD_BINS];
extern const float AN_TABLE [VE_RPM_BINS][AN_TPS_BINS];

typedef struct {
    float  rpm;
    float  map_kpa;
    float  map_kpa_dot;
    float  bap_kpa;
    float  density_ratio;
    float  iat_degC;
    float  cht_degC;
    float  egt_degC;
    float  tps_pct;
    float  tps_dot;
    float  lambda_meas;
    bool   closed_loop;
    bool   map_sensor_valid;
    bool   iat_sensor_valid;
    bool   egt_sensor_valid;
    uint16_t fault_flags;
} FuelInput_t;

typedef struct {
    float  ve_pct;
    float  afr_target;
    float  air_mass_mg;
    float  ipw_sd_us;
    float  ipw_an_us;
    float  alpha_n_blend;
    float  ipw_blended_us;
    float  ipw_warmup_us;
    float  ipw_accel_us;
    float  ipw_egt_us;
    float  egt_enrich_pct;
    float  lambda_trim;
    float  ipw_final_us;
    FailMode_t fail_mode;
} FuelOutput_t;

void  fuel_calc_init(void);
void  fuel_calc_run (const FuelInput_t *in, FuelOutput_t *out);
float table_lookup_2d(const float table[VE_RPM_BINS][VE_LOAD_BINS],
                      float rpm, float load);
float table_lookup_an(const float table[VE_RPM_BINS][AN_TPS_BINS],
                      float rpm, float tps_pct);

#ifdef __cplusplus
}
#endif
