/**
 * @file    fuel_calc.cpp  [2-STROKE]
 * @brief   2-stroke fuel calculation
 *
 * Key differences from 4-stroke fuel_calc.cpp
 * ─────────────────────────────────────────────
 *  VE table  – pronounced hump at ~4000 RPM (exhaust pipe resonance);
 *              VE can exceed 100 % due to scavenging effect.
 *  EGT thresholds – lower (680 / 780 °C vs 750 / 850 °C).
 *  EGT enrichment – more aggressive (7 % per 50 °C vs 5 %).
 *  FF_IPW_FIXED_US – shorter (2000 µs vs 3500 µs, smaller engine).
 */

#include "efi/fuel_calc.h"
#include "ecu_config.h"
#include "fusion/kalman_filter.h"
#include <math.h>
#include <string.h>

/* ── Table axes ─────────────────────────────────────────────── */
const float VE_RPM_AXIS[VE_RPM_BINS] = {
     800, 1000, 1500, 2000, 2500, 3000, 3500, 4000,
    5000, 6000, 6500, 7000, 7500, 8000, 8500, 9000
};
const float VE_LOAD_AXIS[VE_LOAD_BINS] = {
    20, 25, 30, 35, 40, 50, 60, 70,
    75, 80, 85, 88, 90, 92, 96, 100
};
const float AN_TPS_AXIS[AN_TPS_BINS] = {
    0, 5, 10, 15, 20, 25, 30, 40,
    50, 60, 65, 70, 75, 80, 90, 100
};

/* ── 2-stroke VE table ──────────────────────────────────────── */
/* Values in percent. Hump at 4000 RPM (resonance peak row).
   VE > 100 % is possible due to exhaust pipe scavenging. */
const float VE_TABLE[VE_RPM_BINS][VE_LOAD_BINS] = {
    /* 800  */ { 45,47,48,50,52,54,56,57,58,58,59,59,60,60,61,61 },
    /* 1000 */ { 48,50,52,54,56,58,60,61,62,62,63,63,64,64,65,65 },
    /* 1500 */ { 52,54,56,58,60,62,64,65,66,66,67,67,68,68,69,69 },
    /* 2000 */ { 56,58,60,62,64,66,68,70,71,72,73,73,74,74,75,75 },
    /* 2500 */ { 62,64,66,68,70,72,74,76,77,78,79,79,80,80,81,81 },
    /* 3000 */ { 70,72,74,76,78,80,82,84,85,86,87,87,88,88,89,89 },
    /* 3500 */ { 80,82,84,86,88,90,92,94,95,96,97,97,98,98,99,99 },
    /* 4000 */ { 90,92,94,96,98,100,102,104,105,106,107,108,108,109,110,110 },
    /* 5000 */ { 82,84,86,88,90,92,94,96,97,98,99,100,100,101,102,102 },
    /* 6000 */ { 72,74,76,78,80,82,84,86,87,88,89,90,90,91,92,92 },
    /* 6500 */ { 66,68,70,72,74,76,78,80,81,82,83,84,84,85,86,86 },
    /* 7000 */ { 60,62,64,66,68,70,72,74,75,76,77,78,78,79,80,80 },
    /* 7500 */ { 54,56,58,60,62,64,66,68,69,70,71,72,72,73,74,74 },
    /* 8000 */ { 50,52,54,56,58,60,62,64,65,66,67,68,68,69,70,70 },
    /* 8500 */ { 46,48,50,52,54,56,58,60,61,62,63,64,64,65,66,66 },
    /* 9000 */ { 42,44,46,48,50,52,54,56,57,58,59,60,60,61,62,62 },
};

/* ── Target AFR table ───────────────────────────────────────── */
/* 2-strokes run slightly richer than 4-strokes for lubrication */
const float AFR_TABLE[VE_RPM_BINS][VE_LOAD_BINS] = {
    /* 800  */ {13.5f,13.5f,13.5f,13.5f,13.5f,13.5f,13.5f,13.5f,13.2f,13.0f,12.8f,12.5f,12.5f,12.2f,12.0f,11.8f},
    /* 1000 */ {13.5f,13.5f,13.5f,13.5f,13.5f,13.5f,13.5f,13.5f,13.2f,13.0f,12.8f,12.5f,12.5f,12.2f,12.0f,11.8f},
    /* 1500 */ {13.5f,13.5f,13.5f,13.5f,13.5f,13.5f,13.5f,13.5f,13.2f,13.0f,12.8f,12.5f,12.5f,12.2f,12.0f,11.8f},
    /* 2000 */ {13.5f,13.5f,13.5f,13.5f,13.5f,13.5f,13.5f,13.5f,13.2f,13.0f,12.8f,12.5f,12.5f,12.2f,12.0f,11.8f},
    /* 2500 */ {13.5f,13.5f,13.5f,13.5f,13.5f,13.5f,13.5f,13.2f,13.0f,12.8f,12.5f,12.2f,12.2f,12.0f,11.8f,11.6f},
    /* 3000 */ {13.5f,13.5f,13.5f,13.5f,13.5f,13.2f,13.0f,12.8f,12.5f,12.2f,12.0f,11.8f,11.8f,11.6f,11.5f,11.4f},
    /* 3500 */ {13.5f,13.5f,13.5f,13.2f,13.0f,12.8f,12.5f,12.2f,12.0f,11.8f,11.6f,11.5f,11.5f,11.4f,11.3f,11.2f},
    /* 4000 */ {13.2f,13.0f,12.8f,12.5f,12.2f,12.0f,11.8f,11.6f,11.5f,11.4f,11.3f,11.2f,11.2f,11.1f,11.0f,10.9f},
    /* 5000 */ {13.5f,13.2f,13.0f,12.8f,12.5f,12.2f,12.0f,11.8f,11.6f,11.5f,11.4f,11.3f,11.3f,11.2f,11.1f,11.0f},
    /* 6000 */ {13.5f,13.5f,13.2f,13.0f,12.8f,12.5f,12.2f,12.0f,11.8f,11.6f,11.5f,11.4f,11.4f,11.3f,11.2f,11.1f},
    /* 6500 */ {13.5f,13.5f,13.5f,13.2f,13.0f,12.8f,12.5f,12.2f,12.0f,11.8f,11.6f,11.5f,11.5f,11.4f,11.3f,11.2f},
    /* 7000 */ {13.5f,13.5f,13.5f,13.5f,13.2f,13.0f,12.8f,12.5f,12.2f,12.0f,11.8f,11.6f,11.6f,11.5f,11.4f,11.3f},
    /* 7500 */ {13.5f,13.5f,13.5f,13.5f,13.5f,13.2f,13.0f,12.8f,12.5f,12.2f,12.0f,11.8f,11.8f,11.6f,11.5f,11.4f},
    /* 8000 */ {13.5f,13.5f,13.5f,13.5f,13.5f,13.5f,13.2f,13.0f,12.8f,12.5f,12.2f,12.0f,12.0f,11.8f,11.6f,11.5f},
    /* 8500 */ {13.5f,13.5f,13.5f,13.5f,13.5f,13.5f,13.5f,13.2f,13.0f,12.8f,12.5f,12.2f,12.2f,12.0f,11.8f,11.6f},
    /* 9000 */ {13.5f,13.5f,13.5f,13.5f,13.5f,13.5f,13.5f,13.5f,13.2f,13.0f,12.8f,12.5f,12.5f,12.2f,12.0f,11.8f},
};

/* ── Alpha-N table (IPW µs indexed by RPM × TPS) ───────────── */
const float AN_TABLE[VE_RPM_BINS][AN_TPS_BINS] = {
    /* 800  */ { 300, 450, 600, 750, 900,1050,1200,1400,1600,1750,1850,1950,2050,2150,2350,2500},
    /* 1000 */ { 320, 480, 640, 800, 960,1120,1280,1480,1680,1830,1930,2030,2130,2230,2430,2580},
    /* 1500 */ { 360, 530, 700, 870,1040,1210,1380,1580,1780,1930,2030,2130,2230,2330,2530,2680},
    /* 2000 */ { 400, 580, 760, 940,1120,1300,1480,1680,1880,2030,2130,2230,2330,2430,2630,2780},
    /* 2500 */ { 450, 640, 830,1020,1210,1400,1590,1800,2000,2160,2260,2360,2460,2560,2760,2910},
    /* 3000 */ { 500, 700, 900,1100,1300,1500,1700,1900,2100,2260,2360,2460,2560,2660,2860,3010},
    /* 3500 */ { 560, 770, 980,1190,1400,1610,1820,2030,2240,2400,2500,2600,2700,2800,3000,3150},
    /* 4000 */ { 620, 840,1060,1280,1500,1720,1940,2160,2380,2540,2640,2740,2840,2940,3140,3290},
    /* 5000 */ { 580, 790,1000,1210,1420,1630,1840,2050,2260,2420,2520,2620,2720,2820,3020,3170},
    /* 6000 */ { 520, 720, 920,1120,1320,1520,1720,1920,2120,2280,2380,2480,2580,2680,2880,3030},
    /* 6500 */ { 480, 670, 860,1050,1240,1430,1620,1820,2020,2180,2280,2380,2480,2580,2780,2930},
    /* 7000 */ { 440, 620, 800, 980,1160,1340,1520,1720,1920,2080,2180,2280,2380,2480,2680,2830},
    /* 7500 */ { 400, 570, 740, 910,1080,1250,1420,1620,1820,1980,2080,2180,2280,2380,2580,2730},
    /* 8000 */ { 360, 520, 680, 840,1000,1160,1320,1520,1720,1880,1980,2080,2180,2280,2480,2630},
    /* 8500 */ { 320, 470, 620, 770, 920,1070,1220,1420,1620,1780,1880,1980,2080,2180,2380,2530},
    /* 9000 */ { 280, 420, 560, 700, 840, 980,1120,1320,1520,1680,1780,1880,1980,2080,2280,2430},
};

/* ── Injector capacity (mg/µs) ──────────────────────────────── */
static float s_inj_mg_per_us;

/* ── Closed-loop state ──────────────────────────────────────── */
static float s_lambda_int = 0.0f;
static constexpr float LAMBDA_KI  = 0.001f;
static constexpr float LAMBDA_MAX = 0.25f;

/* ── Accel enrichment state ─────────────────────────────────── */
static float s_accel_us = 0.0f;
static constexpr float ACCEL_DECAY = 0.85f;
static constexpr float ACCEL_GAIN  = 0.5f;

/* ── 2D bilinear lookup ─────────────────────────────────────── */
float table_lookup_2d(const float tbl[VE_RPM_BINS][VE_LOAD_BINS],
                      float rpm, float load) {
    int ri=0; for(int i=0;i<VE_RPM_BINS-1;i++) if(rpm>=VE_RPM_AXIS[i]) ri=i;
    int li=0; for(int i=0;i<VE_LOAD_BINS-1;i++) if(load>=VE_LOAD_AXIS[i]) li=i;
    if(ri>=VE_RPM_BINS-1) ri=VE_RPM_BINS-2;
    if(li>=VE_LOAD_BINS-1) li=VE_LOAD_BINS-2;
    float rf=(rpm -VE_RPM_AXIS[ri]) /(VE_RPM_AXIS[ri+1] -VE_RPM_AXIS[ri]);
    float lf=(load-VE_LOAD_AXIS[li])/(VE_LOAD_AXIS[li+1]-VE_LOAD_AXIS[li]);
    rf=rf<0?0:rf>1?1:rf; lf=lf<0?0:lf>1?1:lf;
    return tbl[ri][li]*(1-rf)*(1-lf)+tbl[ri+1][li]*rf*(1-lf)
          +tbl[ri][li+1]*(1-rf)*lf  +tbl[ri+1][li+1]*rf*lf;
}

float table_lookup_an(const float tbl[VE_RPM_BINS][AN_TPS_BINS],
                      float rpm, float tps) {
    int ri=0; for(int i=0;i<VE_RPM_BINS-1;i++) if(rpm>=VE_RPM_AXIS[i]) ri=i;
    int ti=0; for(int i=0;i<AN_TPS_BINS-1;i++) if(tps>=AN_TPS_AXIS[i]) ti=i;
    if(ri>=VE_RPM_BINS-1) ri=VE_RPM_BINS-2;
    if(ti>=AN_TPS_BINS-1) ti=AN_TPS_BINS-2;
    float rf=(rpm-VE_RPM_AXIS[ri])/(VE_RPM_AXIS[ri+1]-VE_RPM_AXIS[ri]);
    float tf=(tps-AN_TPS_AXIS[ti])/(AN_TPS_AXIS[ti+1]-AN_TPS_AXIS[ti]);
    rf=rf<0?0:rf>1?1:rf; tf=tf<0?0:tf>1?1:tf;
    return tbl[ri][ti]*(1-rf)*(1-tf)+tbl[ri+1][ti]*rf*(1-tf)
          +tbl[ri][ti+1]*(1-rf)*tf  +tbl[ri+1][ti+1]*rf*tf;
}

/* ── Warm-up factor (CHT-based, same for both strokes) ──────── */
static float warmup_factor(float cht) {
    if(cht>=80.0f) return 1.0f;
    if(cht<=-40.0f) return 2.0f;
    return 1.0f+(80.0f-cht)/120.0f;
}

/* ── Alpha-N blend (0=Speed-Density, 1=Alpha-N) ──────────────── */
static float an_blend(float rpm, float map_dot) {
    float rb = (rpm<=ALPHA_N_RPM_LOW)  ? 1.0f :
               (rpm>=ALPHA_N_RPM_HIGH) ? 0.0f :
               (ALPHA_N_RPM_HIGH-rpm)/(ALPHA_N_RPM_HIGH-ALPHA_N_RPM_LOW);
    float db = fabsf(map_dot)>20.0f ? 1.0f : fabsf(map_dot)/20.0f;
    return rb>db ? rb : db;
}

/* ── EGT enrichment ─────────────────────────────────────────── */
static float egt_factor(float egt, bool valid, float *pct) {
    *pct=0.0f;
    if(!valid || egt<EGT_WARN_DEGC) return 1.0f;
    float e=(egt-EGT_WARN_DEGC)/50.0f * EGT_ENRICH_PCT_PER_50C;
    if(e>35.0f) e=35.0f;
    *pct=e;
    return 1.0f+e/100.0f;
}

/* ── Init ───────────────────────────────────────────────────── */
void fuel_calc_init(void) {
    s_inj_mg_per_us = (INJ_FLOW_RATE_CC_MIN*740.0f)/(60.0f*1.0e6f);
    s_lambda_int=0.0f; s_accel_us=0.0f;
}

/* ── Main calculation ───────────────────────────────────────── */
void fuel_calc_run(const FuelInput_t *in, FuelOutput_t *out) {
    out->fail_mode = FF_NORMAL;

    /* Effective sensor values (with failsafe substitution) */
    float eff_map = in->map_kpa;
    float eff_iat = in->iat_degC;
    if(!in->map_sensor_valid) {
        eff_map = in->bap_kpa * 0.75f;
        out->fail_mode = FF_MAP_FIXED;
    }
    if(!in->iat_sensor_valid) {
        eff_iat = FF_IAT_DEGC;
        out->fail_mode = (out->fail_mode==FF_MAP_FIXED) ? FF_EMERGENCY : FF_IAT_FIXED;
    }

    /* 1. VE / AFR lookup */
    out->ve_pct     = table_lookup_2d(VE_TABLE,  in->rpm, eff_map);
    out->afr_target = table_lookup_2d(AFR_TABLE, in->rpm, eff_map);

    /* 2. Air mass */
    out->air_mass_mg = compute_air_mass_mg(eff_map, eff_iat,
                                           out->ve_pct, in->density_ratio,
                                           in->map_sensor_valid);

    /* 3. Speed-Density IPW */
    out->ipw_sd_us = (out->air_mass_mg / out->afr_target) / s_inj_mg_per_us;

    /* 4. Alpha-N IPW */
    out->ipw_an_us = table_lookup_an(AN_TABLE, in->rpm, in->tps_pct);

    /* 5. Blend */
    out->alpha_n_blend  = an_blend(in->rpm, in->map_kpa_dot);
    out->ipw_blended_us = out->alpha_n_blend * out->ipw_an_us
                        + (1.0f-out->alpha_n_blend) * out->ipw_sd_us;

    /* 6. Warm-up */
    out->ipw_warmup_us = out->ipw_blended_us * warmup_factor(in->cht_degC);

    /* 7. Accel */
    if(in->tps_dot>50.0f) {
        float na=in->tps_dot*ACCEL_GAIN;
        if(na>s_accel_us) s_accel_us=na;
    }
    out->ipw_accel_us = out->ipw_warmup_us + s_accel_us;
    s_accel_us *= ACCEL_DECAY;
    if(s_accel_us<0.5f) s_accel_us=0.0f;

    /* 8. EGT enrichment */
    float ef = egt_factor(in->egt_degC, in->egt_sensor_valid,
                          &out->egt_enrich_pct);
    out->ipw_egt_us = out->ipw_accel_us * ef;

    /* Update fail_mode for EGT */
    if(in->egt_sensor_valid) {
        if(in->egt_degC >= EGT_CRITICAL_DEGC) out->fail_mode=FF_EMERGENCY;
        else if(in->egt_degC>=EGT_WARN_DEGC && out->fail_mode==FF_NORMAL)
            out->fail_mode=FF_EGT_ENRICH;
    }

    /* 9. Closed-loop lambda trim */
    bool cl = in->closed_loop &&
              (out->fail_mode==FF_NORMAL||out->fail_mode==FF_IAT_FIXED);
    if(cl && in->lambda_meas>0.5f && in->lambda_meas<2.0f) {
        float err = (out->afr_target/STOICH_AFR) - in->lambda_meas;
        s_lambda_int += err*LAMBDA_KI;
        if(s_lambda_int> LAMBDA_MAX) s_lambda_int= LAMBDA_MAX;
        if(s_lambda_int<-LAMBDA_MAX) s_lambda_int=-LAMBDA_MAX;
    }
    out->lambda_trim = s_lambda_int;

    /* 10. Final IPW */
    float ipw = out->ipw_egt_us*(1.0f+out->lambda_trim)+INJ_DEAD_TIME_US;
    if(out->fail_mode==FF_EMERGENCY && !in->map_sensor_valid && !in->iat_sensor_valid)
        ipw = FF_IPW_FIXED_US+INJ_DEAD_TIME_US;
    if(ipw<0.0f) ipw=0.0f;
    if(ipw>15000.0f) ipw=15000.0f;
    out->ipw_final_us = ipw;
}
