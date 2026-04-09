/**
 * @file    ignition.cpp  [4-STROKE]
 * @brief   4-stroke ignition: fires every 720°, advance table, hardware alarm
 *
 * Scheduling:
 *   Every control tick, compute charge_deg and fire_deg per cylinder.
 *   When current crank angle is within 5° of charge_deg, arm a hardware
 *   alarm to assert the coil GPIO at the exact microsecond.
 *   When coil is charged and within 5° of fire_deg, arm fire alarm.
 *
 * Flat-twin TDC reference (0-719°):
 *   Cyl 1: 0°      Cyl 2: 360°
 */

#include "ignition.h"
#include "../ecu_config.h"
#include "../drivers/crank_sensor.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"
#include <math.h>

/* ── Advance table ──────────────────────────────────────────── */
const float IGN_RPM_AXIS[IGN_RPM_BINS] = {
     800, 1000, 1200, 1500, 1800, 2200, 2600, 3000,
    3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000
};
/* MAP-relative axis: 0 (idle vacuum) → 50 kPa (near WOT) */
const float IGN_MAP_AXIS[IGN_MAP_BINS] = {
    0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 48, 50
};
/* degrees BTDC – high vacuum → more advance; high load → less */
const float IGN_ADVANCE_TABLE[IGN_RPM_BINS][IGN_MAP_BINS] = {
    /* 800  */ { 8, 8, 7, 7, 6, 6, 5, 5, 5, 4, 4, 4 },
    /* 1000 */ {10,10, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5 },
    /* 1200 */ {12,12,11,11,10,10, 9, 9, 8, 7, 6, 6 },
    /* 1500 */ {15,14,13,12,12,11,10,10, 9, 8, 7, 7 },
    /* 1800 */ {18,17,16,15,14,13,12,11,10, 9, 8, 8 },
    /* 2200 */ {22,21,20,18,17,16,15,14,13,12,10,10 },
    /* 2600 */ {25,24,23,21,20,18,17,16,15,13,12,11 },
    /* 3000 */ {28,27,25,23,22,20,19,17,16,14,13,12 },
    /* 3500 */ {30,29,27,25,23,22,20,19,17,16,14,13 },
    /* 4000 */ {32,30,28,26,24,23,21,20,18,17,15,14 },
    /* 4500 */ {33,31,29,27,25,24,22,21,19,17,16,15 },
    /* 5000 */ {34,32,30,28,26,25,23,22,20,18,17,16 },
    /* 5500 */ {34,32,30,28,26,25,23,22,20,18,17,16 },
    /* 6000 */ {33,31,29,27,25,24,22,21,19,18,16,15 },
    /* 6500 */ {32,30,28,26,24,23,21,20,18,17,15,14 },
    /* 7000 */ {30,28,26,24,22,21,19,18,17,16,14,13 },
};

/* ── Cylinder layout ────────────────────────────────────────── */
static const uint16_t CYL_TDC_DEG[2] = { 0, 360 };  // 4-stroke: 0°/360°
static const uint      CYL_GPIO[2]    = { PIN_IGN_1, PIN_IGN_2 };

/* ── State ──────────────────────────────────────────────────── */
static CylIgnState_t   s_cyl[2];
static float           s_advance_deg = 10.0f;
static int             s_alarm_hw    = -1;

typedef enum { ALARM_NONE, ALARM_CHARGE, ALARM_FIRE } AlarmAction_t;
static volatile AlarmAction_t s_pending_action = ALARM_NONE;
static volatile uint          s_pending_cyl    = 0;

/* ── Helpers ────────────────────────────────────────────────── */
static float lookup_advance(float rpm, float map_rel) {
    if (map_rel < 0.0f)  map_rel = 0.0f;
    if (map_rel > 50.0f) map_rel = 50.0f;
    int ri = 0;
    for (int i = 0; i < IGN_RPM_BINS-1; i++) if (rpm >= IGN_RPM_AXIS[i]) ri=i;
    int mi = 0;
    for (int i = 0; i < IGN_MAP_BINS-1; i++) if (map_rel >= IGN_MAP_AXIS[i]) mi=i;
    if (ri >= IGN_RPM_BINS-1) ri = IGN_RPM_BINS-2;
    if (mi >= IGN_MAP_BINS-1) mi = IGN_MAP_BINS-2;
    float rf = (rpm    - IGN_RPM_AXIS[ri]) / (IGN_RPM_AXIS[ri+1]-IGN_RPM_AXIS[ri]);
    float mf = (map_rel-IGN_MAP_AXIS[mi]) / (IGN_MAP_AXIS[mi+1]-IGN_MAP_AXIS[mi]);
    rf = rf<0?0:rf>1?1:rf; mf = mf<0?0:mf>1?1:mf;
    float v = IGN_ADVANCE_TABLE[ri][mi]    *(1-rf)*(1-mf)
            + IGN_ADVANCE_TABLE[ri+1][mi]  *rf*(1-mf)
            + IGN_ADVANCE_TABLE[ri][mi+1]  *(1-rf)*mf
            + IGN_ADVANCE_TABLE[ri+1][mi+1]*rf*mf;
    if (v < IGN_MIN_ADVANCE_DEG) v = IGN_MIN_ADVANCE_DEG;
    if (v > IGN_MAX_ADVANCE_DEG) v = IGN_MAX_ADVANCE_DEG;
    return v;
}

/* angle_deg → microseconds (4-stroke: full revolution = 360° per 60/RPM s) */
static uint32_t angle_to_us(float angle_deg, float rpm) {
    if (rpm < 100.0f) return 0;
    return (uint32_t)((angle_deg / 360.0f) * (60.0e6f / rpm));
}

/* Wrap into [0, 720) for 4-stroke */
static inline uint16_t wrap720(int32_t a) {
    a %= 720; if (a < 0) a += 720; return (uint16_t)a;
}

/* ── Hardware alarm ISR ─────────────────────────────────────── */
static void __isr __not_in_flash_func(ign_alarm_isr)(void) {
    hardware_alarm_cancel(s_alarm_hw);
    uint c = s_pending_cyl;
    AlarmAction_t act = s_pending_action;
    s_pending_action = ALARM_NONE;
    if (act == ALARM_CHARGE) {
        gpio_put(CYL_GPIO[c], 1);
        s_cyl[c].coil_charging = true;
    } else if (act == ALARM_FIRE) {
        gpio_put(CYL_GPIO[c], 0);
        s_cyl[c].coil_charging = false;
        s_cyl[c].armed = false;
    }
}

static void sched_alarm(uint32_t delay_us, AlarmAction_t act, uint cyl) {
    if (delay_us == 0 || delay_us > 200000) return;
    s_pending_action = act;
    s_pending_cyl    = cyl;
    hardware_alarm_set_target(s_alarm_hw, time_us_64() + delay_us);
}

/* ── Public API ─────────────────────────────────────────────── */
void ignition_init(void) {
    for (int i = 0; i < ENGINE_CYL; i++) {
        gpio_init(CYL_GPIO[i]);
        gpio_set_dir(CYL_GPIO[i], GPIO_OUT);
        gpio_put(CYL_GPIO[i], 0);
        s_cyl[i].coil_charging = false;
        s_cyl[i].armed = false;
    }
    s_alarm_hw = hardware_alarm_claim_unused(true);
    hardware_alarm_set_callback(s_alarm_hw, ign_alarm_isr);
}

void ignition_update(float rpm, float map_rel_kpa, bool synced) {
    if (!synced || rpm < 200.0f) { ignition_kill(); return; }

    s_advance_deg = lookup_advance(rpm, map_rel_kpa);

    float dwell_deg = IGN_DWELL_MS * rpm * 360.0f / 60000.0f;
    if (dwell_deg > 60.0f) dwell_deg = 60.0f;   // cap at 60° (4-stroke)

    uint16_t cur = crank_get_angle_tdeg() / 10;  // 0-719

    for (int c = 0; c < ENGINE_CYL; c++) {
        int32_t fire   = (int32_t)CYL_TDC_DEG[c] - (int32_t)s_advance_deg;
        int32_t charge = fire - (int32_t)dwell_deg;
        s_cyl[c].fire_deg   = wrap720(fire);
        s_cyl[c].charge_deg = wrap720(charge);

        int32_t dc = (int32_t)s_cyl[c].charge_deg - (int32_t)cur;
        if (dc < 0) dc += 720;
        int32_t df = (int32_t)s_cyl[c].fire_deg   - (int32_t)cur;
        if (df < 0) df += 720;

        if (dc <= 5 && !s_cyl[c].armed && !s_cyl[c].coil_charging) {
            s_cyl[c].armed = true;
            sched_alarm(angle_to_us((float)dc, rpm), ALARM_CHARGE, (uint)c);
        }
        if (s_cyl[c].coil_charging && df <= 5)
            sched_alarm(angle_to_us((float)df, rpm), ALARM_FIRE, (uint)c);
    }
}

void ignition_kill(void) {
    hardware_alarm_cancel(s_alarm_hw);
    s_pending_action = ALARM_NONE;
    for (int i = 0; i < ENGINE_CYL; i++) {
        gpio_put(CYL_GPIO[i], 0);
        s_cyl[i].coil_charging = false;
        s_cyl[i].armed = false;
    }
}

float ignition_get_advance_deg(void) { return s_advance_deg; }
