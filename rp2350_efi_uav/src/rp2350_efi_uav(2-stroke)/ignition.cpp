/**
 * @file    ignition.cpp  (2-STROKE VARIANT)
 * @brief   2-stroke ignition - fires every 360 degrees (every crank revolution)
 *
 * 2-stroke vs 4-stroke ignition differences:
 *  - Fire every 360 deg (not 720 deg): coil recharge time is halved
 *  - Shorter dwell: IGN_DWELL_MS = 1.8 ms  (vs 2.5 ms for 4-stroke)
 *  - No cam sensor: every TDC is compression TDC in a 2-stroke
 *  - Advance map shape: hump at exhaust pipe resonance RPM (~4000-6000),
 *    retard at very high RPM to prevent detonation / seizure
 *  - Flat-twin: Cyl1 @ 0 deg, Cyl2 @ 180 deg (within 360 deg window)
 */

#include "ignition.h"
#include "../ecu_config.h"
#include "../drivers/crank_sensor.h"

#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"
#include <math.h>

const float IGN_RPM_AXIS[IGN_RPM_BINS] = {
     800, 1000, 1500, 2000, 2500, 3000, 3500, 4000,
    5000, 6000, 6500, 7000, 7500, 8000, 8500, 9000
};

const float IGN_MAP_AXIS[IGN_MAP_BINS] = {
    0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 48, 50
};

// 2-stroke advance table [RPM][MAP-rel kPa] - degrees BTDC
// Hump at mid-RPM (exhaust pipe resonance), conservative at WOT
const float IGN_ADVANCE_TABLE[IGN_RPM_BINS][IGN_MAP_BINS] = {
    /* 800  */ { 5, 5, 4, 4, 3, 3, 3, 2, 2, 2, 2, 2 },
    /* 1000 */ { 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 3, 2 },
    /* 1500 */ {10,10, 9, 9, 8, 8, 7, 7, 6, 5, 5, 4 },
    /* 2000 */ {14,13,12,11,10,10, 9, 8, 8, 7, 6, 5 },
    /* 2500 */ {18,17,16,15,14,13,12,11,10, 9, 8, 7 },
    /* 3000 */ {22,21,20,18,17,16,15,13,12,11,10, 9 },
    /* 3500 */ {25,24,22,20,19,18,16,15,14,12,11,10 },
    /* 4000 */ {28,27,25,23,21,20,18,17,15,14,12,11 },
    /* 5000 */ {27,26,24,22,20,19,17,16,14,13,12,11 },
    /* 6000 */ {25,24,22,20,18,17,15,14,13,12,11,10 },
    /* 6500 */ {23,22,20,18,16,15,14,13,12,11,10, 9 },
    /* 7000 */ {21,20,18,16,15,14,13,12,11,10, 9, 8 },
    /* 7500 */ {18,17,16,14,13,12,11,10, 9, 9, 8, 7 },
    /* 8000 */ {16,15,14,12,11,10, 9, 9, 8, 8, 7, 6 },
    /* 8500 */ {14,13,12,11,10, 9, 8, 8, 7, 7, 6, 5 },
    /* 9000 */ {12,11,10, 9, 8, 8, 7, 7, 6, 6, 5, 5 },
};

static const uint16_t CYL_TDC_DEG[2] = { 0, 180 };  // flat-twin, 180 deg phase
static const uint      CYL_GPIO[2]    = { PIN_IGN_1, PIN_IGN_2 };
static CylIgnState_t   s_cyl[2];
static float           s_advance_deg  = 8.0f;
static int             s_alarm_hw     = -1;

typedef enum { ALARM_NONE, ALARM_CHARGE, ALARM_FIRE } AlarmAction_t;
static volatile AlarmAction_t s_pending_action = ALARM_NONE;
static volatile uint          s_pending_cyl    = 0;

static float lookup_advance(float rpm, float map_rel_kpa) {
    if (map_rel_kpa < 0.0f)  map_rel_kpa = 0.0f;
    if (map_rel_kpa > 50.0f) map_rel_kpa = 50.0f;
    int ri = 0;
    for (int i = 0; i < IGN_RPM_BINS - 1; i++)
        if (rpm >= IGN_RPM_AXIS[i]) ri = i;
    int mi = 0;
    for (int i = 0; i < IGN_MAP_BINS - 1; i++)
        if (map_rel_kpa >= IGN_MAP_AXIS[i]) mi = i;
    if (ri >= IGN_RPM_BINS - 1) ri = IGN_RPM_BINS - 2;
    if (mi >= IGN_MAP_BINS - 1) mi = IGN_MAP_BINS - 2;
    float rf = (rpm         - IGN_RPM_AXIS[ri]) / (IGN_RPM_AXIS[ri+1] - IGN_RPM_AXIS[ri]);
    float mf = (map_rel_kpa - IGN_MAP_AXIS[mi]) / (IGN_MAP_AXIS[mi+1] - IGN_MAP_AXIS[mi]);
    rf = rf < 0.0f ? 0.0f : rf > 1.0f ? 1.0f : rf;
    mf = mf < 0.0f ? 0.0f : mf > 1.0f ? 1.0f : mf;
    float v = IGN_ADVANCE_TABLE[ri][mi]    * (1-rf)*(1-mf)
            + IGN_ADVANCE_TABLE[ri+1][mi]  * rf*(1-mf)
            + IGN_ADVANCE_TABLE[ri][mi+1]  * (1-rf)*mf
            + IGN_ADVANCE_TABLE[ri+1][mi+1]* rf*mf;
    if (v < IGN_MIN_ADVANCE_DEG) v = IGN_MIN_ADVANCE_DEG;
    if (v > IGN_MAX_ADVANCE_DEG) v = IGN_MAX_ADVANCE_DEG;
    return v;
}

static uint32_t angle_to_us(float angle_deg, float rpm) {
    if (rpm < 100.0f) return 0;
    // 2-stroke: one revolution = 360 deg
    return (uint32_t)((angle_deg / 360.0f) * (60.0e6f / rpm));
}

static inline uint16_t wrap360(int32_t a) {
    a %= 360;
    if (a < 0) a += 360;
    return (uint16_t)a;
}

static void __isr __not_in_flash_func(ignition_alarm_isr)(void) {
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
        s_cyl[c].armed         = false;
    }
}

static void schedule_alarm(uint32_t delay_us, AlarmAction_t action, uint cyl) {
    if (delay_us == 0 || delay_us > 100000) return;
    s_pending_action = action;
    s_pending_cyl    = cyl;
    hardware_alarm_set_target(s_alarm_hw, time_us_64() + delay_us);
}

void ignition_init(void) {
    for (int i = 0; i < ENGINE_CYL; i++) {
        gpio_init(CYL_GPIO[i]);
        gpio_set_dir(CYL_GPIO[i], GPIO_OUT);
        gpio_put(CYL_GPIO[i], 0);
        s_cyl[i].coil_charging = false;
        s_cyl[i].armed         = false;
    }
    s_alarm_hw = hardware_alarm_claim_unused(true);
    hardware_alarm_set_callback(s_alarm_hw, ignition_alarm_isr);
}

void ignition_update(float rpm, float map_rel_kpa, bool synced) {
    if (!synced || rpm < 200.0f) { ignition_kill(); return; }
    s_advance_deg = lookup_advance(rpm, map_rel_kpa);

    // Dwell: shorter cap for 2-stroke (less time between firings)
    float dwell_deg = IGN_DWELL_MS * rpm * 360.0f / 60000.0f;
    if (dwell_deg > 40.0f) dwell_deg = 40.0f;

    // 2-stroke: crank angle is 0-359 deg only
    uint16_t cur = crank_get_angle_tdeg() / 10;
    if (cur >= 360) cur -= 360;

    for (int c = 0; c < ENGINE_CYL; c++) {
        int32_t fire_deg   = (int32_t)CYL_TDC_DEG[c] - (int32_t)s_advance_deg;
        int32_t charge_deg = fire_deg - (int32_t)dwell_deg;
        s_cyl[c].fire_deg   = wrap360(fire_deg);
        s_cyl[c].charge_deg = wrap360(charge_deg);

        int32_t dc = (int32_t)s_cyl[c].charge_deg - (int32_t)cur;
        if (dc < 0) dc += 360;
        int32_t df = (int32_t)s_cyl[c].fire_deg   - (int32_t)cur;
        if (df < 0) df += 360;

        if (dc <= 3 && !s_cyl[c].armed && !s_cyl[c].coil_charging) {
            s_cyl[c].armed = true;
            schedule_alarm(angle_to_us((float)dc, rpm), ALARM_CHARGE, (uint)c);
        }
        if (s_cyl[c].coil_charging && df <= 3) {
            schedule_alarm(angle_to_us((float)df, rpm), ALARM_FIRE, (uint)c);
        }
    }
}

void ignition_kill(void) {
    hardware_alarm_cancel(s_alarm_hw);
    s_pending_action = ALARM_NONE;
    for (int i = 0; i < ENGINE_CYL; i++) {
        gpio_put(CYL_GPIO[i], 0);
        s_cyl[i].coil_charging = false;
        s_cyl[i].armed         = false;
    }
}

float ignition_get_advance_deg(void) { return s_advance_deg; }
