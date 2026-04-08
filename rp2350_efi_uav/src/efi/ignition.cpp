/**
 * @file    ignition.cpp
 * @brief   4-stroke ignition timing implementation – RP2350 hardware alarm
 *
 * Scheduling strategy
 * ───────────────────
 *  Every tooth ISR updates crank angle and RPM.
 *  This module, called at 2 kHz, recalculates:
 *    1. advance_deg  ← table lookup (RPM, MAP-rel)
 *    2. fire_deg     ← TDC_angle − advance_deg          (per cylinder)
 *    3. charge_deg   ← fire_deg  − dwell_deg            (per cylinder)
 *
 *  It then checks whether the current crank angle is within one tooth
 *  of the charge_deg or fire_deg window. If so, it arms a hardware
 *  alarm (timer0) to fire at the precise microsecond offset.
 *
 *  Hardware alarm callback (ISR):
 *    CHARGE callback → set coil GPIO HIGH (begin charging)
 *    FIRE   callback → set coil GPIO LOW  (spark!)
 *
 *  4-stroke flat-twin TDC reference angles (0–719°):
 *    Cyl 1 TDC compression:   0°   (crank tooth gap = 0°)
 *    Cyl 2 TDC compression: 360°   (180° crankshaft × 2 = 360° cam cycle)
 */

#include "ignition.h"
#include "../ecu_config.h"
#include "../drivers/crank_sensor.h"

#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"

#include <math.h>
#include <string.h>

// ── Ignition advance table ────────────────────────────────────
// Indexed by [RPM][MAP-relative kPa].
// MAP-relative = MAP_manifold - BAP  (negative = vacuum below atmosphere)
// Here we store it as 0–50 kPa range (0 = idle vacuum, 50 ≈ WOT at altitude)
// Values in degrees BTDC.

const float IGN_RPM_AXIS[IGN_RPM_BINS] = {
     800, 1000, 1200, 1500, 1800, 2200, 2600, 3000,
    3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000
};

// MAP-relative axis: 0 kPa (idle vacuum) → 50 kPa (near WOT)
const float IGN_MAP_AXIS[IGN_MAP_BINS] = {
    0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 48, 50
};

// Advance table [RPM_BINS][MAP_BINS] – degrees BTDC
// High vacuum (low load) → more advance; high MAP (high load) → less advance
// UAV: conservative on high-load cells (altitude reduces knock margin)
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

// ── 4-stroke cylinder TDC angles (crank degrees, 0–719°) ─────
// Flat-twin, 360° firing interval
static const uint16_t CYL_TDC_DEG[2] = { 0, 360 };
static const uint      CYL_GPIO[2]    = { PIN_IGN_1, PIN_IGN_2 };

// ── Module state ──────────────────────────────────────────────
static CylIgnState_t s_cyl[2];
static float         s_advance_deg    = 10.0f;
static int           s_alarm_hw       = -1;  // hardware alarm number

// Pending alarm action type
typedef enum { ALARM_NONE, ALARM_CHARGE, ALARM_FIRE } AlarmAction_t;
static volatile AlarmAction_t s_pending_action = ALARM_NONE;
static volatile uint          s_pending_cyl    = 0;

// ── Advance table lookup (bilinear) ──────────────────────────
static float lookup_advance(float rpm, float map_rel_kpa) {
    // Clamp map_rel to table range (0–50 kPa)
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

    float v = IGN_ADVANCE_TABLE[ri][mi]   * (1-rf)*(1-mf)
            + IGN_ADVANCE_TABLE[ri+1][mi] * rf*(1-mf)
            + IGN_ADVANCE_TABLE[ri][mi+1] * (1-rf)*mf
            + IGN_ADVANCE_TABLE[ri+1][mi+1]* rf*mf;

    // Safety clamp
    if (v < IGN_MIN_ADVANCE_DEG) v = IGN_MIN_ADVANCE_DEG;
    if (v > IGN_MAX_ADVANCE_DEG) v = IGN_MAX_ADVANCE_DEG;
    return v;
}

// ── Convert crank-angle offset to microseconds ────────────────
// t_us = (angle_deg / 360.0) × (60 / RPM) × 1e6
static uint32_t angle_to_us(float angle_deg, float rpm) {
    if (rpm < 100.0f) return 0;
    return (uint32_t)((angle_deg / 360.0f) * (60.0e6f / rpm));
}

// Wrap crank angle to [0, 720) for 4-stroke
static inline uint16_t wrap720(int32_t a) {
    a %= 720;
    if (a < 0) a += 720;
    return (uint16_t)a;
}

// ── Hardware alarm ISR ────────────────────────────────────────
static void __isr __not_in_flash_func(ignition_alarm_isr)(void) {
    hardware_alarm_cancel(s_alarm_hw);

    uint cyl = s_pending_cyl;
    AlarmAction_t act = s_pending_action;
    s_pending_action = ALARM_NONE;

    if (act == ALARM_CHARGE) {
        gpio_put(CYL_GPIO[cyl], 1);  // begin coil charge (HIGH)
        s_cyl[cyl].coil_charging = true;
    } else if (act == ALARM_FIRE) {
        gpio_put(CYL_GPIO[cyl], 0);  // spark! falling edge fires IGBT
        s_cyl[cyl].coil_charging = false;
        s_cyl[cyl].armed         = false;
    }
}

// ── Schedule a hardware alarm ─────────────────────────────────
static void schedule_alarm(uint32_t delay_us, AlarmAction_t action, uint cyl) {
    if (delay_us == 0 || delay_us > 200000) return;  // sanity: 0–200 ms
    s_pending_action = action;
    s_pending_cyl    = cyl;
    uint64_t target  = time_us_64() + delay_us;
    hardware_alarm_set_target(s_alarm_hw, target);
}

// ── Public API ────────────────────────────────────────────────

void ignition_init(void) {
    // Coil GPIO outputs – start LOW (coil discharged)
    for (int i = 0; i < 2; i++) {
        gpio_init(CYL_GPIO[i]);
        gpio_set_dir(CYL_GPIO[i], GPIO_OUT);
        gpio_put(CYL_GPIO[i], 0);
        s_cyl[i].coil_gpio    = CYL_GPIO[i];
        s_cyl[i].coil_charging = false;
        s_cyl[i].armed        = false;
    }

    // Claim a hardware alarm (RP2350 has 4 timers)
    s_alarm_hw = hardware_alarm_claim_unused(true);
    hardware_alarm_set_callback(s_alarm_hw, ignition_alarm_isr);
}

void ignition_update(float rpm, float map_rel_kpa, bool synced) {
    if (!synced || rpm < 200.0f) {
        ignition_kill();
        return;
    }

    // ── 1. Compute advance ───────────────────────────────────
    s_advance_deg = lookup_advance(rpm, map_rel_kpa);

    // ── 2. Compute dwell angle ────────────────────────────────
    // dwell_deg = dwell_ms × RPM × 360 / 60000
    float dwell_deg = IGN_DWELL_MS * rpm * 360.0f / 60000.0f;
    if (dwell_deg > 60.0f) dwell_deg = 60.0f;  // cap at 60° (coil saturation)

    // ── 3. Current crank angle (4-stroke: 0–719°) ─────────────
    uint16_t cur_angle = crank_get_angle_tdeg() / 10;
    // Extend to 4-stroke cycle: if cam not fitted, ECU uses alternating
    // revolution counter to distinguish compression from exhaust TDC.
    // For simplicity, assume crank angle 0–719° is managed by crank_sensor.

    // ── 4. Schedule per cylinder ──────────────────────────────
    for (int c = 0; c < ENGINE_CYL; c++) {
        int32_t fire_deg   = (int32_t)CYL_TDC_DEG[c] - (int32_t)s_advance_deg;
        int32_t charge_deg = fire_deg - (int32_t)dwell_deg;
        s_cyl[c].fire_deg   = (uint16_t)wrap720(fire_deg);
        s_cyl[c].charge_deg = (uint16_t)wrap720(charge_deg);

        // Angle delta from now to charge start (forward-looking)
        int32_t delta_charge = (int32_t)s_cyl[c].charge_deg - (int32_t)cur_angle;
        if (delta_charge < 0)  delta_charge += 720;
        int32_t delta_fire   = (int32_t)s_cyl[c].fire_deg   - (int32_t)cur_angle;
        if (delta_fire < 0)    delta_fire   += 720;

        // Convert angle deltas to time (µs)
        uint32_t t_charge_us = angle_to_us((float)delta_charge, rpm);
        uint32_t t_fire_us   = angle_to_us((float)delta_fire,   rpm);

        // Arm coil charge if within next 5° and not already armed
        if (delta_charge <= 5 && !s_cyl[c].armed && !s_cyl[c].coil_charging) {
            s_cyl[c].armed = true;
            schedule_alarm(t_charge_us, ALARM_CHARGE, (uint)c);
        }
        // Schedule fire if coil is charging
        if (s_cyl[c].coil_charging && delta_fire <= 5) {
            schedule_alarm(t_fire_us, ALARM_FIRE, (uint)c);
        }
    }
}

void ignition_kill(void) {
    hardware_alarm_cancel(s_alarm_hw);
    s_pending_action = ALARM_NONE;
    for (int i = 0; i < 2; i++) {
        gpio_put(CYL_GPIO[i], 0);
        s_cyl[i].coil_charging = false;
        s_cyl[i].armed         = false;
    }
}

float ignition_get_advance_deg(void) { return s_advance_deg; }
