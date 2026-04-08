/**
 * @file    ignition.h
 * @brief   Ignition timing, coil dwell, and crank-angle scheduling
 *          for a 4-stroke UAV piston engine on RP2350
 *
 * 4-Stroke ignition fundamentals
 * ──────────────────────────────
 *  One power stroke per cylinder every TWO crank revolutions (720°).
 *  For a flat-twin (2 cyl, 360° firing interval):
 *    Cyl 1 fires at   0° (TDC compression)  — crank angle 0°
 *    Cyl 2 fires at 360° (TDC compression)  — crank angle 360°
 *
 *  Coil charge (dwell) must begin DWELL_DEG degrees BEFORE the fire point:
 *    dwell_deg = IGN_DWELL_MS × RPM × 360° / 60000
 *
 *  Fire angle = TDC − advance_deg  (BTDC → subtract from TDC reference)
 *  Charge start angle = fire_angle − dwell_deg
 *
 * Ignition advance table
 * ──────────────────────
 *  2D look-up table indexed by RPM and MAP [kPa].
 *  Values in degrees BTDC.
 *  At idle: ~5–8° BTDC; full power: ~20–32° BTDC.
 *  UAV note: advance is deliberately conservative at high MAP-relative
 *            values because altitude reduces octane knock resistance.
 *
 * Hardware implementation
 * ────────────────────────
 *  The RP2350 hardware alarm (timer) fires a callback at the exact
 *  microsecond the coil charge or spark should occur.
 *  The callback is computed each tooth by converting the target crank
 *  angle into a time offset:
 *
 *    t_offset_us = (angle_deg / 360°) × (60 / RPM) × 1e6
 *
 *  This is scheduled as an absolute alarm via hardware_alarm_set_target().
 *
 * Actuators driven
 * ────────────────
 *  PIN_IGN_1 (GP10) – Coil driver cylinder 1  (active HIGH = charging)
 *  PIN_IGN_2 (GP11) – Coil driver cylinder 2
 *  Both are open-collector IGBT outputs; HIGH charges the coil,
 *  LOW → falling edge → spark.
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ── Ignition advance table dimensions ───────────────────────
#define IGN_RPM_BINS   16
#define IGN_MAP_BINS   12

extern const float IGN_RPM_AXIS[IGN_RPM_BINS];  // [RPM]
extern const float IGN_MAP_AXIS[IGN_MAP_BINS];  // [kPa] – MAP-relative

// Advance table: rows = RPM, cols = MAP-relative [kPa] → degrees BTDC
extern const float IGN_ADVANCE_TABLE[IGN_RPM_BINS][IGN_MAP_BINS];

// ── Per-cylinder ignition state ──────────────────────────────
typedef struct {
    uint      coil_gpio;       // GP10 or GP11
    uint16_t  fire_angle_deg;  // crank angle to fire spark [0–719]
    uint16_t  charge_angle_deg;// crank angle to begin coil charge [0–719]
    bool      coil_charging;   // true while coil is being charged
    bool      armed;           // true when next fire angle is valid
} CylIgnState_t;

/**
 * Initialise ignition module.
 * Must be called after crank_sensor_init().
 */
void ignition_init(void);

/**
 * Called every control loop tick (2 kHz) from Core 0.
 * Computes advance angle, schedules coil charge and fire via hardware alarm.
 *
 * @param rpm          current engine speed [RPM]
 * @param map_rel_kpa  MAP relative to BAP [kPa]  (negative = vacuum)
 * @param synced       true when crank wheel is synchronised
 */
void ignition_update(float rpm, float map_rel_kpa, bool synced);

/**
 * Emergency immediate spark cutoff (kill switch / over-temp).
 * Discharges both coils immediately.
 */
void ignition_kill(void);

/**
 * Returns the last computed ignition advance in degrees BTDC.
 */
float ignition_get_advance_deg(void);

#ifdef __cplusplus
}
#endif
