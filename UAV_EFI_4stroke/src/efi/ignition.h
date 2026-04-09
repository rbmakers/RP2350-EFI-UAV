/**
 * @file    ignition.h  [4-STROKE]
 * @brief   Ignition timing, coil dwell and scheduling for 4-stroke UAV engine
 *
 * 4-stroke firing cycle
 * ─────────────────────
 *  Power stroke every 720° (two crank revolutions).
 *  Flat-twin: Cyl1 TDC @ 0°, Cyl2 TDC @ 360°.
 *
 *  Coil dwell cap: 60° (enough recharge time at 7000 RPM).
 *  Max advance:    35° BTDC.
 *
 *  Hardware: RP2350 hardware alarm (timer) schedules charge and fire
 *  events to microsecond precision without blocking Core 0.
 *
 *  Pins: PIN_IGN_1 (GP10), PIN_IGN_2 (GP11)
 *        HIGH = coil charging, falling edge = spark.
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IGN_RPM_BINS   16
#define IGN_MAP_BINS   12

/* Advance table axes */
extern const float IGN_RPM_AXIS[IGN_RPM_BINS];
extern const float IGN_MAP_AXIS[IGN_MAP_BINS];   // MAP-relative [kPa]
/* Advance table [RPM][MAP-rel] → degrees BTDC */
extern const float IGN_ADVANCE_TABLE[IGN_RPM_BINS][IGN_MAP_BINS];

typedef struct {
    uint      coil_gpio;
    uint16_t  fire_deg;
    uint16_t  charge_deg;
    bool      coil_charging;
    bool      armed;
} CylIgnState_t;

void  ignition_init(void);
void  ignition_update(float rpm, float map_rel_kpa, bool synced);
void  ignition_kill(void);
float ignition_get_advance_deg(void);

#ifdef __cplusplus
}
#endif
