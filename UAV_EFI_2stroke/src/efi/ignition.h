/**
 * @file    ignition.h  [2-STROKE]
 * @brief   Ignition timing for 2-stroke UAV engine – fires every 360°
 *
 * Differences from 4-stroke ignition.h
 * ─────────────────────────────────────
 *  - Fires every revolution (360°), not every other (720°)
 *  - No cam sensor: every TDC is compression TDC
 *  - Dwell cap: 40° (vs 60° for 4-stroke) – less recharge time available
 *  - Advance map: hump at exhaust pipe resonance RPM (~4000), max 28° BTDC
 *  - Flat-twin: Cyl1 @ 0°, Cyl2 @ 180° within a single 360° window
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IGN_RPM_BINS   16
#define IGN_MAP_BINS   12

extern const float IGN_RPM_AXIS[IGN_RPM_BINS];
extern const float IGN_MAP_AXIS[IGN_MAP_BINS];
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
