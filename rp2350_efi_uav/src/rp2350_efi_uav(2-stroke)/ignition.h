/**
 * @file    ignition.h  (2-STROKE VARIANT)
 * @brief   Ignition timing for 2-stroke UAV engine – fires every 360°
 *
 * Key differences from 4-stroke ignition.h:
 *  - Fires every revolution (360°) not every other (720°)
 *  - No cam sensor dependency
 *  - Shorter maximum dwell (IGN_DWELL_MS = 1.8 ms)
 *  - Conservative advance map (max 28° BTDC, hump at pipe resonance RPM)
 *  - Flat-twin TDC: Cyl1=0°, Cyl2=180° (within single 360° window)
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
