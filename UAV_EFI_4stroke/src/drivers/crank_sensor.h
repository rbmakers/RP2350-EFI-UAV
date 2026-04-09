/**
 * @file    crank_sensor.h
 * @brief   60-2 crankshaft trigger wheel driver for RP2350 PIO
 *
 * The PIO state machine fires an IRQ on every rising edge.
 * This ISR counts teeth, detects the missing-tooth gap (60-2),
 * computes RPM, and updates the global crank-angle accumulator.
 *
 * Tooth gap detection
 * ───────────────────
 * A normal tooth period ≈ T_tooth.
 * When the gap is detected the measured period ≈ 3 × T_tooth.
 * We use a ratio threshold of 1.8× to confirm the gap.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Initialise PIO0/SM0 for the crank trigger wheel.
 * Must be called before the main loop.                        */
void crank_sensor_init(void);

/* Returns current RPM (updated every tooth, exponential average) */
float crank_get_rpm(void);

/* Returns current crank angle in tenths of a degree [0 – 7199]   */
uint16_t crank_get_angle_tdeg(void);

/* True once at least one full revolution has been observed        */
bool crank_is_synced(void);

/* Reset sync state (e.g. after stall)                            */
void crank_reset_sync(void);

#ifdef __cplusplus
}
#endif
