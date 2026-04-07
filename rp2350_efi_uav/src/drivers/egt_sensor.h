/**
 * @file    egt_sensor.h / egt_sensor.cpp
 * @brief   Exhaust Gas Temperature driver – MAX31855 via SPI1
 *
 * Why EGT is essential for UAV EFI (unlike automotive)
 * ─────────────────────────────────────────────────────
 * Automotive ECUs rely on wideband O2 sensors as the primary closed-loop
 * feedback. UAV piston engines often lack O2 sensors due to weight limits
 * and the high temperatures at altitude. EGT fills a critical gap:
 *
 *   EGT Reading     Interpretation               Action
 *   ─────────────────────────────────────────────────────────────
 *   < 650°C         Very rich or low power         No action
 *   650 – 750°C     Ideal cruise zone              Maintain
 *   750 – EGT_WARN  Approaching lean limit          Monitor
 *   EGT_WARN (780°) Warning: running lean           Add 10% fuel
 *   EGT_CRIT (850°) Critical: detonation risk       Add 20%, limit power
 *   EGT_SHUT (950°) Valve/piston damage imminent    Emergency shutdown
 *
 * At altitude, EGT tends to rise because:
 *   1. Less dense air → combustion chamber temperatures spike
 *   2. Mixture may drift lean if altitude not compensated
 *   3. Reduced ram-air cooling of cylinder head
 *
 * MAX31855 characteristics
 * ─────────────────────────
 *   K-type range : −200 … +1350°C
 *   Resolution   : 0.25°C (12-bit thermocouple)
 *   Cold junction: −40 … +125°C (internal compensation)
 *   Fault detect : open, short-to-VCC, short-to-GND
 *   Interface    : SPI read-only (32-bit frame, CS-controlled)
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// MAX31855 fault bits (bits 2:0 of the 32-bit read)
#define MAX31855_FAULT_OC    0x01  // thermocouple open circuit
#define MAX31855_FAULT_SCG   0x02  // short to GND
#define MAX31855_FAULT_SCV   0x04  // short to VCC
#define MAX31855_FAULT_ANY   0x07  // any fault

// ── EGT protection states ─────────────────────────────────────
typedef enum {
    EGT_STATE_NORMAL   = 0,  // below warning threshold
    EGT_STATE_WARN     = 1,  // > EGT_WARN_DEGC  → richen 10%
    EGT_STATE_CRITICAL = 2,  // > EGT_CRITICAL_DEGC → richen 20%, power limit
    EGT_STATE_SHUTDOWN = 3,  // > EGT_SHUTDOWN_DEGC → stop engine
    EGT_STATE_FAULT    = 4,  // sensor fault detected
} EgtState_t;

typedef struct {
    float      egt_degC;          // thermocouple temperature [°C]
    float      cold_junction_degC;// MAX31855 internal (cold junction) [°C]
    EgtState_t state;             // protection state
    uint8_t    fault_bits;        // MAX31855 fault register bits
    bool       valid;             // false on SPI read error or open circuit
    // Derived enrichment command
    float      enrich_factor;     // 1.0 = none, 1.10 = warn, 1.20 = critical
    float      power_limit_pct;   // 100 = none, 80 = critical limit
} EgtReading_t;

/**
 * Initialise SPI1 for the MAX31855.
 */
void egt_sensor_init(void);

/**
 * Read 32-bit MAX31855 frame and decode.
 * Non-blocking SPI read (~1 µs at 1 MHz).
 * Call at 10-50 Hz from Core 0 (no need for 2 kHz).
 */
void egt_sensor_read(EgtReading_t *out);

/**
 * Evaluate EGT reading and populate protection fields.
 * Updates enrich_factor and power_limit_pct.
 */
void egt_evaluate(EgtReading_t *reading);

#ifdef __cplusplus
}
#endif


/* ─── IMPLEMENTATION ─────────────────────────────────────────── */
#ifdef EGT_IMPL

#include "../ecu_config.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <math.h>

static inline void egt_cs_low(void)  { gpio_put(PIN_SPI1_CS_EGT, 0); }
static inline void egt_cs_high(void) { gpio_put(PIN_SPI1_CS_EGT, 1); }

void egt_sensor_init(void) {
    spi_init(spi1, 1000000);
    spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PIN_SPI1_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI1_MISO, GPIO_FUNC_SPI);

    gpio_init(PIN_SPI1_CS_EGT);
    gpio_set_dir(PIN_SPI1_CS_EGT, GPIO_OUT);
    egt_cs_high();
}

void egt_sensor_read(EgtReading_t *out) {
    // Read 4 bytes (32-bit frame) from MAX31855
    uint8_t buf[4] = {};
    egt_cs_low();
    sleep_us(1);
    spi_read_blocking(spi1, 0x00, buf, 4);
    egt_cs_high();

    uint32_t raw = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16)
                 | ((uint32_t)buf[2] << 8)  | buf[3];

    // Bit 16: fault flag
    out->fault_bits = raw & 0x07;
    bool fault      = (raw >> 16) & 0x01;

    if (fault || out->fault_bits) {
        out->valid = false;
        out->state = EGT_STATE_FAULT;
        egt_evaluate(out);
        return;
    }

    // Thermocouple temperature: bits [31:18], 0.25°C/LSB, sign-extended
    int32_t tc_raw = (int32_t)(raw >> 18);
    if (tc_raw & (1 << 13)) tc_raw |= ~0x3FFF;  // sign-extend 14-bit
    out->egt_degC = (float)tc_raw * 0.25f;

    // Cold junction: bits [15:4], 0.0625°C/LSB, sign-extended
    int32_t cj_raw = (int32_t)((raw >> 4) & 0xFFF);
    if (cj_raw & (1 << 11)) cj_raw |= ~0x0FFF;
    out->cold_junction_degC = (float)cj_raw * 0.0625f;

    // Plausibility check
    out->valid = (out->egt_degC > -50.0f && out->egt_degC < EGT_INVALID_DEGC);
    egt_evaluate(out);
}

void egt_evaluate(EgtReading_t *r) {
    if (!r->valid) {
        r->state         = EGT_STATE_FAULT;
        r->enrich_factor = 1.10f;   // conservative: add 10% fuel on sensor fault
        r->power_limit_pct = 80.0f;
        return;
    }
    if (r->egt_degC >= EGT_SHUTDOWN_DEGC) {
        r->state = EGT_STATE_SHUTDOWN;
        r->enrich_factor = 1.20f;
        r->power_limit_pct = 0.0f;    // stop engine
    } else if (r->egt_degC >= EGT_CRITICAL_DEGC) {
        r->state = EGT_STATE_CRITICAL;
        r->enrich_factor = 1.20f;     // +20% fuel
        r->power_limit_pct = 80.0f;   // limit power output
    } else if (r->egt_degC >= EGT_WARN_DEGC) {
        r->state = EGT_STATE_WARN;
        r->enrich_factor = 1.10f;     // +10% fuel
        r->power_limit_pct = 100.0f;
    } else {
        r->state = EGT_STATE_NORMAL;
        r->enrich_factor = 1.00f;
        r->power_limit_pct = 100.0f;
    }
}

#endif /* EGT_IMPL */
