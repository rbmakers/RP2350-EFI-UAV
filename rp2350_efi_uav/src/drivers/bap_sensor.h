/**
 * @file    bap_sensor.h
 * @brief   Barometric Absolute Pressure (BAP) driver – MS5611 via SPI0
 *
 * Why BAP is critical for UAV EFI
 * ────────────────────────────────
 * Automotive ECUs primarily use MAP (Manifold Absolute Pressure) to infer
 * engine load, because the vehicle operates in a narrow altitude band.
 * A UAV piston engine faces a radically different problem:
 *
 *   Altitude [m]   BAP [kPa]   Air density ratio (ρ/ρ₀)
 *   ──────────────────────────────────────────────────────
 *        0          101.3          1.000   ← sea level (ISA)
 *      500           95.5          0.954
 *     1000           89.9          0.908
 *     2000           79.5          0.822
 *     3000           70.1          0.742
 *     4000           61.6          0.669
 *
 * If the ECU does NOT correct for altitude:
 *   • Air mass per cycle is over-estimated → mixture runs RICH
 *   • At 3000 m the engine would receive ~35% too much fuel
 *   • This causes excessive fuel consumption, fouled plugs, and EGT spikes
 *
 * Correction strategy implemented here:
 *   density_ratio = (BAP_kPa / ISA_P0) × (ISA_T0 / T_air_K)
 *   fuel_correction = density_ratio  (scales the computed IPW down)
 *   ign_correction  = f(altitude)    (ignition advances slightly at altitude)
 *
 * MS5611 sensor characteristics
 * ──────────────────────────────
 *   Pressure range : 10 … 1200 mbar  (1 … 120 kPa)
 *   Resolution     : 0.012 mbar RMS  @ OSR=4096
 *   Temperature    : −40 … +85°C (internal compensation)
 *   Interface      : SPI mode 0 or I²C
 *   PROM           : 6 factory calibration coefficients (C1…C6)
 *
 * Reference: MS5611-01BA03 datasheet (TE Connectivity)
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ── MS5611 SPI commands ──────────────────────────────────────
#define MS5611_CMD_RESET        0x1E
#define MS5611_CMD_CONV_D1_4096 0x48  // pressure conversion OSR=4096
#define MS5611_CMD_CONV_D2_4096 0x58  // temperature conversion OSR=4096
#define MS5611_CMD_ADC_READ     0x00  // read 24-bit ADC result
#define MS5611_CMD_PROM_BASE    0xA0  // PROM read base (0xA0 … 0xAC)
#define MS5611_CONV_DELAY_MS    9     // OSR=4096 conversion time [ms]

// ── BAP reading structure ─────────────────────────────────────
typedef struct {
    float bap_kpa;          // compensated barometric pressure [kPa]
    float bap_temp_degC;    // internal temperature [°C]
    float altitude_m;       // pressure altitude (ISA model) [m]
    float density_ratio;    // ρ/ρ₀ actual vs ISA sea-level [0.0-1.0]
    bool  valid;            // false on SPI fault or implausible reading
    uint8_t fault_count;    // consecutive read failures
} BapReading_t;

/**
 * Initialise SPI0 and MS5611.
 * Reads and stores the 6 PROM calibration coefficients.
 * @returns true on success, false if sensor not detected
 */
bool bap_sensor_init(void);

/**
 * Trigger a conversion and read back pressure + temperature.
 * Blocking: takes ~18 ms for two OSR=4096 conversions.
 * Call from Core 0 at a reduced rate (e.g. 50 Hz) in a dedicated slot,
 * NOT every control tick, to avoid blocking the 2 kHz loop.
 */
void bap_sensor_read(BapReading_t *out);

/**
 * Compute pressure altitude using the ISA barometric formula.
 *   h = (T0/L) × [1 - (P/P0)^(R·L / (g·M))]
 * @param bap_kpa  measured barometric pressure [kPa]
 * @returns altitude above sea level [m]
 */
float bap_pressure_to_altitude(float bap_kpa);

/**
 * Compute air density ratio ρ/ρ₀ from pressure and temperature.
 * Uses the ideal gas law: ρ = P/(R_air × T)
 * @param bap_kpa   barometric pressure [kPa]
 * @param iat_degC  measured intake air temperature [°C]
 * @returns density ratio [dimensionless, 0.0-1.0+]
 */
float bap_density_ratio(float bap_kpa, float iat_degC);

/**
 * Returns the ignition advance correction for altitude.
 * At altitude, reduced knock tendency allows slightly more advance.
 * Linear model: +0.5° per 1000 m, capped at +4°.
 * @param altitude_m  pressure altitude [m]
 * @returns additional advance [° BTDC]
 */
float bap_ignition_altitude_correction(float altitude_m);

#ifdef __cplusplus
}
#endif
