/**
 * @file    uav_sensors.h
 * @brief   UAV-specific sensor drivers
 *
 * ┌─────────────────────────────────────────────────────────────┐
 * │ MS5611 – Barometric Absolute Pressure sensor (SPI0)         │
 * │                                                             │
 * │  Resolution:  0.012 mbar (≈ 10 cm altitude resolution)      │
 * │  Range:       10 … 1200 mbar                                │
 * │  Interface:   SPI (Mode 0 or Mode 3), up to 20 MHz          │
 * │  Output:      Temperature-compensated pressure in Pa        │
 * │                                                             │
 * │  Why needed for UAV:                                        │
 * │  Automotive ECUs assume ~101.3 kPa sea-level. A UAV at      │
 * │  3000 m sees ~70 kPa — a 31 % air density drop. Without     │
 * │  BAP, the MAP reading has no absolute reference and the     │
 * │  VE lookup is completely wrong.                             │
 * └─────────────────────────────────────────────────────────────┘
 *
 * ┌─────────────────────────────────────────────────────────────┐
 * │ MAX31855 – K-type Thermocouple amplifier (SPI0)             │
 * │                                                             │
 * │  Range:       -200 … +1350 °C (K-type thermocouple)        │
 * │  Resolution:  0.25 °C                                       │
 * │  Interface:   SPI read-only, 32-bit frame                   │
 * │                                                             │
 * │  Why needed for UAV:                                        │
 * │  EGT is the primary indicator of combustion quality in      │
 * │  aviation. At altitude, lean mixtures are dangerous.        │
 * │  EGT > 750°C triggers enrichment; > 850°C = emergency.     │
 * └─────────────────────────────────────────────────────────────┘
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ─────────────────────────────────────────────────────────────
 *  MS5611 Barometric Pressure
 * ───────────────────────────────────────────────────────────── */

// MS5611 commands
#define MS5611_CMD_RESET        0x1E
#define MS5611_CMD_CONV_D1_4096 0x48   // pressure conversion, OSR=4096
#define MS5611_CMD_CONV_D2_4096 0x58   // temperature conversion, OSR=4096
#define MS5611_CMD_ADC_READ     0x00
#define MS5611_CMD_PROM_READ(n) (0xA0 | ((n) << 1))  // n = 0..7

// MS5611 internal calibration coefficients (loaded from PROM at init)
typedef struct {
    uint16_t C[8];   // factory calibration: C1-C6 valid, C0=manufacturer, C7=CRC
    bool     valid;
} Ms5611Prom_t;

/**
 * Initialise MS5611: SPI0, reset, read PROM coefficients, verify CRC.
 * @returns true if PROM CRC passes.
 */
bool ms5611_init(void);

/**
 * Trigger a conversion and read compensated pressure + temperature.
 * Must be called once per control loop cycle (takes ~10 ms per conversion
 * at OSR=4096 – use a state machine to interleave with other work).
 *
 * @param pressure_pa   output: compensated pressure [Pa]
 * @param temp_degC     output: compensated temperature [°C] (ambient at sensor)
 * @returns false if SPI communication failed.
 */
bool ms5611_read(float *pressure_pa, float *temp_degC);


/* ─────────────────────────────────────────────────────────────
 *  MAX31855 EGT Thermocouple Amplifier
 * ───────────────────────────────────────────────────────────── */

typedef struct {
    float  thermocouple_degC;   // junction temperature (EGT)
    float  internal_degC;       // cold-junction (PCB) temperature
    bool   fault_open;          // thermocouple open circuit
    bool   fault_short_gnd;     // thermocouple shorted to GND
    bool   fault_short_vcc;     // thermocouple shorted to VCC
    bool   any_fault;
} Max31855Data_t;

/**
 * Initialise MAX31855 SPI chip-select pin.
 */
void max31855_init(void);

/**
 * Read 32-bit frame from MAX31855, decode temperature and fault bits.
 * @param out  pointer to result structure
 * @returns false if SPI read failed (all zeros / timeout).
 */
bool max31855_read(Max31855Data_t *out);


/* ─────────────────────────────────────────────────────────────
 *  Altitude & Density computation
 * ───────────────────────────────────────────────────────────── */

/**
 * Compute pressure altitude from barometric pressure using ISA model.
 * Uses the hypsometric formula (valid to ~11 km):
 *
 *   H = (T0/L) × [1 - (P/P0)^(R·L/g)]
 *
 * @param bap_pa  measured barometric pressure [Pa]
 * @returns       altitude above MSL [m]
 */
float bap_to_altitude_m(float bap_pa);

/**
 * Compute air density ratio σ = ρ/ρ₀ at given pressure and temperature.
 * Used directly as a multiplier on the fuel equation.
 *
 *   σ = (P / P0) × (T0 / T)
 *
 * @param bap_pa    measured barometric pressure [Pa]
 * @param iat_degC  measured intake air temperature [°C]
 * @returns         density ratio [dimensionless, 1.0 at ISA sea level]
 */
float compute_density_ratio(float bap_pa, float iat_degC);

#ifdef __cplusplus
}
#endif
