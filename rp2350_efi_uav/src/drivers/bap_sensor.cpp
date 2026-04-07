/**
 * @file    bap_sensor.cpp
 * @brief   MS5611 barometric sensor driver + ISA altitude / density model
 *
 * MS5611 compensation algorithm (from official datasheet, AN520)
 * ──────────────────────────────────────────────────────────────
 *  1. Read PROM: C1…C6 (factory calibrated coefficients)
 *  2. Convert D1 (pressure) and D2 (temperature) @ OSR=4096
 *  3. Calculate temperature:
 *       dT   = D2 - C5 × 2⁸
 *       TEMP = 2000 + dT × C6 / 2²³   (°C × 100)
 *  4. Calculate temperature-compensated pressure:
 *       OFF  = C2 × 2¹⁷ + (C4 × dT) / 2⁶
 *       SENS = C1 × 2¹⁶ + (C3 × dT) / 2⁷
 *       P    = (D1 × SENS/2²¹ - OFF) / 2¹⁵   (mbar × 100)
 *  5. Second-order temperature compensation (below 20°C)
 */

#include "bap_sensor.h"
#include "../ecu_config.h"

#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

// ── ISA constants ─────────────────────────────────────────────
static constexpr float ISA_P0   = ISA_P0_KPA;       // 101.325 kPa
static constexpr float ISA_T0   = ISA_T0_K;          // 288.15 K
static constexpr float ISA_L    = ISA_LAPSE_K_PER_M; // 0.0065 K/m
static constexpr float ISA_G    = 9.80665f;           // m/s²
static constexpr float ISA_M    = 0.0289644f;         // kg/mol
static constexpr float ISA_R    = 8.31446f;           // J/(mol·K)
static constexpr float R_AIR    = 287.05f;            // J/(kg·K)

// ISA exponent: g·M / (R·L)
static constexpr float ISA_EXP  = (ISA_G * ISA_M) / (ISA_R * ISA_L);  // ≈ 5.2561

// ── MS5611 PROM coefficients ──────────────────────────────────
static uint16_t s_C[7] = {};   // C1…C6 + CRC word (index 1-6 used)
static bool     s_prom_valid = false;
static uint8_t  s_fault_count = 0;

// ── SPI low-level helpers ─────────────────────────────────────
static inline void cs_low(void)  { gpio_put(PIN_SPI0_CS_BAP, 0); }
static inline void cs_high(void) { gpio_put(PIN_SPI0_CS_BAP, 1); }

static void spi_write_byte(uint8_t cmd) {
    cs_low();
    spi_write_blocking(spi0, &cmd, 1);
    cs_high();
}

static uint32_t spi_read_adc(void) {
    uint8_t cmd  = MS5611_CMD_ADC_READ;
    uint8_t buf[3] = {};
    cs_low();
    spi_write_blocking(spi0, &cmd, 1);
    spi_read_blocking(spi0, 0x00, buf, 3);
    cs_high();
    return ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
}

static uint16_t spi_read_prom(uint8_t addr_index) {
    uint8_t cmd = MS5611_CMD_PROM_BASE | (addr_index << 1);
    uint8_t buf[2] = {};
    cs_low();
    spi_write_blocking(spi0, &cmd, 1);
    spi_read_blocking(spi0, 0x00, buf, 2);
    cs_high();
    return ((uint16_t)buf[0] << 8) | buf[1];
}

// ── Init ──────────────────────────────────────────────────────
bool bap_sensor_init(void) {
    // SPI0 @ 1 MHz (safe for MS5611, well within 20 MHz max)
    spi_init(spi0, 1000000);
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PIN_SPI0_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI0_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI0_MISO, GPIO_FUNC_SPI);

    gpio_init(PIN_SPI0_CS_BAP);
    gpio_set_dir(PIN_SPI0_CS_BAP, GPIO_OUT);
    cs_high();

    // Reset sensor
    spi_write_byte(MS5611_CMD_RESET);
    sleep_ms(3);

    // Read PROM (7 words: word 0 = factory data, 1-6 = coefficients, 7 = CRC)
    for (int i = 0; i <= 6; i++) {
        s_C[i] = spi_read_prom(i);
    }

    // Basic sanity: C1 (pressure sensitivity) should be > 0
    s_prom_valid = (s_C[1] > 0 && s_C[2] > 0);
    if (!s_prom_valid) {
        printf("[BAP] PROM read failed – sensor not detected\n");
        return false;
    }
    printf("[BAP] MS5611 init OK  C1=%u C2=%u C3=%u\n", s_C[1], s_C[2], s_C[3]);
    return true;
}

// ── Read ──────────────────────────────────────────────────────
void bap_sensor_read(BapReading_t *out) {
    if (!s_prom_valid) {
        out->valid = false;
        out->fault_count++;
        return;
    }

    // ── Convert D1 (pressure) ─────────────────────────────────
    spi_write_byte(MS5611_CMD_CONV_D1_4096);
    sleep_ms(MS5611_CONV_DELAY_MS);
    uint32_t D1 = spi_read_adc();

    // ── Convert D2 (temperature) ──────────────────────────────
    spi_write_byte(MS5611_CMD_CONV_D2_4096);
    sleep_ms(MS5611_CONV_DELAY_MS);
    uint32_t D2 = spi_read_adc();

    // ── Temperature calculation ───────────────────────────────
    int32_t dT   = (int32_t)D2 - ((int32_t)s_C[5] << 8);
    int32_t TEMP = 2000 + ((int64_t)dT * s_C[6]) / (1 << 23);

    // ── Pressure calculation ──────────────────────────────────
    int64_t OFF  = ((int64_t)s_C[2] << 17) + ((int64_t)s_C[4] * dT) / (1 << 6);
    int64_t SENS = ((int64_t)s_C[1] << 16) + ((int64_t)s_C[3] * dT) / (1 << 7);

    // ── 2nd-order temperature compensation (T < 20°C) ────────
    if (TEMP < 2000) {
        int32_t T2    = ((int64_t)dT * dT) / (1LL << 31);
        int64_t OFF2  = 5LL * (TEMP - 2000) * (TEMP - 2000) / 2;
        int64_t SENS2 = 5LL * (TEMP - 2000) * (TEMP - 2000) / 4;
        if (TEMP < -1500) {
            OFF2  += 7LL * (TEMP + 1500) * (TEMP + 1500);
            SENS2 += 11LL * (TEMP + 1500) * (TEMP + 1500) / 2;
        }
        TEMP -= T2;
        OFF  -= OFF2;
        SENS -= SENS2;
    }

    int32_t P = (int32_t)(((int64_t)D1 * SENS / (1 << 21) - OFF) / (1 << 15));
    // P is in units of 0.01 mbar → convert to kPa
    float bap_kpa = (float)P / 1000.0f;   // 0.01 mbar = 0.001 kPa

    // ── Sanity check ──────────────────────────────────────────
    if (bap_kpa < 30.0f || bap_kpa > 110.0f) {
        s_fault_count++;
        out->valid = false;
        out->fault_count = s_fault_count;
        return;
    }

    s_fault_count = 0;
    out->bap_kpa       = bap_kpa;
    out->bap_temp_degC = (float)TEMP / 100.0f;
    out->altitude_m    = bap_pressure_to_altitude(bap_kpa);
    out->density_ratio = bap_density_ratio(bap_kpa, out->bap_temp_degC);
    out->valid         = true;
    out->fault_count   = 0;
}

// ── ISA altitude model ────────────────────────────────────────
float bap_pressure_to_altitude(float bap_kpa) {
    // ISA hypsometric formula (troposphere, h < 11 km):
    //   h = (T0/L) × [1 - (P/P0)^(R·L/(g·M))]
    //     = (T0/L) × [1 - (P/P0)^(1/ISA_EXP)]
    float ratio = bap_kpa / ISA_P0;
    if (ratio <= 0.0f) return UAV_MAX_ALT_M;
    float h = (ISA_T0 / ISA_L) * (1.0f - powf(ratio, 1.0f / ISA_EXP));
    if (h < 0.0f)         h = 0.0f;
    if (h > UAV_MAX_ALT_M) h = UAV_MAX_ALT_M;
    return h;
}

float bap_density_ratio(float bap_kpa, float iat_degC) {
    // ρ/ρ₀ = (P/P₀) × (T₀/T)   using the ideal gas law
    // where T₀ = 288.15 K (ISA sea level) and T = actual IAT
    float T_K   = iat_degC + 273.15f;
    float ratio = (bap_kpa / ISA_P0) * (ISA_T0 / T_K);
    if (ratio < 0.3f) ratio = 0.3f;   // extreme upper atmosphere clamp
    if (ratio > 1.2f) ratio = 1.2f;   // supercharged / cold day clamp
    return ratio;
}

float bap_ignition_altitude_correction(float altitude_m) {
    // Linear: +0.5° per 1000 m, max +4° at 8000 m
    // Rationale: thinner mixture is less knock-prone → allow slightly more
    // advance for peak efficiency. Conservative limit to protect engine.
    float corr = altitude_m * 0.0005f;    // 0.5°/1000 m
    if (corr > 4.0f) corr = 4.0f;
    if (corr < 0.0f) corr = 0.0f;
    return corr;
}
