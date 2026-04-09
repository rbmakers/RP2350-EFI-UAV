/**
 * @file    uav_sensors.cpp
 * @brief   MS5611 BAP + MAX31855 EGT drivers for RP2350 UAV-EFI
 */

#include "uav_sensors.h"
#include "../ecu_config.h"

#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <math.h>
#include <string.h>

// ── SPI0 instance ─────────────────────────────────────────────
static constexpr uint SPI_HZ = 5'000'000;   // 5 MHz – safe for both ICs

static inline void cs_bap_low (void) { gpio_put(PIN_SPI0_CS_BAP, 0); }
static inline void cs_bap_high(void) { gpio_put(PIN_SPI0_CS_BAP, 1); }
static inline void cs_egt_low (void) { gpio_put(PIN_SPI0_CS_EGT, 0); }
static inline void cs_egt_high(void) { gpio_put(PIN_SPI0_CS_EGT, 1); }

static Ms5611Prom_t s_prom;

// ─────────────────────────────────────────────────────────────
//  SPI0 shared initialiser (called once)
// ─────────────────────────────────────────────────────────────
static bool s_spi_inited = false;

static void spi0_init_once(void) {
    if (s_spi_inited) return;
    spi_init(spi0, SPI_HZ);
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(PIN_SPI0_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI0_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI0_MISO, GPIO_FUNC_SPI);

    // CS lines – manual GPIO
    gpio_init(PIN_SPI0_CS_BAP);  gpio_set_dir(PIN_SPI0_CS_BAP, GPIO_OUT);
    gpio_init(PIN_SPI0_CS_EGT);  gpio_set_dir(PIN_SPI0_CS_EGT, GPIO_OUT);
    cs_bap_high();
    cs_egt_high();
    s_spi_inited = true;
}

// ─────────────────────────────────────────────────────────────
//  MS5611 – Barometric Pressure
// ─────────────────────────────────────────────────────────────

static uint8_t ms5611_cmd_read8(uint8_t cmd) {
    uint8_t rx;
    cs_bap_low();
    spi_write_blocking(spi0, &cmd, 1);
    spi_read_blocking (spi0, 0x00, &rx, 1);
    cs_bap_high();
    return rx;
}

static void ms5611_send_cmd(uint8_t cmd) {
    cs_bap_low();
    spi_write_blocking(spi0, &cmd, 1);
    cs_bap_high();
}

static uint32_t ms5611_read_adc(void) {
    uint8_t cmd = MS5611_CMD_ADC_READ;
    uint8_t buf[3];
    cs_bap_low();
    spi_write_blocking(spi0, &cmd, 1);
    spi_read_blocking (spi0, 0x00, buf, 3);
    cs_bap_high();
    return ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
}

static uint16_t ms5611_read_prom_word(int n) {
    uint8_t cmd = MS5611_CMD_PROM_READ(n);
    uint8_t buf[2];
    cs_bap_low();
    spi_write_blocking(spi0, &cmd, 1);
    spi_read_blocking (spi0, 0x00, buf, 2);
    cs_bap_high();
    return ((uint16_t)buf[0] << 8) | buf[1];
}

// 4-bit CRC check from MS5611 datasheet
static uint8_t ms5611_crc4(uint16_t C[8]) {
    uint32_t rem = 0;
    C[7] = 0;
    uint16_t crc_save = C[0] & 0xF000;
    C[0] &= 0x0FFF;
    for (int i = 0; i < 16; i++) {
        rem ^= (i % 2 == 0) ? ((C[i >> 1]) >> 8) : (C[i >> 1] & 0xFF);
        for (int j = 8; j > 0; j--) {
            rem = (rem & 0x8000) ? (rem << 1) ^ 0x3000 : (rem << 1);
        }
    }
    rem = (rem >> 12) & 0xF;
    C[0] |= crc_save;
    return (uint8_t)rem;
}

bool ms5611_init(void) {
    spi0_init_once();

    // Reset
    ms5611_send_cmd(MS5611_CMD_RESET);
    sleep_ms(3);

    // Read PROM (8 × 16-bit words)
    for (int i = 0; i < 8; i++) {
        s_prom.C[i] = ms5611_read_prom_word(i);
    }

    // Verify CRC4
    uint8_t crc_calc = ms5611_crc4(s_prom.C);
    uint8_t crc_read = (uint8_t)(s_prom.C[7] & 0x000F);
    s_prom.valid = (crc_calc == crc_read);
    return s_prom.valid;
}

// State machine for interleaved conversion (D1 → D2 → D1 → …)
typedef enum { MS_STATE_D1, MS_STATE_D2 } Ms5611State_t;
static Ms5611State_t s_ms_state  = MS_STATE_D1;
static uint32_t      s_D1        = 0;
static uint32_t      s_D2        = 0;
static float         s_pres_pa   = ISA_P0_PA;
static float         s_temp_degC = 20.0f;

bool ms5611_read(float *pressure_pa, float *temp_degC) {
    if (!s_prom.valid) return false;

    // Read the result from the previous conversion, then trigger next
    if (s_ms_state == MS_STATE_D2) {
        s_D2 = ms5611_read_adc();

        // Temperature first (required for pressure compensation)
        int32_t dT   = (int32_t)s_D2 - ((int32_t)s_prom.C[5] << 8);
        int32_t TEMP = 2000 + (int32_t)(((int64_t)dT * s_prom.C[6]) >> 23);

        // Pressure compensation (2nd order)
        int64_t OFF  = ((int64_t)s_prom.C[2] << 16)
                     + (((int64_t)s_prom.C[4] * dT) >> 7);
        int64_t SENS = ((int64_t)s_prom.C[1] << 15)
                     + (((int64_t)s_prom.C[3] * dT) >> 8);

        // 2nd order correction for T < 20°C
        int32_t T2    = 0;
        int64_t OFF2  = 0, SENS2 = 0;
        if (TEMP < 2000) {
            T2    = (int32_t)(((int64_t)dT * dT) >> 31);
            int64_t d = (TEMP - 2000);
            OFF2  = (5 * d * d) >> 1;
            SENS2 = (5 * d * d) >> 2;
            if (TEMP < -1500) {
                int64_t e = (TEMP + 1500);
                OFF2  += 7 * e * e;
                SENS2 += (11 * e * e) >> 1;
            }
        }
        TEMP -= T2;
        OFF  -= OFF2;
        SENS -= SENS2;

        int32_t P = (int32_t)(((((int64_t)s_D1 * SENS) >> 21) - OFF) >> 15);

        s_pres_pa   = (float)P;       // in 0.01 mbar → Pa (P is already in Pa × 100 / 100)
        s_temp_degC = (float)TEMP / 100.0f;

        // Trigger next D1 conversion
        ms5611_send_cmd(MS5611_CMD_CONV_D1_4096);
        s_ms_state = MS_STATE_D1;
    } else {
        // Read D1 result, trigger D2 conversion
        s_D1 = ms5611_read_adc();
        ms5611_send_cmd(MS5611_CMD_CONV_D2_4096);
        s_ms_state = MS_STATE_D2;
    }

    *pressure_pa = s_pres_pa;
    *temp_degC   = s_temp_degC;
    return true;
}

// ─────────────────────────────────────────────────────────────
//  MAX31855 – K-type EGT Thermocouple
// ─────────────────────────────────────────────────────────────

void max31855_init(void) {
    spi0_init_once();
    // CS already configured in spi0_init_once
}

bool max31855_read(Max31855Data_t *out) {
    uint8_t buf[4];
    cs_egt_low();
    // MAX31855 is read-only: send 4 dummy bytes, read 32-bit frame
    uint8_t dummy[4] = {0, 0, 0, 0};
    spi_write_read_blocking(spi0, dummy, buf, 4);
    cs_egt_high();

    uint32_t raw = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16)
                 | ((uint32_t)buf[2] << 8)  | buf[3];

    // Check fault bits (bits 16, 2, 1, 0)
    out->fault_open      = (raw >> 0) & 1;
    out->fault_short_gnd = (raw >> 1) & 1;
    out->fault_short_vcc = (raw >> 2) & 1;
    out->any_fault       = (raw >> 16) & 1;

    if (out->any_fault) {
        out->thermocouple_degC = EGT_SENSOR_FAIL_LO;
        out->internal_degC     = 0.0f;
        return false;
    }

    // Thermocouple temperature: bits 31:18 (14-bit, 0.25°C resolution)
    int32_t tc_raw = (int32_t)(raw >> 18);
    if (tc_raw & 0x2000) tc_raw |= 0xFFFFC000;  // sign-extend 14-bit
    out->thermocouple_degC = (float)tc_raw * 0.25f;

    // Internal (cold junction) temperature: bits 15:4 (12-bit, 0.0625°C)
    int32_t cj_raw = (int32_t)((raw >> 4) & 0xFFF);
    if (cj_raw & 0x0800) cj_raw |= 0xFFFFF000;  // sign-extend 12-bit
    out->internal_degC = (float)cj_raw * 0.0625f;

    return true;
}

// ─────────────────────────────────────────────────────────────
//  Altitude & Density computation
// ─────────────────────────────────────────────────────────────

float bap_to_altitude_m(float bap_pa) {
    // Hypsometric formula (ISA troposphere, valid to ~11 km):
    //   H = (T0/L) × [1 - (P/P0)^(R·L/g)]
    // Exponent: R·L/g = 287.05 × 0.0065 / 9.80665 ≈ 0.190263
    static constexpr float EXP = (R_AIR_J_KGK * ISA_LAPSE_K_PER_M) / ISA_G_M_S2;
    static constexpr float T0_OVER_L = ISA_T0_K / ISA_LAPSE_K_PER_M;

    float ratio = bap_pa / ISA_P0_PA;
    if (ratio <= 0.0f) return UAV_ALT_MAX_M;
    float alt = T0_OVER_L * (1.0f - powf(ratio, EXP));
    if (alt < 0.0f)             alt = 0.0f;
    if (alt > UAV_ALT_MAX_M)    alt = UAV_ALT_MAX_M;
    return alt;
}

float compute_density_ratio(float bap_pa, float iat_degC) {
    // σ = ρ/ρ₀ = (P/P₀) × (T₀/T)
    // where T₀ = ISA sea-level temperature
    float T = iat_degC + 273.15f;
    if (T < 200.0f) T = 200.0f;    // safety clamp
    float sigma = (bap_pa / ISA_P0_PA) * (ISA_T0_K / T);
    if (sigma < 0.1f) sigma = 0.1f;
    if (sigma > 1.2f) sigma = 1.2f;
    return sigma;
}
