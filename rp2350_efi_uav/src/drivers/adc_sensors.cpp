/**
 * @file    adc_sensors.cpp
 * @brief   RP2350 ADC + DMA round-robin driver for EFI sensors
 *
 * The RP2350 ADC runs in free-running mode with DMA chaining.
 * Channel sequence: MAP → TPS → IAT → CLT (4 channels × 12-bit).
 * DMA fills a 256-sample circular buffer per channel.
 * adc_sensors_read() averages each channel's 256 samples for
 * anti-aliasing before applying the sensor transfer function.
 */

#include "adc_sensors.h"
#include "../ecu_config.h"

#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include <math.h>
#include <string.h>

// ── DMA buffer ───────────────────────────────────────────────
#define DMA_BUF_SAMPLES   256                   // must be power of 2
#define DMA_BUF_TOTAL     (DMA_BUF_SAMPLES * ADC_NUM_CH)

// Interleaved: [MAP0, TPS0, IAT0, CLT0, MAP1, TPS1, …]
static uint16_t  s_dma_buf[DMA_BUF_TOTAL];
static int       s_dma_chan = -1;

// ── Math helpers ──────────────────────────────────────────────

/**
 * ADC raw count → voltage at the ADC pin.
 */
static inline float adc_to_volts(uint16_t raw) {
    return ((float)raw / ADC_RESOLUTION) * ADC_VREF;
}

/**
 * MAP sensor: voltage → manifold absolute pressure [kPa]
 * Transfer: P_kPa = (V - V_min) / (V_max - V_min) × P_max
 * Clamp to physical limits.
 */
static float map_volts_to_kpa(float v) {
    float p = (v - MAP_V_MIN_V) / (MAP_V_MAX_V - MAP_V_MIN_V) * MAP_P_MAX_KPA;
    if (p <   0.0f) p =   0.0f;
    if (p > MAP_P_MAX_KPA) p = MAP_P_MAX_KPA;
    return p;
}

/**
 * NTC thermistor β-model: voltage divider → temperature [°C]
 *
 *   R_ntc = R_pullup × V_adc / (Vref - V_adc)
 *   1/T   = 1/T_ref + (1/β) × ln(R_ntc / R_ref)
 *
 * @param v     voltage at ADC pin [V]
 * @param beta  β coefficient of the NTC
 * @returns     temperature in °C
 */
static float ntc_volts_to_degC(float v, float beta) {
    if (v <= 0.01f || v >= (ADC_VREF - 0.01f)) return -40.0f; // clamp
    float r_ntc = NTC_R_PULLUP * v / (ADC_VREF - v);
    float inv_T = (1.0f / NTC_T_REF_K) + (1.0f / beta) * logf(r_ntc / NTC_R_REF_OHM);
    return (1.0f / inv_T) - 273.15f;  // Kelvin → Celsius
}

/**
 * TPS potentiometer: voltage → throttle position [0-100 %]
 */
static float tps_volts_to_pct(float v) {
    float pct = (v - TPS_ADC_MIN) / (TPS_ADC_MAX - TPS_ADC_MIN) * 100.0f;
    if (pct <   0.0f) pct =   0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return pct;
}

/**
 * Wideband O2 (Innovate LC-2 analog output): voltage → lambda λ
 *   AFR = (V / V_max) × (AFR_max - AFR_min) + AFR_min
 *   λ   = AFR / STOICH_AFR
 */
static float o2_volts_to_lambda(float v) {
    // Scale voltage from 3.3 V ADC domain back to 0-5 V sensor output
    float v_sensor = v * (O2_V_MAX / ADC_VREF);
    float afr = (v_sensor / O2_V_MAX) * (O2_AFR_MAX - O2_AFR_MIN) + O2_AFR_MIN;
    return afr / STOICH_AFR;
}

// ── Average over DMA buffer per channel ──────────────────────
static float channel_average(int ch) {
    uint32_t acc = 0;
    for (int i = 0; i < DMA_BUF_SAMPLES; i++) {
        acc += s_dma_buf[i * ADC_NUM_CH + ch];
    }
    return adc_to_volts((uint16_t)(acc / DMA_BUF_SAMPLES));
}

// ── Public API ───────────────────────────────────────────────

void adc_sensors_init(void) {
    // Enable ADC and configure input pins (GP26-GP29)
    adc_init();
    adc_gpio_init(PIN_ADC_MAP);
    adc_gpio_init(PIN_ADC_TPS);
    adc_gpio_init(PIN_ADC_IAT);
    adc_gpio_init(PIN_ADC_CLT);

    // Round-robin mode: channels 0-3
    adc_set_round_robin(0x0F);          // bits 0-3 = ch 0-3
    adc_fifo_setup(
        true,    // enable FIFO
        true,    // enable DMA data request
        1,       // DREQ threshold = 1 sample
        false,   // no error bit in FIFO data
        false    // keep 12-bit resolution (no 8-bit shift)
    );

    // DMA channel
    s_dma_chan = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(s_dma_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, false);   // always read from ADC FIFO
    channel_config_set_write_increment(&cfg, true);   // advance through buffer
    channel_config_set_dreq(&cfg, DREQ_ADC);          // pace to ADC output rate
    channel_config_set_ring(&cfg, true, 9);           // ring on write: 2^9 = 512 bytes = DMA_BUF_TOTAL × 2

    dma_channel_configure(
        s_dma_chan,
        &cfg,
        s_dma_buf,              // write destination
        &adc_hw->fifo,          // read source
        DMA_BUF_TOTAL,          // transfer count (ring restarts automatically)
        true                    // start immediately
    );

    // Start ADC free-running
    adc_set_clkdiv(0);          // max ADC clock ≈ 500 kHz → 125 kHz per channel
    adc_run(true);
}

void adc_sensors_read(SensorReadings_t *out) {
    // Average each channel over the DMA buffer
    float v_map = channel_average(ADC_CH_MAP);
    float v_tps = channel_average(ADC_CH_TPS);
    float v_iat = channel_average(ADC_CH_IAT);
    float v_clt = channel_average(ADC_CH_CLT);

    // Apply transfer functions
    out->map_kpa  = map_volts_to_kpa(v_map);
    out->tps_pct  = tps_volts_to_pct(v_tps);
    out->iat_degC = ntc_volts_to_degC(v_iat, NTC_BETA_IAT);
    out->clt_degC = ntc_volts_to_degC(v_clt, NTC_BETA_CLT);
    // O2 sensor shares ADC_CH_MAP pin in this demo – in real HW use 5th channel
    out->lambda   = o2_volts_to_lambda(v_map);  // placeholder – wire separately
}
