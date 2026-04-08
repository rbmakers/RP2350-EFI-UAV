/**
 *
 * 火箭鳥創客倉庫 製作
 *
 * @file    main.cpp
 * @brief   RP2350 UAV-EFI – Dual-core main entry point
 *
 * UAV additions vs. automotive main.cpp
 * ──────────────────────────────────────
 *  [NEW] MS5611 BAP sensor read (SPI, every 10 ms via state machine)
 *  [NEW] MAX31855 EGT sensor read
 *  [NEW] Altitude + density_ratio computation
 *  [NEW] Fault detection with fail-functional strategy
 *  [NEW] DroneCAN telemetry stub (Core 1)
 *  [NEW] Choke solenoid for in-flight cold restart
 *  [MOD] Fuel calc input extended with BAP, density_ratio, EGT, fault flags
 *  [MOD] No engine kill on sensor fault → Fail-Functional mode instead
 *
 * Seqlock: Core 0 writes g_ecu, Core 1 reads (same protocol as automotive).
 */

#include "ecu_config.h"
#include "drivers/crank_sensor.h"
#include "drivers/adc_sensors.h"
#include "drivers/uav_sensors.h"
#include "fusion/kalman_filter.h"
#include "efi/fuel_calc.h"
#include "efi/pid.h"

#define PID_IMPL
#include "efi/pid.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "hardware/uart.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

// ── Global shared state ──────────────────────────────────────
volatile EcuState_t g_ecu = {};

// ── Module instances ──────────────────────────────────────────
static MapBapKalman_t s_map_kf;
static IatKalman_t    s_iat_kf;
static EgtKalman_t    s_egt_kf;
static PID_t          s_iac_pid;

// ── Derivative state ──────────────────────────────────────────
static float s_tps_prev = 0.0f;
static float s_tps_dot  = 0.0f;

// ── BAP state-machine (MS5611 needs ~10 ms per conversion) ────
// We interleave conversions across many control ticks.
static float s_bap_kpa   = ISA_P0_PA / 1000.0f;  // init to sea-level
static float s_bap_tempC = 20.0f;
static uint32_t s_bap_tick = 0;
#define BAP_CONV_TICKS 20   // 20 × 0.5 ms = 10 ms per BAP sample

// ── EGT sample (MAX31855 reads instantly) ─────────────────────
static Max31855Data_t s_egt_data = {};

// ── Seqlock helpers ──────────────────────────────────────────
static inline void seqlock_write_begin(void) { g_ecu.seq++; __dmb(); }
static inline void seqlock_write_end  (void) { __dmb(); g_ecu.seq++; }

// ── Fault detection ───────────────────────────────────────────
static uint16_t detect_faults(const SensorReadings_t *sens,
                               float bap_kpa, float egt_degC,
                               bool egt_valid) {
    uint16_t f = FAULT_NONE;

    // MAP: should be 20-120 kPa (wider than auto: high altitude UAV can see 40 kPa)
    if (sens->map_kpa < 15.0f || sens->map_kpa > 130.0f) f |= FAULT_MAP_SENSOR;

    // IAT: NTC open = rail voltage, short = 0 V
    if (sens->iat_degC < -55.0f || sens->iat_degC > 110.0f) f |= FAULT_IAT_SENSOR;

    // CHT (cylinder head temperature) – replaces CLT in air-cooled UAV engines
    if (sens->clt_degC < -55.0f || sens->clt_degC > 250.0f) f |= FAULT_CHT_SENSOR;

    // BAP: sea-level = 101.3 kPa, 5000 m ≈ 54 kPa
    if (bap_kpa < 40.0f || bap_kpa > 110.0f) f |= FAULT_BAP_SENSOR;

    // EGT
    if (!egt_valid) f |= FAULT_EGT_SENSOR;
    else if (egt_degC >= EGT_CRITICAL_DEGC) { f |= FAULT_EGT_CRITICAL; f |= FAULT_EGT_WARN; }
    else if (egt_degC >= EGT_WARN_DEGC)     { f |= FAULT_EGT_WARN; }

    return f;
}

// ── Injector scheduling (simplified; production uses PIO alarm) ─
static void schedule_injection(uint gpio, float ipw_us) {
    if (ipw_us < 200.0f) return;
    gpio_put(gpio, 1);
    busy_wait_us((uint32_t)ipw_us);
    gpio_put(gpio, 0);
}

// ── IAC PWM ──────────────────────────────────────────────────
static void iac_set_duty(float pct) {
    uint16_t lv = (uint16_t)(pct / 100.0f * 65535.0f);
    pwm_set_gpio_level(PIN_IAC_PWM, lv);
}

// ── DroneCAN telemetry (stub) ─────────────────────────────────
// In production replace with a proper UAVCAN v1 / Cyphal stack.
// The CAN peripheral is mapped to GPIO22/23; use a MCP2515 or
// RP2350's built-in CAN controller when available in pico-sdk.
static void dronecan_send_ecu_status(const EcuState_t *e) {
    // UAVCAN v1 message: uavcan.equipment.ice.reciprocating.Status (ID 1110)
    // Fields: engine_speed_rpm, atmospheric_pressure_kpa,
    //         intake_manifold_pressure_kpa, intake_manifold_temperature, ...
    // For now: encode as a simple 8-byte CAN frame (CAN 2.0B, 29-bit ID)
    uint8_t payload[8];
    uint16_t rpm_u16 = (uint16_t)e->rpm;
    uint16_t map_u16 = (uint16_t)(e->map_kpa * 10.0f);   // 0.1 kPa resolution
    uint16_t egt_u16 = (uint16_t)(e->egt_degC);
    uint16_t fault_u16 = e->fault_flags;

    payload[0] = rpm_u16 & 0xFF;
    payload[1] = (rpm_u16 >> 8) & 0xFF;
    payload[2] = map_u16 & 0xFF;
    payload[3] = (map_u16 >> 8) & 0xFF;
    payload[4] = egt_u16 & 0xFF;
    payload[5] = (egt_u16 >> 8) & 0xFF;
    payload[6] = fault_u16 & 0xFF;
    payload[7] = (fault_u16 >> 8) & 0xFF;

    // TODO: transmit via CAN peripheral driver
    (void)payload;  // suppress unused warning until CAN driver integrated
}

// ── Serial CSV output ─────────────────────────────────────────
static void uart_send_csv(const EcuState_t *e) {
    char buf[256];
    snprintf(buf, sizeof(buf),
        "%.0f,%.2f,%.2f,%.1f,%.1f,%.1f,%.3f,%.0f,%.2f,%.3f,%.1f,%.1f,%d\r\n",
        (double)e->rpm,
        (double)e->map_kpa,
        (double)e->bap_kpa,
        (double)e->altitude_m,
        (double)e->iat_degC,
        (double)e->egt_degC,
        (double)e->lambda,
        (double)e->ipw_us,
        (double)e->density_ratio,
        (double)e->lambda_trim,
        (double)e->alpha_n_blend,
        (double)e->egt_degC,
        (int)e->fault_flags);
    uart_puts(uart1, buf);
}

// ─────────────────────────────────────────────────────────────
//  CORE 1 – Housekeeping / DroneCAN / Serial @ 100 Hz
// ─────────────────────────────────────────────────────────────
static void core1_entry(void) {
    uart_init(uart1, UART_BAUD);
    gpio_set_function(PIN_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART_RX, GPIO_FUNC_UART);

    const uint32_t period_us = 1000000 / COMMS_LOOP_HZ;

    while (true) {
        uint64_t t0 = time_us_64();

        // Seqlock read snapshot
        EcuState_t snap;
        uint32_t s0, s1;
        do {
            s0 = g_ecu.seq;
            __dmb();
            memcpy((void*)&snap, (const void*)&g_ecu, sizeof(snap));
            __dmb();
            s1 = g_ecu.seq;
        } while (s0 != s1 || (s0 & 1u));

        uart_send_csv(&snap);
        dronecan_send_ecu_status(&snap);

        // Receive tuning commands from GCS / TunerStudio
        while (uart_is_readable(uart1)) {
            (void)uart_getc(uart1);   // accumulate into line buffer (placeholder)
        }

        uint64_t elapsed = time_us_64() - t0;
        if (elapsed < period_us) sleep_us(period_us - elapsed);
    }
}

// ─────────────────────────────────────────────────────────────
//  CORE 0 – Setup
// ─────────────────────────────────────────────────────────────
static void hardware_init(void) {
    stdio_init_all();

    // Injector GPIOs
    const uint inj_pins[] = { PIN_INJ_1, PIN_INJ_2, PIN_INJ_3, PIN_INJ_4 };
    for (int i = 0; i < ENGINE_CYL; i++) {
        gpio_init(inj_pins[i]);
        gpio_set_dir(inj_pins[i], GPIO_OUT);
        gpio_put(inj_pins[i], 0);
    }

    // Fuel pump, choke, engine kill
    for (uint pin : { (uint)PIN_FUEL_PUMP, (uint)PIN_CHOKE, (uint)PIN_ENGINE_KILL }) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_put(pin, 0);
    }

    // IAC PWM @ 50 Hz
    gpio_set_function(PIN_IAC_PWM, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PIN_IAC_PWM);
    pwm_config pcfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&pcfg, 38.15f);
    pwm_init(slice, &pcfg, true);
    iac_set_duty(20.0f);

    // ADC + DMA
    adc_sensors_init();

    // PIO crank sensor
    crank_sensor_init();

    // UAV sensors: BAP (MS5611) + EGT (MAX31855)
    bool bap_ok = ms5611_init();
    max31855_init();
    if (!bap_ok) {
        // BAP init failed – flag will be set when fault_flags is checked
        // Do NOT halt – we continue in degraded mode
    }
}

// ─────────────────────────────────────────────────────────────
//  CORE 0 – Main Control Loop @ 2 kHz
// ─────────────────────────────────────────────────────────────
int main(void) {
    hardware_init();
    fuel_calc_init();

    // Seed KFs from first ADC read
    SensorReadings_t first;
    adc_sensors_read(&first);
    map_bap_kf_init(&s_map_kf, first.map_kpa, s_bap_kpa);
    iat_kf_init    (&s_iat_kf, first.iat_degC);
    egt_kf_init    (&s_egt_kf, 25.0f);

    // IAC PID
    pid_init(&s_iac_pid, 0.05f, 0.01f, 0.005f, 5.0f, 80.0f,
             1.0f / CONTROL_LOOP_HZ, 0.7f);

    // Fuel pump prime
    gpio_put(PIN_FUEL_PUMP, 1);
    sleep_ms(2000);

    multicore_launch_core1(core1_entry);

    const uint32_t period_us = 1000000 / CONTROL_LOOP_HZ;
    const uint inj_pins[]    = { PIN_INJ_1, PIN_INJ_2 };  // flat-twin

    while (true) {
        uint64_t tick_start = time_us_64();

        // ── 1. ADC sensors ────────────────────────────────────
        SensorReadings_t sens;
        adc_sensors_read(&sens);

        // ── 2. BAP state machine (non-blocking, ~every 10 ms) ─
        if (++s_bap_tick >= BAP_CONV_TICKS) {
            s_bap_tick = 0;
            float new_bap, new_bap_t;
            if (ms5611_read(&new_bap, &new_bap_t)) {
                s_bap_kpa   = new_bap / 1000.0f;   // Pa → kPa
                s_bap_tempC = new_bap_t;
            }
        }

        // ── 3. EGT sensor ─────────────────────────────────────
        // MAX31855 read takes < 1 µs; safe to call every tick
        bool egt_ok = max31855_read(&s_egt_data);
        float egt_meas = egt_ok ? s_egt_data.thermocouple_degC : EGT_SENSOR_FAIL_LO;

        // ── 4. Kalman Filter updates ──────────────────────────
        uint64_t now_us = time_us_64();
        map_bap_kf_update(&s_map_kf, sens.map_kpa, s_bap_kpa, now_us);
        iat_kf_update     (&s_iat_kf, sens.iat_degC);
        egt_kf_update     (&s_egt_kf, egt_meas);

        float map_est = map_bap_kf_get_map (&s_map_kf);
        float map_dot = map_bap_kf_get_rate(&s_map_kf);
        float bap_est = map_bap_kf_get_bap (&s_map_kf);
        float iat_est = iat_kf_get_temp    (&s_iat_kf);
        float egt_est = egt_kf_get_temp    (&s_egt_kf);

        // ── 5. Altitude + density ratio ───────────────────────
        float bap_pa        = bap_est * 1000.0f;       // kPa → Pa
        float altitude_m    = bap_to_altitude_m(bap_pa);
        float density_ratio = compute_density_ratio(bap_pa, iat_est);

        // ── 6. TPS derivative ─────────────────────────────────
        s_tps_dot  = (sens.tps_pct - s_tps_prev) * CONTROL_LOOP_HZ;
        s_tps_prev = sens.tps_pct;

        // ── 7. Crank state ────────────────────────────────────
        float    rpm    = crank_get_rpm();
        bool     synced = crank_is_synced();
        uint16_t angle  = crank_get_angle_tdeg() / 10;

        // ── 8. Fault detection ────────────────────────────────
        uint16_t faults = detect_faults(&sens, bap_est, egt_est, egt_ok);
        bool map_valid  = !(faults & FAULT_MAP_SENSOR);
        bool iat_valid  = !(faults & FAULT_IAT_SENSOR);

        // ── 9. Fuel calculation ───────────────────────────────
        FuelInput_t  fi = {};
        FuelOutput_t fo = {};

        fi.rpm              = rpm;
        fi.map_kpa          = map_est;
        fi.map_kpa_dot      = map_dot;
        fi.bap_kpa          = bap_est;
        fi.density_ratio    = density_ratio;
        fi.iat_degC         = iat_est;
        fi.cht_degC         = sens.clt_degC;   // CHT sensor on CLT ADC channel
        fi.egt_degC         = egt_est;
        fi.tps_pct          = sens.tps_pct;
        fi.tps_dot          = s_tps_dot;
        fi.lambda_meas      = sens.lambda;
        fi.closed_loop      = (sens.clt_degC > 70.0f) && synced && (rpm > 500.0f)
                              && !(faults & FAULT_EGT_WARN);  // disable CLO during EGT warn
        fi.map_sensor_valid = map_valid;
        fi.iat_sensor_valid = iat_valid;
        fi.egt_sensor_valid = egt_ok && !(faults & FAULT_EGT_SENSOR);
        fi.fault_flags      = faults;

        if (synced && rpm > 200.0f) {
            fuel_calc_run(&fi, &fo);
        }

        // ── 10. Injector scheduling ───────────────────────────
        if (synced && fo.ipw_final_us > 300.0f) {
            if (angle < 5) {           // ~TDC cylinder 1
                schedule_injection(inj_pins[0], fo.ipw_final_us);
            }
            if (angle > 355 && angle < 365) {   // ~TDC cylinder 2 (180° later)
                schedule_injection(inj_pins[1], fo.ipw_final_us);
            }
        }

        // ── 11. EGT emergency actions ─────────────────────────
        if (faults & FAULT_EGT_CRITICAL) {
            // Activate choke solenoid for maximum enrichment
            gpio_put(PIN_CHOKE, 1);
        } else {
            gpio_put(PIN_CHOKE, 0);
        }

        // ── 12. IAC PID ───────────────────────────────────────
        float iac_duty = 20.0f;
        if (synced && sens.tps_pct < 2.0f) {
            iac_duty = pid_update(&s_iac_pid, 800.0f, rpm);
        } else {
            pid_reset(&s_iac_pid);
            iac_duty = 20.0f;
        }
        iac_set_duty(iac_duty);

        // ── 13. Seqlock publish ───────────────────────────────
        seqlock_write_begin();

        g_ecu.map_kpa         = map_est;
        g_ecu.map_kpa_dot     = map_dot;
        g_ecu.bap_kpa         = bap_est;
        g_ecu.altitude_m      = altitude_m;
        g_ecu.density_ratio   = density_ratio;
        g_ecu.map_rel_kpa     = map_bap_kf_get_rel(&s_map_kf);
        g_ecu.iat_degC        = iat_est;
        g_ecu.cht_degC        = sens.clt_degC;
        g_ecu.egt_degC        = egt_est;
        g_ecu.tps_pct         = sens.tps_pct;
        g_ecu.lambda          = sens.lambda;
        g_ecu.rpm             = rpm;
        g_ecu.crank_angle_deg = angle;
        g_ecu.engine_sync     = synced;
        g_ecu.air_mass_mg     = fo.air_mass_mg;
        g_ecu.ipw_us          = fo.ipw_final_us;
        g_ecu.ign_advance_deg = 12.0f;   // ignition module placeholder
        g_ecu.lambda_trim     = fo.lambda_trim;
        g_ecu.iac_duty        = iac_duty;
        g_ecu.alpha_n_blend   = fo.alpha_n_blend;
        g_ecu.fail_mode       = fo.fail_mode;
        g_ecu.fault_flags     = faults;
        g_ecu.egt_emergency   = (faults & FAULT_EGT_CRITICAL) != 0;

        seqlock_write_end();

        // ── 14. Loop timing ───────────────────────────────────
        uint64_t elapsed = time_us_64() - tick_start;
        if (elapsed >= period_us) {
            // Overrun: flag it but don't halt (UAV must keep running)
            g_ecu.fault_flags |= FAULT_CORE_OVERRUN;
        } else {
            busy_wait_us(period_us - (uint32_t)elapsed);
        }
    }

    return 0;
}
