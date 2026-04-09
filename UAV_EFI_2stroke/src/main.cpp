/**
 * @file    main.cpp  [2-STROKE]
 * @brief   RP2350 UAV-EFI 2-stroke – dual-core main
 *
 * 2-stroke specific additions vs 4-stroke main.cpp
 * ─────────────────────────────────────────────────
 *  - Injection fires every 360° (Cyl1@0°, Cyl2@180°), not every 720°
 *  - No cam sensor (PIN_CMP not initialised)
 *  - Oil injection pump (GP13 PWM): duty ∝ RPM, adjusts with TPS
 *  - EGT thresholds lower (EGT_WARN=680°C, EGT_CRITICAL=780°C)
 *  - oil_pump_duty published to g_ecu for telemetry
 *
 * Core 0 @ 2 kHz – deterministic control
 *   ADC → BAP/EGT → KF → faults → fuel → injectors (360°) →
 *   ignition → oil pump → IAC → seqlock publish
 *
 * Core 1 @ 100 Hz – housekeeping
 *   UART CSV (includes oil_pump_duty) → DroneCAN stub → tuning RX
 */

#include "ecu_config.h"
#include "drivers/crank_sensor.h"
#include "drivers/adc_sensors.h"
#include "drivers/uav_sensors.h"
#include "fusion/kalman_filter.h"
#include "efi/fuel_calc.h"
#include "efi/ignition.h"
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

/* ── Global shared state ─────────────────────────────────────── */
volatile EcuState_t g_ecu = {};

/* ── Module instances ────────────────────────────────────────── */
static MapBapKalman_t s_map_kf;
static IatKalman_t    s_iat_kf;
static EgtKalman_t    s_egt_kf;
static PID_t          s_iac_pid;

static float    s_tps_prev = 0.0f, s_tps_dot = 0.0f;
static float    s_bap_kpa  = ISA_P0_PA / 1000.0f;
static float    s_bap_tC   = 20.0f;
static uint32_t s_bap_tick = 0;
#define BAP_CONV_TICKS 20

static Max31855Data_t s_egt = {};

/* ── Seqlock ──────────────────────────────────────────────────── */
static inline void sl_begin(void) { g_ecu.seq++; __dmb(); }
static inline void sl_end  (void) { __dmb(); g_ecu.seq++; }

/* ── Fault detection ──────────────────────────────────────────── */
static uint16_t detect_faults(const SensorReadings_t *s,
                               float bap, float egt, bool egt_ok) {
    uint16_t f = FAULT_NONE;
    if (s->map_kpa  < 15.0f || s->map_kpa  > 130.0f) f |= FAULT_MAP_SENSOR;
    if (s->iat_degC < -55.0f|| s->iat_degC > 110.0f ) f |= FAULT_IAT_SENSOR;
    if (s->clt_degC < -55.0f|| s->clt_degC > 300.0f ) f |= FAULT_CHT_SENSOR;
    if (bap         < 40.0f || bap         > 110.0f  ) f |= FAULT_BAP_SENSOR;
    if (!egt_ok)                                        f |= FAULT_EGT_SENSOR;
    else if (egt >= EGT_CRITICAL_DEGC) f |= (FAULT_EGT_CRITICAL|FAULT_EGT_WARN);
    else if (egt >= EGT_WARN_DEGC)     f |= FAULT_EGT_WARN;
    return f;
}

/* ── Actuator helpers ─────────────────────────────────────────── */
static void schedule_injection(uint gpio, float ipw_us) {
    if (ipw_us < 200.0f) return;
    gpio_put(gpio, 1);
    busy_wait_us((uint32_t)ipw_us);
    gpio_put(gpio, 0);
}

static void iac_set_duty(float pct) {
    pwm_set_gpio_level(PIN_IAC_PWM,
        (uint16_t)(pct / 100.0f * 65535.0f));
}

/* ── [2-STROKE] Oil injection pump ───────────────────────────── */
/* Duty proportional to RPM; light-load = more oil for ring protection */
static float compute_oil_duty(float rpm, float tps_pct) {
    float base   = 20.0f + (rpm / 9000.0f) * 40.0f;   // 20-60 %
    float factor = 1.0f + (1.0f - tps_pct / 100.0f) * 0.3f;
    float duty   = base * factor;
    if (duty < 10.0f) duty = 10.0f;
    if (duty > 80.0f) duty = 80.0f;
    return duty;
}

static void oil_pump_set_duty(float pct) {
    pwm_set_gpio_level(PIN_OIL_INJ_PWM,
        (uint16_t)(pct / 100.0f * 65535.0f));
}

/* ── Serial CSV ───────────────────────────────────────────────── */
static void uart_csv(const EcuState_t *e) {
    char buf[280];
    snprintf(buf, sizeof(buf),
        "%.0f,%.2f,%.2f,%.1f,%.1f,%.1f,%.3f,%.1f,%.2f,%.1f,%.3f,%.1f,%u\r\n",
        (double)e->rpm, (double)e->map_kpa, (double)e->bap_kpa,
        (double)e->altitude_m, (double)e->iat_degC, (double)e->egt_degC,
        (double)e->lambda, (double)e->ipw_us, (double)e->density_ratio,
        (double)e->ign_advance_deg, (double)e->lambda_trim,
        (double)e->oil_pump_duty,  /* 2-stroke extra field */
        (unsigned)e->fault_flags);
    uart_puts(uart1, buf);
}

/* ── DroneCAN stub ────────────────────────────────────────────── */
static void dronecan_send(const EcuState_t *e) {
    uint8_t p[8];
    uint16_t r=(uint16_t)e->rpm, m=(uint16_t)(e->map_kpa*10.0f);
    uint16_t g=(uint16_t)e->egt_degC, ff=e->fault_flags;
    p[0]=r&0xFF;p[1]=(r>>8);p[2]=m&0xFF;p[3]=(m>>8);
    p[4]=g&0xFF;p[5]=(g>>8);p[6]=ff&0xFF;p[7]=(ff>>8);
    (void)p;
}

/* ══════════════════════════════════════════════════════════════
 *  CORE 1 – Communications @ 100 Hz
 * ══════════════════════════════════════════════════════════════ */
static void core1_entry(void) {
    uart_init(uart1, UART_BAUD);
    gpio_set_function(PIN_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART_RX, GPIO_FUNC_UART);

    const uint32_t period_us = 1000000u / COMMS_LOOP_HZ;
    while (true) {
        uint64_t t0 = time_us_64();
        EcuState_t snap;
        uint32_t s0, s1;
        do {
            s0=g_ecu.seq; __dmb();
            memcpy((void*)&snap,(const void*)&g_ecu,sizeof(snap));
            __dmb(); s1=g_ecu.seq;
        } while (s0!=s1||(s0&1u));

        uart_csv(&snap);
        dronecan_send(&snap);

        while (uart_is_readable(uart1))
            (void)uart_getc(uart1);

        uint64_t el = time_us_64()-t0;
        if (el < period_us) sleep_us(period_us-el);
    }
}

/* ══════════════════════════════════════════════════════════════
 *  CORE 0 – Hardware init
 * ══════════════════════════════════════════════════════════════ */
static void hw_init(void) {
    stdio_init_all();

    /* Injector GPIOs */
    for (uint p : {(uint)PIN_INJ_1,(uint)PIN_INJ_2}) {
        gpio_init(p); gpio_set_dir(p,GPIO_OUT); gpio_put(p,0);
    }
    /* Relays / solenoids */
    for (uint p : {(uint)PIN_FUEL_PUMP,(uint)PIN_CHOKE,(uint)PIN_ENGINE_KILL}) {
        gpio_init(p); gpio_set_dir(p,GPIO_OUT); gpio_put(p,0);
    }

    /* IAC PWM ~50 Hz */
    gpio_set_function(PIN_IAC_PWM, GPIO_FUNC_PWM);
    uint sl = pwm_gpio_to_slice_num(PIN_IAC_PWM);
    pwm_config pc = pwm_get_default_config();
    pwm_config_set_clkdiv(&pc, 38.15f);
    pwm_init(sl, &pc, true);
    iac_set_duty(20.0f);

    /* [2-STROKE] Oil injection pump PWM ~50 Hz */
    gpio_set_function(PIN_OIL_INJ_PWM, GPIO_FUNC_PWM);
    uint osl = pwm_gpio_to_slice_num(PIN_OIL_INJ_PWM);
    pwm_config opc = pwm_get_default_config();
    pwm_config_set_clkdiv(&opc, 38.15f);
    pwm_init(osl, &opc, true);
    oil_pump_set_duty(25.0f);   /* safe starting duty */

    /* Ignition */
    ignition_init();

    /* Sensors */
    adc_sensors_init();
    crank_sensor_init();
    ms5611_init();
    max31855_init();
}

/* ══════════════════════════════════════════════════════════════
 *  CORE 0 – Main control loop @ 2 kHz
 * ══════════════════════════════════════════════════════════════ */
int main(void) {
    hw_init();
    fuel_calc_init();

    SensorReadings_t first;
    adc_sensors_read(&first);
    map_bap_kf_init(&s_map_kf, first.map_kpa, s_bap_kpa);
    iat_kf_init    (&s_iat_kf, first.iat_degC);
    egt_kf_init    (&s_egt_kf, 25.0f);

    pid_init(&s_iac_pid, 0.05f, 0.01f, 0.005f,
             5.0f, 80.0f, 1.0f/CONTROL_LOOP_HZ, 0.7f);

    gpio_put(PIN_FUEL_PUMP, 1);
    sleep_ms(2000);

    multicore_launch_core1(core1_entry);

    const uint32_t period_us = 1000000u / CONTROL_LOOP_HZ;

    while (true) {
        uint64_t t0 = time_us_64();

        /* ── 1. ADC ─────────────────────────────────────────── */
        SensorReadings_t sens;
        adc_sensors_read(&sens);

        /* ── 2. BAP (non-blocking) ──────────────────────────── */
        if (++s_bap_tick >= BAP_CONV_TICKS) {
            s_bap_tick = 0;
            float bp, bt;
            if (ms5611_read(&bp, &bt)) { s_bap_kpa=bp/1000.0f; s_bap_tC=bt; }
        }

        /* ── 3. EGT ─────────────────────────────────────────── */
        bool egt_ok = max31855_read(&s_egt);
        float egt_m = egt_ok ? s_egt.thermocouple_degC : EGT_SENSOR_FAIL_LO;

        /* ── 4. Kalman filters ──────────────────────────────── */
        uint64_t now = time_us_64();
        map_bap_kf_update(&s_map_kf, sens.map_kpa, s_bap_kpa, now);
        iat_kf_update     (&s_iat_kf, sens.iat_degC);
        egt_kf_update     (&s_egt_kf, egt_m);

        float map_est = map_bap_kf_get_map (&s_map_kf);
        float map_dot = map_bap_kf_get_rate(&s_map_kf);
        float bap_est = map_bap_kf_get_bap (&s_map_kf);
        float iat_est = iat_kf_get_temp    (&s_iat_kf);
        float egt_est = egt_kf_get_temp    (&s_egt_kf);

        /* ── 5. Altitude + density ──────────────────────────── */
        float bap_pa  = bap_est * 1000.0f;
        float alt_m   = bap_to_altitude_m(bap_pa);
        float dens    = compute_density_ratio(bap_pa, iat_est);

        /* ── 6. TPS derivative ──────────────────────────────── */
        s_tps_dot  = (sens.tps_pct - s_tps_prev) * CONTROL_LOOP_HZ;
        s_tps_prev = sens.tps_pct;

        /* ── 7. Crank ───────────────────────────────────────── */
        float    rpm    = crank_get_rpm();
        bool     synced = crank_is_synced();
        uint16_t angle  = crank_get_angle_tdeg() / 10;
        /* 2-stroke: keep angle in [0, 359] */
        if (angle >= 360) angle -= 360;

        /* ── 8. Faults ──────────────────────────────────────── */
        uint16_t faults = detect_faults(&sens, bap_est, egt_est, egt_ok);

        /* ── 9. Fuel calc ───────────────────────────────────── */
        FuelInput_t  fi = {};
        FuelOutput_t fo = {};
        fi.rpm              = rpm;
        fi.map_kpa          = map_est;
        fi.map_kpa_dot      = map_dot;
        fi.bap_kpa          = bap_est;
        fi.density_ratio    = dens;
        fi.iat_degC         = iat_est;
        fi.cht_degC         = sens.clt_degC;
        fi.egt_degC         = egt_est;
        fi.tps_pct          = sens.tps_pct;
        fi.tps_dot          = s_tps_dot;
        fi.lambda_meas      = sens.lambda;
        fi.closed_loop      = (sens.clt_degC > 60.0f) && synced
                              && (rpm > 500.0f) && !(faults & FAULT_EGT_WARN);
        fi.map_sensor_valid = !(faults & FAULT_MAP_SENSOR);
        fi.iat_sensor_valid = !(faults & FAULT_IAT_SENSOR);
        fi.egt_sensor_valid = egt_ok && !(faults & FAULT_EGT_SENSOR);
        fi.fault_flags      = faults;

        if (synced && rpm > 200.0f) fuel_calc_run(&fi, &fo);

        /* ── 10. Injectors (2-stroke: every 360°) ───────────── */
        /* Cyl1 fires at ~0° TDC, Cyl2 at ~180° TDC */
        if (synced && fo.ipw_final_us > 200.0f) {
            if (angle < 5)
                schedule_injection(PIN_INJ_1, fo.ipw_final_us);
            if (angle >= 175 && angle < 185)
                schedule_injection(PIN_INJ_2, fo.ipw_final_us);
        }

        /* ── 11. Ignition (hardware alarm) ──────────────────── */
        ignition_update(rpm, map_bap_kf_get_rel(&s_map_kf), synced);

        /* ── 12. EGT choke ──────────────────────────────────── */
        gpio_put(PIN_CHOKE, (faults & FAULT_EGT_CRITICAL) ? 1 : 0);

        /* ── 13. [2-STROKE] Oil pump ────────────────────────── */
        float oil_duty = (synced && rpm > 500.0f)
                         ? compute_oil_duty(rpm, sens.tps_pct)
                         : 30.0f;   /* safe duty while cranking */
        oil_pump_set_duty(oil_duty);

        /* ── 14. IAC PID ────────────────────────────────────── */
        float iac = 20.0f;
        if (synced && sens.tps_pct < 2.0f)
            iac = pid_update(&s_iac_pid, 800.0f, rpm);
        else
            { pid_reset(&s_iac_pid); iac = 20.0f; }
        iac_set_duty(iac);

        /* ── 15. Seqlock publish ────────────────────────────── */
        sl_begin();
        g_ecu.map_kpa         = map_est;
        g_ecu.map_kpa_dot     = map_dot;
        g_ecu.bap_kpa         = bap_est;
        g_ecu.altitude_m      = alt_m;
        g_ecu.density_ratio   = dens;
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
        g_ecu.ign_advance_deg = ignition_get_advance_deg();
        g_ecu.lambda_trim     = fo.lambda_trim;
        g_ecu.iac_duty        = iac;
        g_ecu.oil_pump_duty   = oil_duty;
        g_ecu.alpha_n_blend   = fo.alpha_n_blend;
        g_ecu.fail_mode       = fo.fail_mode;
        g_ecu.fault_flags     = faults;
        g_ecu.egt_emergency   = (faults & FAULT_EGT_CRITICAL) != 0;
        sl_end();

        /* ── 16. Loop timing ────────────────────────────────── */
        uint64_t el = time_us_64()-t0;
        if (el >= period_us)
            g_ecu.fault_flags |= FAULT_CORE_OVERRUN;
        else
            busy_wait_us(period_us-(uint32_t)el);
    }
    return 0;
}
