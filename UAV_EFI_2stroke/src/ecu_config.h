/**
 * @file    ecu_config.h  [2-STROKE]
 * @brief   RP2350 UAV-EFI – 2-stroke flat-twin configuration
 *
 * Engine cycle: power stroke every 360° (one crank revolution)
 * Reference engine: DLE-35RA twin, 35 cc total
 *
 * Key 2-stroke differences from 4-stroke
 * ───────────────────────────────────────
 *  - Cam sensor (PIN_CMP) NOT required – every TDC is compression TDC
 *  - Injection every revolution (360°), not every other (720°)
 *  - Oil injection pump (PIN_OIL_INJ_PWM) – separate lubrication
 *  - EGT thresholds lower (750°C warn, 850°C crit → 680/780°C)
 *  - Shorter IGN_DWELL_MS (2.5 ms → 1.8 ms) – less recharge time
 *  - VE table has resonance hump at ~4000 RPM (exhaust pipe scavenging)
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ── GPIO / Pin map ───────────────────────────────────────── */
#define PIN_CKP              2   // Crank Position  – PIO SM0  (no cam needed)
/* PIN_CMP NOT USED in 2-stroke – every TDC is firing TDC */

#define PIN_INJ_1            6   // Injector cyl 1
#define PIN_INJ_2            7   // Injector cyl 2

#define PIN_IGN_1           10   // Ignition coil 1 (IGBT gate)
#define PIN_IGN_2           11   // Ignition coil 2

#define PIN_IAC_PWM         14   // Idle Air Control servo
#define PIN_FUEL_PUMP       15   // Fuel pump relay
#define PIN_CHOKE           16   // Choke solenoid
#define PIN_ENGINE_KILL     17   // Kill relay

/* [2-STROKE] Oil injection pump – separate PWM-controlled pump */
#define PIN_OIL_INJ_PWM     13   // GP13 – PWM slice 6B, 50 Hz

/* ADC channels */
#define PIN_ADC_MAP         26
#define PIN_ADC_TPS         27
#define PIN_ADC_IAT         28
#define PIN_ADC_CLT         29   // Cylinder head temperature (not coolant)

/* SPI0 – BAP + EGT */
#define PIN_SPI0_SCK        18
#define PIN_SPI0_MOSI       19
#define PIN_SPI0_MISO       20
#define PIN_SPI0_CS_BAP     21
#define PIN_SPI0_CS_EGT     22

/* CAN0 – DroneCAN */
#define PIN_CAN_TX          23
#define PIN_CAN_RX          24

/* UART1 */
#define PIN_UART_TX          4
#define PIN_UART_RX          5
#define UART_BAUD           115200

/* ── Engine constants ──────────────────────────────────────── */
#define ENGINE_CYL              2
#define ENGINE_STROKE           2       // ← 2-STROKE
#define ENGINE_DISPLACEMENT_CC  35.0f   // DLE-35RA total cc

/* 2-stroke trigger: 12-1 (simpler, fires every revolution) */
#define TRIGGER_TEETH_TOTAL     12
#define TRIGGER_TEETH_MISSING   1

/* Injector */
#define INJ_FLOW_RATE_CC_MIN    55.0f
#define INJ_DEAD_TIME_US        750.0f
#define FUEL_PRESSURE_BAR       3.0f

/* Ignition – shorter dwell for 2-stroke */
#define IGN_DWELL_MS            1.8f    // ← shorter than 4-stroke (2.5 ms)
#define IGN_MIN_ADVANCE_DEG     0.0f
#define IGN_MAX_ADVANCE_DEG     30.0f   // ← lower max (2-strokes more knock-sensitive)

/* Oil injection ratio */
#define OIL_RATIO_IDLE_PCT      3.0f    // 3 % oil at idle (33:1)
#define OIL_RATIO_WOT_PCT       2.0f    // 2 % oil at WOT  (50:1)

/* ADC */
#define ADC_VREF                3.3f
#define ADC_RESOLUTION          4096.0f
#define STOICH_AFR              14.7f

/* Loop rates */
#define CONTROL_LOOP_HZ         2000
#define COMMS_LOOP_HZ           100

/* ── ISA atmospheric constants ─────────────────────────────── */
#define ISA_P0_PA               101325.0f
#define ISA_T0_K                288.15f
#define ISA_LAPSE_K_PER_M       0.0065f
#define ISA_G_M_S2              9.80665f
#define R_AIR_J_KGK             287.05f
#define UAV_ALT_MAX_M           5000.0f

/* ── EGT protection – lower thresholds for 2-stroke ────────── */
#define EGT_WARN_DEGC           680.0f  // ← lower than 4-stroke (750°C)
#define EGT_CRITICAL_DEGC       780.0f  // ← lower than 4-stroke (850°C)
#define EGT_ENRICH_PCT_PER_50C  7.0f    // ← more aggressive enrichment
#define EGT_SENSOR_FAIL_LO     -100.0f

/* Alpha-N / Speed-Density blend */
#define ALPHA_N_RPM_LOW         2000.0f
#define ALPHA_N_RPM_HIGH        3500.0f

/* Fail-functional fallbacks */
#define FF_MAP_KPA              65.0f
#define FF_IAT_DEGC             20.0f
#define FF_IPW_FIXED_US         2000.0f // ← shorter (smaller engine)

/* ── Fault flags ───────────────────────────────────────────── */
typedef enum {
    FAULT_NONE          = 0x0000,
    FAULT_MAP_SENSOR    = 0x0001,
    FAULT_IAT_SENSOR    = 0x0002,
    FAULT_CHT_SENSOR    = 0x0004,
    FAULT_BAP_SENSOR    = 0x0008,
    FAULT_EGT_SENSOR    = 0x0010,
    FAULT_EGT_WARN      = 0x0020,
    FAULT_EGT_CRITICAL  = 0x0040,
    FAULT_NO_CRANK      = 0x0080,
    FAULT_CORE_OVERRUN  = 0x0100,
    FAULT_OIL_PUMP      = 0x0200,   // [2-STROKE] oil pump fault
} FaultFlags_t;

typedef enum {
    FF_NORMAL    = 0,
    FF_MAP_FIXED = 1,
    FF_IAT_FIXED = 2,
    FF_EGT_ENRICH= 3,
    FF_EMERGENCY = 4,
} FailMode_t;

/* ── Shared ECU state ──────────────────────────────────────── */
typedef struct __attribute__((packed)) {
    volatile uint32_t seq;

    float   map_kpa;
    float   map_kpa_dot;
    float   iat_degC;
    float   cht_degC;
    float   tps_pct;
    float   lambda;

    float   bap_kpa;
    float   altitude_m;
    float   density_ratio;
    float   map_rel_kpa;
    float   egt_degC;

    /* 2-stroke: crank angle 0-359° */
    float    rpm;
    uint16_t crank_angle_deg;
    bool     engine_sync;

    float   air_mass_mg;
    float   ipw_us;
    float   ign_advance_deg;
    float   lambda_trim;
    float   iac_duty;
    float   oil_pump_duty;   // [2-STROKE]

    float       alpha_n_blend;
    FailMode_t  fail_mode;
    uint16_t    fault_flags;
    bool        egt_emergency;
} EcuState_t;

extern volatile EcuState_t g_ecu;
