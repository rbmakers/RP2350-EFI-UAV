/**
 * @file    ecu_config.h  [4-STROKE]
 * @brief   RP2350 UAV-EFI – 4-stroke flat-twin configuration
 *
 * Engine cycle: power stroke every 720° (two crank revolutions)
 * Reference engine: DA-60 / Rotax 582 class flat-twin, 60 cc total
 *
 * Dual-core assignment
 *   Core 0 → 2 kHz deterministic control (ADC, KF, fuel, ignition, ISR)
 *   Core 1 → 100 Hz housekeeping  (UART CSV, DroneCAN stub, tuning RX)
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ── GPIO / Pin map ───────────────────────────────────────── */
#define PIN_CKP              2   // Crank Position  – PIO SM0 (Hall/VR)
#define PIN_CMP              3   // Cam Position    – PIO SM1 (1-tooth)

#define PIN_INJ_1            6   // Injector cyl 1 (MOSFET open-drain)
#define PIN_INJ_2            7   // Injector cyl 2

#define PIN_IGN_1           10   // Ignition coil 1 (IGBT gate, active-HIGH=charge)
#define PIN_IGN_2           11   // Ignition coil 2

#define PIN_IAC_PWM         14   // Idle Air Control servo – PWM 50 Hz
#define PIN_FUEL_PUMP       15   // Fuel pump relay (12 V coil)
#define PIN_CHOKE           16   // Choke solenoid  (EGT emergency enrichment)
#define PIN_ENGINE_KILL     17   // Kill relay      (last-resort only)

/* ADC channels – GP26-GP29 = ADC0-ADC3 */
#define PIN_ADC_MAP         26   // Manifold absolute pressure  (MPX4250A)
#define PIN_ADC_TPS         27   // Throttle position sensor    (pot)
#define PIN_ADC_IAT         28   // Intake air temperature      (NTC 10 kΩ)
#define PIN_ADC_CLT         29   // Cylinder head temperature   (NTC 10 kΩ)

/* SPI0 – BAP (MS5611) + EGT (MAX31855) */
#define PIN_SPI0_SCK        18
#define PIN_SPI0_MOSI       19
#define PIN_SPI0_MISO       20
#define PIN_SPI0_CS_BAP     21   // MS5611  CS
#define PIN_SPI0_CS_EGT     22   // MAX31855 CS

/* CAN0 – DroneCAN / UAVCAN v1 */
#define PIN_CAN_TX          23
#define PIN_CAN_RX          24

/* UART1 – serial tuning link */
#define PIN_UART_TX          4
#define PIN_UART_RX          5
#define UART_BAUD           115200

/* ── Engine constants ──────────────────────────────────────── */
#define ENGINE_CYL              2       // flat-twin
#define ENGINE_STROKE           4       // 4-stroke
#define ENGINE_DISPLACEMENT_CC  60.0f   // total cc

/* Trigger wheel: 60-2  (60 teeth, 2 missing = TDC reference) */
#define TRIGGER_TEETH_TOTAL     60
#define TRIGGER_TEETH_MISSING   2

/* Injector: Hana R2141-class, small UAV injector */
#define INJ_FLOW_RATE_CC_MIN    75.0f   // cc/min @ 3 bar
#define INJ_DEAD_TIME_US        800.0f  // latency @ 12 V battery
#define FUEL_PRESSURE_BAR       3.0f

/* Ignition */
#define IGN_DWELL_MS            2.5f    // coil charge time
#define IGN_MIN_ADVANCE_DEG     0.0f
#define IGN_MAX_ADVANCE_DEG     35.0f   // BTDC limit

/* ADC */
#define ADC_VREF                3.3f
#define ADC_RESOLUTION          4096.0f
#define STOICH_AFR              14.7f

/* Loop rates */
#define CONTROL_LOOP_HZ         2000
#define COMMS_LOOP_HZ           100

/* ── Atmospheric constants (ISA) ──────────────────────────── */
#define ISA_P0_PA               101325.0f
#define ISA_T0_K                288.15f
#define ISA_LAPSE_K_PER_M       0.0065f
#define ISA_G_M_S2              9.80665f
#define R_AIR_J_KGK             287.05f
#define UAV_ALT_MAX_M           5000.0f

/* ── EGT protection (4-stroke thresholds) ─────────────────── */
#define EGT_WARN_DEGC           750.0f
#define EGT_CRITICAL_DEGC       850.0f
#define EGT_ENRICH_PCT_PER_50C  5.0f
#define EGT_SENSOR_FAIL_LO     -100.0f

/* ── Alpha-N / Speed-Density blend ────────────────────────── */
#define ALPHA_N_RPM_LOW         2000.0f
#define ALPHA_N_RPM_HIGH        3500.0f

/* ── Fail-functional fallbacks ────────────────────────────── */
#define FF_MAP_KPA              65.0f
#define FF_IAT_DEGC             20.0f
#define FF_IPW_FIXED_US         3500.0f

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
} FaultFlags_t;

typedef enum {
    FF_NORMAL    = 0,
    FF_MAP_FIXED = 1,
    FF_IAT_FIXED = 2,
    FF_EGT_ENRICH= 3,
    FF_EMERGENCY = 4,
} FailMode_t;

/* ── Shared ECU state (seqlock-protected) ─────────────────── */
typedef struct __attribute__((packed)) {
    volatile uint32_t seq;

    float   map_kpa;
    float   map_kpa_dot;
    float   iat_degC;
    float   cht_degC;
    float   tps_pct;
    float   lambda;

    /* UAV atmospheric */
    float   bap_kpa;
    float   altitude_m;
    float   density_ratio;
    float   map_rel_kpa;
    float   egt_degC;

    /* Crank – 4-stroke uses 0-719° window */
    float    rpm;
    uint16_t crank_angle_deg;   // 0-719
    bool     engine_sync;

    /* EFI outputs */
    float   air_mass_mg;
    float   ipw_us;
    float   ign_advance_deg;
    float   lambda_trim;
    float   iac_duty;

    /* Status */
    float       alpha_n_blend;
    FailMode_t  fail_mode;
    uint16_t    fault_flags;
    bool        egt_emergency;
} EcuState_t;

extern volatile EcuState_t g_ecu;
