/**
 * @file    ecu_config.h
 * @brief   RP2350 EFI – UAV Piston Engine variant
 *          (Modified from automotive baseline for UAV deployment)
 *
 * Key UAV modifications vs. automotive baseline
 * ─────────────────────────────────────────────
 *  [NEW] BAP  – Barometric Absolute Pressure sensor (altitude reference)
 *  [NEW] EGT  – Exhaust Gas Temperature via thermocouple + MAX31855
 *  [NEW] Altitude density-ratio correction to fuel equation
 *  [NEW] Alpha-N / Speed-Density automatic switching
 *  [NEW] Fail-Functional (not Limp) – engine tries to keep running
 *  [NEW] DroneCAN (UAVCAN v1) telemetry stub on CAN0
 *  [MOD] MAP compensation now references BAP, not sea-level assumption
 *  [MOD] EcuState_t extended with UAV fields
 *
 * Board:  RB-RP2354A (Curio flight-controller form factor)
 * Engine: flat-twin 4-stroke (e.g. DLE-55, DA-60, Rotax 277)
 * SDK:    pico-sdk 2.x
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/* ─────────────────────────────────────────────────────────────
 *  GPIO / Pin Assignments
 * ───────────────────────────────────────────────────────────── */

#define PIN_CKP              2   // Crank Position – PIO SM0
#define PIN_CMP              3   // Cam Position   – PIO SM1

#define PIN_INJ_1            6
#define PIN_INJ_2            7
#define PIN_INJ_3            8
#define PIN_INJ_4            9

#define PIN_IGN_1           10
#define PIN_IGN_2           11

#define PIN_IAC_PWM         14
#define PIN_FUEL_PUMP       15
#define PIN_CHOKE           16   // [UAV] choke solenoid – cold-start enrichment
#define PIN_ENGINE_KILL     17   // [UAV] engine kill relay – last-resort failsafe

// ADC (GP26-GP29 = ADC0-ADC3)
#define PIN_ADC_MAP         26   // ADC0 – manifold absolute pressure
#define PIN_ADC_TPS         27   // ADC1 – throttle position
#define PIN_ADC_IAT         28   // ADC2 – intake air temperature (NTC)
#define PIN_ADC_CLT         29   // ADC3 – cylinder head temperature (NTC)

// [UAV] SPI0 – BAP (MS5611) + EGT (MAX31855)
#define PIN_SPI0_SCK        18
#define PIN_SPI0_MOSI       19
#define PIN_SPI0_MISO       20
#define PIN_SPI0_CS_BAP     21   // MS5611 chip-select
#define PIN_SPI0_CS_EGT     22   // MAX31855 chip-select

// [UAV] CAN0 – DroneCAN / UAVCAN v1 telemetry
#define PIN_CAN_TX          23
#define PIN_CAN_RX          24

// UART1 – serial tuning
#define PIN_UART_TX          4
#define PIN_UART_RX          5
#define UART_BAUD           115200


/* ─────────────────────────────────────────────────────────────
 *  Engine Configuration
 * ───────────────────────────────────────────────────────────── */

#define ENGINE_CYL              2           // flat-twin (e.g. DA-60)
#define ENGINE_STROKE           4
#define ENGINE_DISPLACEMENT_CC  60.0f       // total cc

#define TRIGGER_TEETH_TOTAL     60
#define TRIGGER_TEETH_MISSING   2

#define INJ_FLOW_RATE_CC_MIN    75.0f       // cc/min @ 3 bar
#define INJ_DEAD_TIME_US        800.0f      // latency @ 12 V battery
#define FUEL_PRESSURE_BAR       3.0f

#define IGN_DWELL_MS            2.5f
#define IGN_MIN_ADVANCE_DEG     0.0f
#define IGN_MAX_ADVANCE_DEG     35.0f

#define ADC_VREF                3.3f
#define ADC_RESOLUTION          4096.0f
#define STOICH_AFR              14.7f

#define CONTROL_LOOP_HZ         2000
#define COMMS_LOOP_HZ           100
#define ADC_SAMPLE_HZ           5000


/* ─────────────────────────────────────────────────────────────
 *  [UAV] Altitude & Atmospheric Constants
 * ───────────────────────────────────────────────────────────── */

// ISA sea-level reference values
#define ISA_P0_PA               101325.0f   // sea-level pressure [Pa]
#define ISA_T0_K                288.15f     // sea-level temperature [K]
#define ISA_LAPSE_K_PER_M       0.0065f     // temperature lapse rate [K/m]
#define ISA_G_M_S2              9.80665f
#define R_AIR_J_KGK             287.05f     // specific gas constant, dry air

#define UAV_ALT_MAX_M           5000.0f     // maximum design altitude [m]

// EGT protection
#define EGT_WARN_DEGC           750.0f      // begin enrichment above this
#define EGT_CRITICAL_DEGC       850.0f      // maximum emergency enrichment
#define EGT_ENRICH_PCT_PER_50C  5.0f        // % extra fuel per 50°C above warn
#define EGT_SENSOR_FAIL_LO      -100.0f     // open-circuit detection threshold

// Alpha-N ↔ Speed-Density blend RPM thresholds
// Below LOW  → full Alpha-N  (TPS-based, ignores MAP lag at high altitude)
// Above HIGH → full Speed-Density (MAP-based, more accurate at steady state)
#define ALPHA_N_RPM_LOW         2000.0f
#define ALPHA_N_RPM_HIGH        3500.0f

// Fail-Functional fallback values (used when sensor faults detected)
#define FF_MAP_KPA              65.0f       // ~2000 m ISA equivalent
#define FF_IAT_DEGC             20.0f
#define FF_IPW_FIXED_US         3500.0f     // emergency fixed IPW


/* ─────────────────────────────────────────────────────────────
 *  Fault Flags (bitmask)
 * ───────────────────────────────────────────────────────────── */

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

// UAV fail-functional strategy
// Priority: keep engine running > degrade gracefully > kill only as last resort
typedef enum {
    FF_NORMAL       = 0,    // all sensors valid, nominal operation
    FF_MAP_FIXED    = 1,    // MAP failed → fixed BAP-derived estimate
    FF_IAT_FIXED    = 2,    // IAT failed → fixed 20°C default
    FF_EGT_ENRICH   = 3,    // EGT warn → progressive enrichment
    FF_EMERGENCY    = 4,    // EGT critical → max enrichment, alert GCS
} FailMode_t;


/* ─────────────────────────────────────────────────────────────
 *  Shared ECU State  (Core 0 writes, Core 1 reads via seqlock)
 * ───────────────────────────────────────────────────────────── */

typedef struct __attribute__((packed)) {
    volatile uint32_t seq;          // seqlock – odd during write

    // Sensor readings (Kalman-filtered)
    float   map_kpa;                // manifold absolute pressure [kPa]
    float   map_kpa_dot;            // dMAP/dt [kPa/s]
    float   iat_degC;
    float   cht_degC;               // cylinder head temperature [°C]
    float   tps_pct;
    float   lambda;

    // [UAV] Atmospheric state
    float   bap_kpa;                // barometric absolute pressure [kPa]
    float   altitude_m;             // ISA-derived altitude [m]
    float   density_ratio;          // σ = ρ/ρ₀ (1.0 at sea level, <1 at altitude)
    float   map_rel_kpa;            // MAP - BAP [kPa]  (true manifold depression)
    float   egt_degC;               // exhaust gas temperature [°C]

    // Crank state
    float   rpm;
    uint16_t crank_angle_deg;
    bool    engine_sync;

    // EFI outputs
    float   air_mass_mg;
    float   ipw_us;
    float   ign_advance_deg;
    float   lambda_trim;
    float   iac_duty;

    // [UAV] Status
    float       alpha_n_blend;      // 0 = Speed-Density, 1 = Alpha-N
    FailMode_t  fail_mode;
    uint16_t    fault_flags;
    bool        egt_emergency;
} EcuState_t;

extern volatile EcuState_t g_ecu;
