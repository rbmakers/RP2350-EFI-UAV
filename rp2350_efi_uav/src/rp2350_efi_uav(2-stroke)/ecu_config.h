/**
 * @file    ecu_config.h  (2-STROKE VARIANT)
 * @brief   RP2350 EFI – UAV 2-Stroke Piston Engine
 *
 * 2-Stroke vs. 4-Stroke fundamental differences
 * ───────────────────────────────────────────────
 *
 *  Firing cycle:
 *    4-stroke → power stroke every 720° (2 crank revolutions)
 *    2-stroke → power stroke every 360° (1 crank revolution)  ← THIS FILE
 *
 *  Valve/port mechanism:
 *    4-stroke → poppet valves (intake + exhaust), cam timing critical
 *    2-stroke → reed valves / piston-controlled ports, no camshaft needed
 *
 *  Cam sensor:
 *    4-stroke → required to distinguish compression from exhaust TDC
 *    2-stroke → NOT NEEDED; every TDC is a firing TDC
 *
 *  Injection timing:
 *    4-stroke → inject once per 720° (during intake stroke)
 *    2-stroke → inject once per 360° (every revolution)
 *               Typically into the crankcase / reed valve area
 *               (direct-injection 2-strokes inject into cylinder)
 *
 *  Lubrication:
 *    4-stroke → separate oil sump (no fuel mixing)
 *    2-stroke → oil premixed in fuel OR separate oil-injection pump
 *               [NEW] PIN_OIL_INJ – oil injection pump PWM control
 *
 *  VE characteristics:
 *    2-stroke VE is strongly influenced by exhaust pipe resonance (tuned pipe).
 *    Peak VE occurs at the pipe's resonant RPM; outside this band VE drops sharply.
 *    The VE table shape is therefore a pronounced 'hump' vs. the smooth hill of a 4-stroke.
 *
 *  Ignition:
 *    2-stroke → fires EVERY revolution (360°), not every other (720°)
 *    Advance map shape differs: 2-strokes are more sensitive to advance
 *    because there is less time between ignition and power stroke.
 *
 *  Typical UAV 2-stroke engines: DLE-20, DLE-35RA, Zenoah G26, OS GT15HZ
 *
 * Board:  RB-RP2354A (Curio form factor)
 * SDK:    pico-sdk 2.x
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ─────────────────────────────────────────────────────────────
 *  GPIO / Pin Assignments
 * ───────────────────────────────────────────────────────────── */

#define PIN_CKP              2   // Crank Position – PIO SM0
// NOTE: No PIN_CMP (cam sensor) – not needed for 2-stroke

#define PIN_INJ_1            6   // Injector cylinder 1
#define PIN_INJ_2            7   // Injector cylinder 2 (if twin)

#define PIN_IGN_1           10   // Ignition coil driver cylinder 1
#define PIN_IGN_2           11   // Ignition coil driver cylinder 2

#define PIN_IAC_PWM         14   // Idle air / throttle servo PWM
#define PIN_FUEL_PUMP       15   // Fuel pump relay
#define PIN_CHOKE           16   // Choke solenoid (cold restart)
#define PIN_ENGINE_KILL     17   // Engine kill relay

// [2-STROKE NEW] Oil injection pump (separate lubrication system)
// PWM duty controls oil pump stroke rate → oil delivery proportional to RPM
#define PIN_OIL_INJ_PWM     13   // Oil injection pump PWM (GP13, PWM slice 6B)

// ADC inputs (GP26-GP29)
#define PIN_ADC_MAP         26   // ADC0 – MAP
#define PIN_ADC_TPS         27   // ADC1 – TPS
#define PIN_ADC_IAT         28   // ADC2 – IAT
#define PIN_ADC_CLT         29   // ADC3 – CHT (cylinder head temperature)

// SPI0 – BAP (MS5611) + EGT (MAX31855)
#define PIN_SPI0_SCK        18
#define PIN_SPI0_MOSI       19
#define PIN_SPI0_MISO       20
#define PIN_SPI0_CS_BAP     21
#define PIN_SPI0_CS_EGT     22

// CAN0 – DroneCAN
#define PIN_CAN_TX          23
#define PIN_CAN_RX          24

// UART1 – serial tuning
#define PIN_UART_TX          4
#define PIN_UART_RX          5
#define UART_BAUD           115200


/* ─────────────────────────────────────────────────────────────
 *  Engine Configuration  ← KEY DIFFERENCES FROM 4-STROKE
 * ───────────────────────────────────────────────────────────── */

#define ENGINE_CYL              2       // twin-cylinder (e.g. DLE-35RA twin)
#define ENGINE_STROKE           2       // ← 2-STROKE
#define ENGINE_DISPLACEMENT_CC  35.0f   // total cc (e.g. DLE-35RA = 35 cc)

// 2-stroke trigger wheel: simpler – 12-1 (12 teeth, 1 missing)
// Fewer teeth acceptable because 2-stroke fires every revolution
// so crank position reference is needed once per 360° not 720°
#define TRIGGER_TEETH_TOTAL     12
#define TRIGGER_TEETH_MISSING   1
// Angle increment per real tooth: 360° / 11 ≈ 32.73°
#define TOOTH_ANGLE_DEG_2S      (360.0f / (TRIGGER_TEETH_TOTAL - TRIGGER_TEETH_MISSING))

// Injector (reed-valve crankcase injection)
#define INJ_FLOW_RATE_CC_MIN    55.0f   // smaller engine → smaller injector
#define INJ_DEAD_TIME_US        750.0f
#define FUEL_PRESSURE_BAR       3.0f

// Ignition dwell – shorter dwell acceptable in 2-stroke
// (coil has less time between firing events at high RPM)
#define IGN_DWELL_MS            1.8f
#define IGN_MIN_ADVANCE_DEG     0.0f
#define IGN_MAX_ADVANCE_DEG     30.0f   // 2-strokes more sensitive to over-advance

// [2-STROKE] Oil injection ratio (volume %)
// Typical premix equivalent: 50:1 = 2 % oil, 40:1 = 2.5 %
// At idle run slightly rich on oil; at WOT reduce to protect ring seals
#define OIL_RATIO_IDLE_PCT      3.0f    // 3 % oil at idle
#define OIL_RATIO_WOT_PCT       2.0f    // 2 % oil at WOT

// ADC
#define ADC_VREF                3.3f
#define ADC_RESOLUTION          4096.0f
#define STOICH_AFR              14.7f

// Timing
#define CONTROL_LOOP_HZ         2000
#define COMMS_LOOP_HZ           100

/* ─────────────────────────────────────────────────────────────
 *  UAV Atmospheric Constants (same as 4-stroke)
 * ───────────────────────────────────────────────────────────── */

#define ISA_P0_PA               101325.0f
#define ISA_T0_K                288.15f
#define ISA_LAPSE_K_PER_M       0.0065f
#define ISA_G_M_S2              9.80665f
#define R_AIR_J_KGK             287.05f
#define UAV_ALT_MAX_M           5000.0f

// EGT – 2-strokes run hotter; lower warning threshold
#define EGT_WARN_DEGC           680.0f      // lower than 4-stroke (750°C)
#define EGT_CRITICAL_DEGC       780.0f      // lower than 4-stroke (850°C)
#define EGT_ENRICH_PCT_PER_50C  7.0f        // more aggressive enrichment
#define EGT_SENSOR_FAIL_LO     -100.0f

// Alpha-N / Speed-Density blend (same thresholds)
#define ALPHA_N_RPM_LOW         2000.0f
#define ALPHA_N_RPM_HIGH        3500.0f

// Fail-Functional fallback
#define FF_MAP_KPA              65.0f
#define FF_IAT_DEGC             20.0f
#define FF_IPW_FIXED_US         2000.0f     // shorter than 4-stroke (smaller engine)


/* ─────────────────────────────────────────────────────────────
 *  Fault Flags & Fail Mode (same as 4-stroke)
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
    FAULT_OIL_PUMP      = 0x0200,   // [2-STROKE NEW] oil pump failure
} FaultFlags_t;

typedef enum {
    FF_NORMAL    = 0,
    FF_MAP_FIXED = 1,
    FF_IAT_FIXED = 2,
    FF_EGT_ENRICH= 3,
    FF_EMERGENCY = 4,
} FailMode_t;


/* ─────────────────────────────────────────────────────────────
 *  Shared ECU State
 * ───────────────────────────────────────────────────────────── */

typedef struct __attribute__((packed)) {
    volatile uint32_t seq;

    // Sensor readings
    float   map_kpa;
    float   map_kpa_dot;
    float   iat_degC;
    float   cht_degC;
    float   tps_pct;
    float   lambda;

    // UAV atmospheric
    float   bap_kpa;
    float   altitude_m;
    float   density_ratio;
    float   map_rel_kpa;
    float   egt_degC;

    // Crank (2-stroke: 0–359° only, every rev is firing rev)
    float    rpm;
    uint16_t crank_angle_deg;   // 0–359°
    bool     engine_sync;

    // EFI outputs
    float   air_mass_mg;
    float   ipw_us;
    float   ign_advance_deg;
    float   lambda_trim;
    float   iac_duty;
    float   oil_pump_duty;      // [2-STROKE NEW] oil injection pump duty [%]

    // Status
    float       alpha_n_blend;
    FailMode_t  fail_mode;
    uint16_t    fault_flags;
    bool        egt_emergency;
} EcuState_t;

extern volatile EcuState_t g_ecu;
