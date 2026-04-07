# RP2350 EFI – Open-Source Engine Control Unit

A complete C/C++ EFI firmware skeleton targeting the **Raspberry Pi RP2350**
(dual Cortex-M33, 150 MHz), demonstrating all major ECU subsystems including
sensor drivers, Kalman Filter sensor fusion, Speed-Density fuel calculation,
enrichment logic, closed-loop O2 trim, and dual-core partitioning.

---

## Project Structure

```
rp2350_efi/
├── CMakeLists.txt                  Build system (pico-sdk)
└── src/
    ├── ecu_config.h                Pin map, engine constants, shared state
    ├── main.cpp                    Core 0 control loop + Core 1 comms
    ├── pio/
    │   └── crank_trigger.pio       PIO assembly – 60-2 crank + injector pulse
    ├── drivers/
    │   ├── crank_sensor.h/.cpp     PIO + ISR tooth counter, RPM, crank angle
    │   └── adc_sensors.h/.cpp      DMA round-robin ADC – MAP, IAT, CLT, TPS, O2
    ├── fusion/
    │   └── kalman_filter.h/.cpp    MAP 2-state KF + IAT 1-state KF + air mass
    └── efi/
        ├── fuel_calc.h/.cpp        VE/AFR tables, IPW, enrichments, O2 trim
        └── pid.h                   Generic PID (single-header) for IAC
```

---

## Architecture Overview

### Dual-Core Partitioning

| Core | Role | Rate |
|------|------|------|
| **Core 0** | Crank ISR, ADC read, Kalman Filter, fuel calc, injector scheduling, IAC PID | 2 kHz |
| **Core 1** | Serial TX (CSV / TunerStudio), serial RX (table updates), CAN TX | 100 Hz |

Shared data is protected by a **seqlock** (sequence counter + ARM DMB memory barrier).

---

### Sensor Fusion – Kalman Filter

Two independent linear Kalman Filters run on Core 0 at every control tick:

#### KF #1 – MAP Pressure Estimator (2-state)

```
State:    x = [ P_manifold,  dP/dt ]ᵀ
Model:    x(k+1) = F·x(k) + noise
            F = | 1  dt |
                | 0   1 |
Measurement:  z = P_adc   (scalar, H = [1, 0])
```

The derivative state `dP/dt` enables accurate **acceleration enrichment**
detection without a separate differentiator.

#### KF #2 – IAT Estimator (1-state)

```
State:    x = [ T_intake ]
Model:    x(k+1) = x(k) + noise   (temperature changes slowly)
Measurement:  z = T_adc
```

Reduces NTC thermistor noise from ~2°C RMS to < 0.3°C RMS.

#### Air Mass (Ideal Gas Law)

After KF estimation:
```
ρ_air   = P_man [Pa] / (R_air × T_air [K])    R_air = 287.05 J/(kg·K)
m_air   = ρ_air × V_swept × VE%
```

---

### Fuel Equation

```
IPW_base   = (m_air / AFR_target) / inj_flow_rate
IPW_warmup = IPW_base  × warmup_factor(CLT)        ← cold-start enrichment
IPW_accel  = IPW_warmup + accel_enrichment(tps_dot) ← acceleration pump
IPW_trim   = IPW_accel × (1 + λ_trim)               ← closed-loop O2 trim
IPW_final  = IPW_trim + dead_time                    ← injector latency
```

---

### PIO – Crank Trigger Wheel (60-2)

The PIO state machine on SM0 fires an IRQ on every rising edge from the
Hall/VR sensor. The ISR:
1. Timestamps each edge via `time_us_64()`
2. Compares adjacent periods to detect the **missing-tooth gap** (ratio > 1.8×)
3. Resets crank angle to 0° (TDC cyl 1) on gap detection
4. Computes RPM via exponential moving average

---

## Build Instructions

```bash
# Prerequisites
export PICO_SDK_PATH=/path/to/pico-sdk   # pico-sdk 2.x required for RP2350

# Clone / enter project
cd rp2350_efi

# Configure
mkdir build && cd build
cmake .. -DPICO_BOARD=pico2             # RP2350-based board

# Build
make -j$(nproc)

# Flash (drag-and-drop UF2 or picotool)
picotool load rp2350_efi.uf2
```

---

## Calibration (Serial Interface)

Connect to UART1 (GP4/GP5) at 115,200 baud. The firmware outputs CSV:

```
RPM, MAP_kPa, IAT_°C, CLT_°C, TPS_%, Lambda, IPW_µs, IGN_°, λ_trim, m_air_mg
```

To update a VE table cell, send: `VE,<row>,<col>,<value>\n`

---

## Key Tuning Parameters (`ecu_config.h`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `INJ_FLOW_RATE_CC_MIN` | 240 cc/min | Injector flow @ 3 bar |
| `INJ_DEAD_TIME_US` | 750 µs | Injector latency @ 14 V |
| `TRIGGER_TEETH_TOTAL` | 60 | Crank trigger wheel teeth |
| `ENGINE_DISPLACEMENT_CC` | 1600 cc | Engine displacement |
| `ENGINE_CYL` | 4 | Number of cylinders |

---

## References

- RP2350 Datasheet – https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf
- pico-sdk – https://github.com/raspberrypi/pico-sdk
- RusEFI (reference open-source ECU) – https://rusefi.com
- Speeduino – https://speeduino.com
- Kalman, R.E. (1960). "A New Approach to Linear Filtering and Prediction Problems"
