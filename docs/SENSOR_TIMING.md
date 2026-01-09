# CATS Sensor Timing Analysis

## Bus Topology
```
SPI1 (PA5/PA6/PA7) ─┬─ LSM6DSO32 IMU (CS: PB0)
                    └─ MS5607 Baro  (CS: PB1)

SPI2 (PB10/PB14/PB15) ── W25Q Flash (CS: PB12)

I2C: Available on STM32F411 but NOT routed on PCB (HAL_I2C_MODULE disabled)
```

## RTOS Configuration
- **Tick rate**: 1000Hz (1ms resolution) - FreeRTOSConfig.h
- **Control loop**: 100Hz (CONTROL_SAMPLING_FREQ in globals.hpp)
- **Sensor task**: 200Hz, alternates baro temp/pressure + IMU read

## IMU (LSM6DSO32) Timing Problem

### Available ODRs (no 100Hz option!)
| Setting | ODR |
|---------|-----|
| 0x10 | 12.5 Hz |
| 0x20 | 26 Hz |
| 0x30 | 52 Hz |
| 0x40 | **104 Hz** (current) |
| 0x50 | 208 Hz |
| 0x60 | 416 Hz |

### Current Implementation
- ODR: 104Hz (sample every 9.615ms)
- Read: Direct register polling at 100Hz (10ms)
- No FIFO usage
- Timestamps: osKernelGetTickCount() (1ms resolution, RTOS jitter)

### Problem
- 104Hz sensor polled at 100Hz = 4 samples/sec phase drift
- Over 60s flight: ~240 samples with timing issues
- Variable latency: 0-9.6ms depending on phase alignment

### Solution (Issue #2)
- Enable LSM6DSO32 3KB FIFO in continuous mode
- FIFO provides hardware-precise 104Hz sampling
- Read FIFO via interrupt (INT1) or fast-poll (9ms loop)
- Derive timestamps: ts[n] = ts[0] + n * 9615µs

## Barometer (MS5607) Timing

### Conversion Times by OSR
| OSR | Time | Max Rate |
|-----|------|----------|
| 256 | 0.5ms | ~2000 Hz |
| 512 | 1.0ms | ~1000 Hz |
| 1024 | **2.1ms** | ~470 Hz |
| 2048 | 4.1ms | ~240 Hz |
| 4096 | 8.2ms | ~120 Hz |

### Current Implementation
- OSR1024 (2.1ms conversion)
- Alternating temp/pressure at 200Hz task
- Effective rate: 100Hz each
- Temperature read for pressure compensation

### Temperature Compensation Reality
- MS5607 outputs raw ADC values (D1=pressure, D2=temperature)
- Compensation is **software-calculated** using factory coefficients C1-C6
- Temperature compensates for **sensor die drift**, not ambient
- Die thermal time constant: minutes (PCB inside bay inside rocket)
- During flight (tens of seconds): die temp change < 1°C

### Optimization Opportunity
- Read temperature once at calibration (on launch pad)
- Cache dT value for entire flight
- Run pressure-only at 200-400Hz
- For ambient temperature: need external sensor (e.g., TMP117 on body)

## Kalman Filter
- **t_sampl**: 1/CONTROL_SAMPLING_FREQ = 0.01s (hardcoded)
- **Files**: orientation_filter.cpp, kalman_filter.cpp
- **104Hz impact**: 4% difference (0.01 vs 0.00962s) negligible vs sensor noise
- Options: update value, leave as-is, or make configurable

## Recording System
- **Flash**: W25Q 16MB with LittleFS
- **Buffer**: 256 bytes, 512-element queue
- **Decimation**: rec_speed_idx 0-9 (100Hz to 10Hz)
- **Throughput**: ~3.6 KB/sec at 100Hz - not a bottleneck

## External Temperature Sensor Notes
If adding external temperature (e.g., TMP117 via I2C):
- Mount on rocket body for airflow exposure
- Useful for: ambient logging, Mach calculation, air density
- NOT useful for MS5607 compensation (needs die temp)
- Would require enabling I2C in HAL and routing pins

## Open Issues
- [Issue #2: FIFO-based IMU sampling](https://github.com/tonuonu/cats-embedded/issues/2)
