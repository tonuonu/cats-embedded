# CATS Hardware Documentation

## Hardware Repository
- **GitHub**: https://github.com/catsystems/cats-hardware
- **Schematics (PDF)**:
  - Vega: https://github.com/catsystems/cats-hardware/raw/main/CATS-Vega/CATS-Vega.pdf
  - Ground Station: https://github.com/catsystems/cats-hardware/raw/main/CATS-GS/CATS-GS.pdf
- **3D CAD (STEP)**:
  - Vega: https://github.com/catsystems/cats-hardware/tree/main/CATS-Vega/3D
  - Ground Station: https://github.com/catsystems/cats-hardware/tree/main/CATS-GS/3D

## CATS Vega Flight Computer

### Specifications
| Parameter | Value |
|-----------|-------|
| Microcontroller | STM32F411 (Cortex-M4, 100MHz) |
| Flash Memory | 16MB (W25Q series) |
| Input Voltage | 7-24V |
| Power Consumption | 100mA |
| IMU | LSM6DSO32 (±32g accel, ±2000 dps gyro) |
| Barometer | MS5607 |
| GNSS | u-blox MAX-M10S |
| Radio | SX1280 @ 2.4GHz ISM band |
| Radio Power | Up to 1W |
| Radio Range | 10+ km (flight tested) |
| Pyro Channels | 2 channels, Vin switching, 5A max |
| Servo Channels | 2 channels, 5V, 3A max |
| Additional I/O | UART + 1 GPIO |
| Size | 100 x 33 x 21 mm (without antenna) |

### Schematic Modules (Altium)
The hardware design is modular with separate schematic sheets:
- **Overview** - Top-level block diagram
- **MCU** - STM32F411 microcontroller
- **Frontend** - Power input, main connectors
- **PM** - Power management, voltage regulators
- **IMU** - LSM6DSO32 accelerometer/gyroscope
- **Baro** - MS5607 barometric pressure sensor
- **GNSS** - MAX-M10S GPS/GNSS module
- **Storage** - W25Q 16MB SPI flash
- **USB** - USB-C interface with ESD protection
- **Pyro** - Pyro channel drivers (2x)
- **Pyro_Enable** - Pyro arming circuit
- **Radio** - SX1280 2.4GHz transceiver
- **Radio_MCU** - STM32G0 dedicated telemetry processor
- **RF** - RF frontend, antenna matching
- **Telemetry** - Telemetry interface connector

### Key Hardware Interfaces (firmware mapping)
- **SPI1**: IMU (LSM6DSO32) + Barometer (MS5607)
  - IMU CS: PB0
  - Baro CS: PB1
- **SPI2**: Flash storage (W25Q)
- **UART**: GNSS module, external I/O
- **USB**: TinyUSB device (CDC + MSC)
- **Timers**: PWM for buzzer, servos
- **ADC**: Battery voltage monitoring
- **GPIO**: Status LED (PC14), Test button (PB13)

### Telemetry System
- Uses FHSS (Frequency-Hopping Spread Spectrum) at 2.4GHz
- Binding via configurable "link phrase"
- 10Hz update rate
- Transmits: altitude, velocity, GNSS coords, battery, flight phase
- Dedicated STM32G0 handles radio communication

## CATS Ground Station

### Specifications
| Parameter | Value |
|-----------|-------|
| Microcontroller | ESP32-S2 |
| Flash Memory | 4MB |
| Battery | Li-Ion 18650 |
| Power Consumption | 60mA |
| Charging Current | 500mA (USB-C) |
| Display | Sharp LS027B7DH01 (transflective) |
| Radio | Dual SX1280 (can track 2 Vegas) |
| Radio Range | 10+ km (flight tested) |
| IMU | LSM6DSRTR |
| Magnetometer | QMC5883L (compass) |
| GNSS | ATGM336H-5N |

### Schematic Modules (Altium)
- **Overview** - Top-level block diagram
- **MCU** - ESP32-S2 microcontroller
- **PM** - Power management, battery charging
- **USB** - USB-C interface
- **GNSS** - ATGM336H-5N GPS module
- **Sensors** - LSM6DSRTR IMU, QMC5883L magnetometer
- **Radio** - Dual SX1280 transceivers
- **Radio_MCU** - STM32G0 telemetry processor
- **Telemetry** - Telemetry interface

### Features
- Track 2 independent Vega boards OR dual-antenna tracking of 1 Vega
- Transflective display readable in sunlight
- Integrated compass for rocket tracking
- On-board flight data logging
- Built-in data plotter for post-flight review
- Remote event triggering for testing
- USB-C charging

## Telemetry Module (separate PCB)
- MCU: STM32G071RB
- Handles radio communication for both Vega and Ground Station
- Connected via internal UART
