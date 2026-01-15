# CATS Embedded - Project Overview

## Purpose
CATS (Control and Telemetry Systems) is open-source flight computer software for sounding rockets. The system includes:
- **CATS Vega** flight computer firmware
- **CATS Ground Station** for telemetry reception
- **Telemetry module** for RF communication

## Key Features
- Kalman filter altitude/velocity estimation
- Configurable events (8 actions per flight state transition)
- High-speed logging (up to 100 Hz)
- Telemetry with 10+ km range
- GNSS logging and transmission
- Accelerometric liftoff detection

## Repository Structure
```
cats-embedded/
├── flight_computer/    # Main flight computer firmware (STM32F411)
│   ├── src/           # Application source code
│   │   ├── tasks/     # FreeRTOS tasks
│   │   ├── drivers/   # Hardware drivers
│   │   ├── sensors/   # Sensor implementations
│   │   ├── control/   # Kalman filter, flight phases
│   │   ├── cli/       # Command line interface
│   │   └── target/    # Board-specific code (VEGA)
│   └── lib/           # Libraries (STM HAL, FreeRTOS, CMSIS, TinyUSB, LittleFS)
├── ground_station/     # Ground station firmware (ESP32)
├── telemetry/          # Telemetry module firmware
└── docs/               # Documentation
```

## Tech Stack
- **MCU**: STM32F411 (ARM Cortex-M4F)
- **RTOS**: FreeRTOS
- **Build System**: PlatformIO
- **Language**: C++ (mostly), some C for HAL/libraries
- **Libraries**: CMSIS-DSP, TinyUSB, LittleFS
- **Formatting**: clang-format (Google-based)
- **Linting**: clang-tidy via PlatformIO

## Git Repository
- Origin: `git@github.com:tonuonu/cats-embedded.git` (personal fork)
- Upstream: `catsystems/cats-embedded` (original)
