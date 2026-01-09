# CATS Embedded - Project Overview

## Purpose
CATS (Control and Telemetry Systems) is an open-source flight computer system for sounding rockets. The project provides:
- Flight computer firmware (CATS Vega)
- Ground station firmware
- Telemetry module firmware

## Key Features
- Kalman filter altitude and velocity estimation
- Configurable events (up to 8 actions per flight state transition)
- High-speed logging (up to 100 Hz)
- Telemetry with 10+ km range
- GNSS logging and transmission
- Accelerometric liftoff detection
- USB mass storage for flight data access

## Repository Structure
```
cats-embedded/
├── flight_computer/     # STM32F411 flight computer firmware
│   ├── src/            # Application source code
│   │   ├── cli/        # Command line interface
│   │   ├── comm/       # Communication streams
│   │   ├── config/     # Configuration management
│   │   ├── control/    # Kalman filter, flight phases, calibration
│   │   ├── drivers/    # Hardware drivers (SPI, UART, PWM, etc.)
│   │   ├── flash/      # Flash storage, recorder, LittleFS
│   │   ├── init/       # System initialization
│   │   ├── sensors/    # Sensor drivers (LSM6DSO32, MS5607)
│   │   ├── tasks/      # FreeRTOS tasks
│   │   ├── target/     # Target-specific code (VEGA)
│   │   ├── usb/        # USB device stack
│   │   └── util/       # Utilities
│   └── lib/            # External libraries (STM HAL, FreeRTOS, CMSIS, TinyUSB, LittleFS)
├── ground_station/      # ESP32-S2 ground station firmware (Arduino framework)
├── telemetry/          # STM32G0 telemetry module firmware
└── .github/            # CI/CD workflows
```

## Tech Stack
- **Flight Computer**: STM32F411CE, C++, FreeRTOS, STM32 HAL, CMSIS-DSP, TinyUSB, LittleFS
- **Ground Station**: ESP32-S2, Arduino framework, TinyUSB, various Adafruit libraries
- **Telemetry**: STM32G071RB, STM32Cube framework
- **Build System**: PlatformIO
- **CI/CD**: GitHub Actions

## Hardware Documentation
- **Product Pages**: https://www.catsystems.io/vega, https://www.catsystems.io/ground-station-1
- **Hardware Repo**: https://github.com/catsystems/cats-hardware (Altium source files)
- **Schematics (PDF)**:
  - Vega: https://github.com/catsystems/cats-hardware/raw/main/CATS-Vega/CATS-Vega.pdf
  - Ground Station: https://github.com/catsystems/cats-hardware/raw/main/CATS-GS/CATS-GS.pdf
- **3D CAD (STEP)**:
  - Vega: https://github.com/catsystems/cats-hardware/tree/main/CATS-Vega/3D
  - Ground Station: https://github.com/catsystems/cats-hardware/tree/main/CATS-GS/3D
- **See**: [HARDWARE.md](HARDWARE.md) for detailed specs

## License
GPL-3.0-or-later
