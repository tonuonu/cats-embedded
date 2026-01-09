# Development Guide

## Build Commands

### Flight Computer
```bash
# Build release
cd flight_computer && platformio run

# Build debug
cd flight_computer && platformio run --environment debug

# Generate compile_commands.json (for IDE support)
cd flight_computer && platformio run --target compiledb --environment debug

# Upload firmware via ST-Link
cd flight_computer && platformio run --target upload

# Clean build
cd flight_computer && platformio run --target clean
```

### Ground Station
```bash
# Build
cd ground_station && platformio run

# Generate compile_commands.json
cd ground_station && platformio run --target compiledb

# Upload firmware
cd ground_station && platformio run --target upload
```

### Telemetry Module
```bash
# Build
cd telemetry && platformio run

# Generate compile_commands.json
cd telemetry && platformio run --target compiledb
```

## Code Quality Commands

### Format Check (clang-format)
```bash
# Check formatting for flight_computer
clang-format --style=file -n flight_computer/src/**/*.cpp flight_computer/src/**/*.hpp

# Auto-format files
clang-format --style=file -i <file>
```

### Linting (clang-tidy via PlatformIO)
```bash
# Lint flight_computer
cd flight_computer && platformio check --environment debug

# Lint ground_station
cd ground_station && platformio check

# Lint telemetry
cd telemetry && platformio check
```
