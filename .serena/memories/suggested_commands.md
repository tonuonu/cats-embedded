# Suggested Commands

## Build Commands

### Flight Computer
```bash
# Build release
cd flight_computer && platformio run

# Build debug
cd flight_computer && platformio run --environment debug

# Generate compile_commands.json (IDE support)
cd flight_computer && platformio run --target compiledb --environment debug

# Upload firmware via ST-Link
cd flight_computer && platformio run --target upload

# Clean build
cd flight_computer && platformio run --target clean
```

### Ground Station
```bash
cd ground_station && platformio run
cd ground_station && platformio run --target compiledb
cd ground_station && platformio run --target upload
```

### Telemetry Module
```bash
cd telemetry && platformio run
cd telemetry && platformio run --target compiledb
```

## Code Quality

### Format Check (clang-format)
```bash
# Check formatting (dry run)
clang-format --style=file -n <files>

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

## Git Commands (Darwin/macOS)
```bash
git status
git branch -a
git log --oneline -10
git diff
git checkout -b <branch-name>
git push -u origin <branch-name>
```

## System Utilities (Darwin)
```bash
ls -la
find . -name "*.cpp"
grep -r "pattern" --include="*.cpp"
```
