# Code Style and Conventions

## Formatting (clang-format - Google-based)
- **Indent**: 2 spaces (no tabs)
- **Column limit**: 120 characters
- **Braces**: Attached (K&R style)
- **Pointer alignment**: Left (`int* ptr`)
- **Include sorting**: Enabled
- **Short functions/lambdas**: Allowed on single line

## Naming Conventions
| Element | Style | Example |
|---------|-------|---------|
| Classes | PascalCase | `SensorRead`, `FlightFsm` |
| Functions/Methods | PascalCase | `GetState()`, `SetPosition()` |
| Variables | snake_case | `global_cats_config`, `rec_queue` |
| Constants/Macros | UPPER_SNAKE_CASE | `REC_QUEUE_SIZE` |
| Namespaces | lowercase | `driver`, `sensor`, `task` |
| Files | snake_case | `task_sensor_read.cpp` |

## File Extensions
- C++ headers: `.hpp`
- C++ source: `.cpp`
- C headers: `.h`
- C source: `.c`

## Header Guards
Use `#pragma once` (not traditional include guards)

## Code Patterns
- **RTOS Tasks**: Static class with `Start()` method returning reference
- **Drivers**: Namespace-based organization (`driver::`, `sensor::`)
- **Configuration**: Global config struct (`global_cats_config`)
- **Logging**: Use `log_info()`, `log_trace()`, etc. from `util/log.h`

## Copyright Header
```cpp
/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later
```

## NOLINT Usage
Use sparingly with explanation:
```cpp
// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
extern driver::Servo* global_servo1;
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)
```
