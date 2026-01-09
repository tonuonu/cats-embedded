# Code Style and Conventions

## Formatting (clang-format)
Based on Google style with modifications:
- **Indent**: 2 spaces (no tabs)
- **Column limit**: 120 characters
- **Braces**: Attached (K&R style)
- **Pointer alignment**: Left (`int* ptr`)
- **Include sorting**: Enabled
- **Short functions/lambdas**: Allowed on single line
- **Short if statements**: Allowed without braces (without else)

## Naming Conventions
- **Classes**: PascalCase (`SensorRead`, `FlightFsm`)
- **Functions/Methods**: PascalCase (`GetState()`, `SetPosition()`)
- **Variables**: snake_case (`global_cats_config`, `rec_queue`)
- **Constants/Macros**: UPPER_SNAKE_CASE (`REC_QUEUE_SIZE`, `BOOTLOADER_MAGIC_PATTERN`)
- **Namespaces**: lowercase (`driver`, `sensor`, `task`)
- **Files**: snake_case (`task_sensor_read.cpp`, `kalman_filter.hpp`)
- **Header guards**: `#pragma once`

## File Organization
- Headers use `.hpp` extension for C++
- Source files use `.cpp` extension
- C files use `.c` and `.h` extensions
- Each task in `tasks/` directory has corresponding `.hpp` and `.cpp` files

## Code Patterns
- **RTOS Tasks**: Static class with `Start()` method returning reference
- **Drivers**: Namespace-based organization (`driver::`, `sensor::`)
- **Configuration**: Global config struct (`global_cats_config`)
- **Logging**: Use `log_info()`, `log_trace()`, etc. from `util/log.h`

## Comments
- Copyright header: `/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH`
- SPDX license identifier: `/// SPDX-License-Identifier: GPL-3.0-or-later`
- Doxygen-style for function documentation

## Linting (clang-tidy)
Enabled checks include:
- bugprone-*, clang-analyzer-*, cppcoreguidelines-*, google-*, hicpp-*, modernize-*, performance-*, readability-*

Disabled checks (notable):
- Magic numbers allowed (no `-readability-magic-numbers`)
- C-style arrays allowed in embedded context
- Some HAL-related checks disabled due to library constraints

## NOLINT Comments
Use sparingly with explanation:
```cpp
// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
extern driver::Servo* global_servo1;
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)
```
