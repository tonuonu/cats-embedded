# Contributing to CATS Embedded

## Task Completion Checklist

### Before Submitting Changes

#### 1. Code Formatting
```bash
# Check that code is properly formatted
clang-format --style=file -n <modified-files>

# Auto-format if needed
clang-format --style=file -i <modified-files>
```

#### 2. Build Verification
```bash
# Build the affected project(s)
cd <project> && platformio run

# For flight_computer, also build debug:
cd flight_computer && platformio run --environment debug
```

#### 3. Linting
```bash
# Run clang-tidy checks
cd <project> && platformio check

# For flight_computer:
cd flight_computer && platformio check --environment debug
```

#### 4. CI Expectations
The GitHub Actions CI will:
1. Run clang-format check (clang-format 17)
2. Build all affected projects
3. Run clang-tidy linting
4. Upload build artifacts

CI runs on both Ubuntu (lint) and Windows (release build).

#### 5. PR Guidelines
- Use the PR template at `.github/pull_request_template.md`
- Ensure CI passes before requesting review
- Changes trigger builds only for affected projects (flight_computer, ground_station, telemetry)

## Known CI Exclusions
The following warnings are excluded from CI lint checks (false positives):
- `clang-diagnostic-c++17-extensions`
- `clang-analyzer-valist.Uninitialized`

## Excluded from Linting
- `lib/` directories (external libraries)
- `src/target/VEGA/stm*` files (HAL generated)
- `src/target/VEGA/system_stm*` files
- `src/syscalls.c`, `src/sysmem.c`
