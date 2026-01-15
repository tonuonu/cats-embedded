# Task Completion Checklist

## Before Submitting Changes

### 1. Code Formatting
```bash
# Check formatting
clang-format --style=file -n <modified-files>

# Auto-format if needed
clang-format --style=file -i <modified-files>
```

### 2. Build Verification
```bash
# Build affected project(s)
cd <project> && platformio run

# For flight_computer, also build debug:
cd flight_computer && platformio run --environment debug
```

### 3. Linting
```bash
cd <project> && platformio check

# For flight_computer:
cd flight_computer && platformio check --environment debug
```

### 4. CI Expectations
GitHub Actions CI will:
1. Run clang-format check (clang-format 17)
2. Build all affected projects
3. Run clang-tidy linting
4. Upload build artifacts

CI runs on Ubuntu (lint) and Windows (release build).

## Excluded from Linting
- `lib/` directories (external libraries)
- `src/target/VEGA/stm*` files (HAL generated)
- `src/syscalls.c`, `src/sysmem.c`

## Known CI Exclusions (False Positives)
- `clang-diagnostic-c++17-extensions`
- `clang-analyzer-valist.Uninitialized`

## PR Workflow
1. Create feature branch from main
2. Make changes
3. Run format + build + lint locally
4. Push branch and create PR
5. Ensure CI passes before requesting review
