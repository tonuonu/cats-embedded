# Canard Roll and Vertical Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:**
- Add config-gated canard control to the CATS Vega flight computer firmware:
  - roll-rate stopping on the two stock servo outputs (`ROLL_2`);
  - vertical hold on four outputs (`FULL_4`);
  - SHADOW / STEP / CLOSED flight modes, logging, and ground test commands.
- Nothing may change when the layout is `OFF`.

**Architecture:**
- **Control module:** a HAL-free, plain-`float` module in `flight_computer/src/control/canard/` (attitude estimator, gain schedule, controllers, mixer, envelope state machine, facade).
  - It is unit-tested and closed-loop-simulated on the host through a new PlatformIO `native` environment.
- **Glue:** a thin firmware layer (`src/control/canard_io.*`) feeds it from `StateEstimation::Run()` at 100 Hz, writes servo pulses through a new compare-register servo driver, and records a new `CONTROL_INFO` log record.
- **Settings:** live in a new `canard_config_t` inside `cats_config_t`, with CLI settings and lookup tables.

**Tech Stack:** C++20, STM32F411 HAL, FreeRTOS (CMSIS-RTOS v2), PlatformIO (`ststm32` + `native`), Unity test framework.

**Spec:** `docs/superpowers/specs/2026-11-16-canard-control-design.md` (approved 2026-11-16). Read it in full before any task.

## Global Constraints

- **Off by default:**
  - `ctrl_layout` defaults to `OFF`.
  - With `OFF`, behaviour, servo outputs and log records are identical to before, apart from `CONFIG_VERSION`.
  - The control module is never called with `OFF`.
  - `CONTROL_INFO` is not recorded with `OFF`.
- **Config version:** `CONFIG_VERSION` goes from 212 to 213, exactly once, in Task 9.
- **No HAL in the control module:** nothing in `src/control/canard/` includes HAL, FreeRTOS, `arm_math.h`, `target.hpp` or `config/*`. It uses `<cstdint>`, `<cmath>` and plain `float` only, so it compiles natively.
- **Code style** (`docs/CODE_STYLE.md`):
  - PascalCase classes and functions, snake_case variables, `m_` members, `#pragma once`, 120 columns, clang-format Google-based.
  - Copyright header `/// Copyright (C) 2026 Control and Telemetry Systems GmbH` plus the SPDX line.
- **Toolchain and checks:** firmware builds with zero new warnings (`pio run -e release`), and `pio check -e release` is clean for new files.
- **Commits:** explicit `git add` paths only.
- **Units inside the module:** degrees for deflection and tilt, °/s for rates, m/s, m, s. Conversion from stored integer settings happens only in `canard_io.cpp`.
- **Spec values to copy exactly (defaults):**
  - **Speeds:** `ctrl_v_enable` 30 m/s, `ctrl_v_ref` 100 m/s, `ctrl_tilt_v_min` 70 m/s.
  - **Deflection and tilt limits:** `ctrl_max_defl` 8°, `ctrl_abort_tilt` 30°.
  - **Step test:** step axis `ROLL`, 3°, 300 ms per side, delay 300 ms, count 3, period 2000 ms; leading sign + − +.
  - **Servo signal:** `ctrl_center_us` 1500, `ctrl_servo_hz` 50.
  - **Exits:** roll-authority check at 180 °/s for 0.5 s; STEP exit at |roll rate| > 360 °/s; gyro near-saturation exit at ≥ 1900 °/s on any axis.
- **Pin and timer mapping:**
  - SERVO1 PB4 TIM3_CH1; SERVO2 PB5 TIM3_CH2.
  - OUT3 PA2 TIM2_CH3 (AF1); OUT4 PA3 TIM2_CH4 (AF1).
  - USART2 is de-initialised only when the layout is `FULL_4`.

## Review Focus

1. **The `sim` CLI command while the layout isn't `OFF`.** The gyro is frozen and the FSM goes to THRUSTING, so a builder might expect canards to move. The spec says neutral, latched with reason SIM. Test: `Envelope_SimRunning_LatchesNeutral` (Task 6).
2. **Reboot or brown-out mid-flight with the layout on.** The FSM restarts at CALIBRATING with the rocket moving, so the canards must hold trim and never jump to an un-levelled attitude. Test: `Envelope_StartsInNeutralPad_UntilThrusting` (Task 6).
3. **`ctrl_test` issued while the FSM leaves READY.** The override must stop at once and output trim. Test: `Envelope_TestOverride_CancelledByPhaseChange` (Task 6).
4. **Settings at extremes:**
   - `ctrl_max_defl = 0` gives all outputs at trim in every mode.
   - `ctrl_v_ref < ctrl_v_enable` must not divide by zero or blow up the gain.
   - Tests: `Mixer_ZeroMaxDefl_AllTrim` (Task 5) and `Schedule_VrefBelowVenable_Finite` (Task 3).
5. **Gyro long axis negative or not board y.** Calibration can pick x or z with either sign. Body mapping must stay right-handed so the roll sign is consistent. Test: `Frame_AllAxesAndSigns_RightHanded` (Task 2).

---

## File structure

| File | Responsibility |
|---|---|
| `flight_computer/src/control/canard/canard_types.hpp` | enums, `CanardParams`, `ControlInputs`, `ControlOutputs`, `TestCommand` |
| `flight_computer/src/control/canard/frame.hpp/.cpp` | board → body axis mapping |
| `flight_computer/src/control/canard/attitude.hpp/.cpp` | `ControlAttitude`: pad levelling, gyro-bias refinement, flight integration, tilt and rates |
| `flight_computer/src/control/canard/schedule.hpp/.cpp` | ISA density, dynamic pressure, gain scale |
| `flight_computer/src/control/canard/controllers.hpp/.cpp` | `RollController` (PI with anti-windup), `TiltController` (PD) |
| `flight_computer/src/control/canard/mixer.hpp/.cpp` | layout coefficients, roll-priority saturation, pulse conversion |
| `flight_computer/src/control/canard/envelope.hpp/.cpp` | `Envelope`: states, exits, flight modes, STEP doublets, test override |
| `flight_computer/src/control/canard/canard_control.hpp/.cpp` | `CanardControl` facade, one `Update()` per 10 ms |
| `flight_computer/test/test_canard_*/test_main.cpp` | Unity host tests, one folder per unit |
| `flight_computer/test/test_canard_sim/` | closed-loop roll and tilt simulation (host) |
| `flight_computer/test/test_canard_replay/` | `fl002` estimator replay with a CSV fixture |
| `flight_computer/tools/export_fl002_imu.py` | one-off fixture exporter |
| `flight_computer/src/drivers/servo_output.hpp/.cpp` | `driver::ServoOutput`: 1 µs-tick timer setup, `SetPulseUs` via compare register |
| `flight_computer/src/control/canard_io.hpp/.cpp` | glue: config → params, sensors → inputs, outputs → servos, `CONTROL_INFO` record |
| Modified | `platformio.ini`, `pre_config.py`, `config/cats_config.{hpp,cpp}`, `util/types.hpp`, `cli/settings.{hpp,cpp}`, `util/enum_str_maps.{hpp,cpp}`, `flash/recorder.{hpp,cpp}`, `tasks/task_recorder.cpp`, `flash/reader.cpp`, `cli/cli_commands.cpp`, `tasks/task_state_est.{hpp,cpp}`, `tasks/task_preprocessing.{hpp,cpp}`, `util/actions.cpp`, `main.cpp`, `target/VEGA/target.{hpp,cpp}`, `target/VEGA/stm32f4xx_hal_msp.cpp` |

---

### Task 1: Native test environment and module types

**Files:**
- Modify: `flight_computer/platformio.ini`, `flight_computer/pre_config.py`
- Create: `flight_computer/src/control/canard/canard_types.hpp`, `flight_computer/test/test_canard_types/test_main.cpp`

**Interfaces:**
- Produces (all in `namespace control::canard`):
  - `enum class Layout : uint8_t { kOff, kRoll2, kFull4 };`
  - `enum class FlightMode : uint8_t { kShadow, kStep, kClosed };`
  - `enum class StepAxis : uint8_t { kRoll, kT1, kT2 };`
  - `enum class Phase : uint8_t { kCalibrating, kReady, kThrusting, kCoasting, kRecovery };`
    The glue maps CALIBRATING/READY/THRUSTING/COASTING and DROGUE+ (kRecovery) to these.
  - `enum class EnvState : uint8_t { kDisabled, kNeutralPad, kActive, kNeutralLatched };`
  - `enum class LatchReason : uint8_t { kNone, kDrogue, kLowSpeed, kTilt, kGyroSat, kImuFault, kAuthority, kStepDone, kStepRollRate, kSim };`
  - `struct CanardParams`, with fields:
    - `Layout layout; FlightMode mode;`
    - `float v_enable, v_ref, tilt_v_min, max_defl_deg, abort_tilt_deg;`
    - `float roll_kp, roll_ki, tilt_kp, tilt_kd;`
    - `StepAxis step_axis; float step_defl_deg, step_s, step_delay_s, step_period_s; uint8_t step_count;`
    - `float authority_rate_dps = 180, authority_time_s = 0.5F, step_abort_rate_dps = 360, gyro_sat_dps = 1900;`
    - `float slew_dps;`
  - `struct ControlInputs`, with fields:
    - `float gyro_board_dps[3]; float acc_board_mps2[3];`
    - `uint8_t long_axis; float long_sign;`
    - `float velocity_mps, altitude_agl_m;`
    - `Phase phase; bool imu_faulty, sim_running; float dt_s;`
  - `struct ControlOutputs`, with fields:
    - `float defl_deg[4]; uint8_t n_outputs;`
    - `EnvState state; LatchReason reason;`
    - `float roll_rate_dps, tilt_deg[2];`
  - `struct TestCommand { enum class Kind : uint8_t { kNone, kSweep, kLive } kind; float forced_v_mps; float duration_s; };`

- [ ] **Step 1:** Add `[env:native]` to `platformio.ini`:
  - `platform = native`, `test_framework = unity`, `test_build_src = yes`;
  - `build_src_filter = -<*> +<control/canard/>`;
  - `build_flags = -std=c++20 -I src -Wall -Wextra -Werror`;
  - `build_unflags = -fno-rtti`; empty `extra_scripts`, `check_tool` and `board_build.ldscript`.
- [ ] **Step 2:** In `pre_config.py`, return before any `env.Append` when `env["PIOENV"] == "native"`.
- [ ] **Step 3:** Write test `test_canard_types/test_main.cpp`:
  - `TypesDefaults_NeutralOutputs`: a value-initialised `ControlOutputs{}` has all `defl_deg == 0`, `state == EnvState::kDisabled` and `reason == LatchReason::kNone`.
  - `sizeof(Layout) == 1`.
- [ ] **Step 4:** Run `pio test -e native -f test_canard_types`. Expected: FAIL to compile (header missing).
- [ ] **Step 5:** Create `canard_types.hpp` with exactly the Interfaces above. Use enum order as listed so that `kDisabled`/`kNone`/`kOff` are 0.
- [ ] **Step 6:** Run `pio test -e native -f test_canard_types`. Expected: PASS.
- [ ] **Step 7:** Run `pio run -e release`. Expected: builds, no new warnings. That proves `[env]` inheritance still works.
  - If PlatformIO rejects the inherited `board` for `native`: move `board`, `board_build.ldscript`, `upload_protocol`, `debug_tool`, `platform_packages`, `extra_scripts`, `build_flags`, `build_unflags` and `build_src_filter` from `[env]` into a new `[stm32]` section.
  - Then add `extends = stm32` to `[env:release]` and `[env:debug]`, and re-run both.
- [ ] **Step 8:** Commit: `build: add native unit-test environment for canard control`.

### Task 2: Body frame and control attitude estimator

**Files:**
- Create: `src/control/canard/frame.{hpp,cpp}`, `src/control/canard/attitude.{hpp,cpp}`, `test/test_canard_attitude/test_main.cpp`

**Interfaces:**
- Consumes: Task 1 types.
- Produces:
  - `void BoardToBody(const float board[3], uint8_t long_axis, float long_sign, float body[3]);`
    - `body[2]` = roll axis = `long_sign·board[a]`; `body[0] = board[(a+1)%3]`; `body[1] = long_sign·board[(a+2)%3]`.
  - `class ControlAttitude`:
    - `void Reset();`
    - `void UpdatePad(const float gyro_body_dps[3], const float acc_body_mps2[3], float dt_s);`
    - `void UpdateFlight(const float gyro_body_dps[3], float dt_s);`
    - `float RollRateDps() const;`
    - `void TransverseRatesDps(float out[2]) const;`
    - `void TiltDeg(float out[2]) const;`
    - `void GyroBiasDps(float out[3]) const;`

Algorithm decisions the tests pin:
- **Quaternion:** body→world, initial identity on `Reset()`.
- **Pad update:**
  - Subtract the bias, integrate, then apply a complementary correction rotating the estimated body "up" toward the measured `acc` direction, with time constant 2 s.
  - Skip the correction and the bias update when `| |acc| − 9.81 | > 0.5`.
  - Bias update: exponential average with time constant 10 s of the raw gyro, applied only when `|gyro − bias| < 2 °/s` on all axes.
- **Flight update:** bias-corrected gyro integration only, with no levelling and the bias frozen.
- **Tilt:**
  - `up_b = R^T·[0,0,1]`; `tilt_t1 = asin(clamp(−up_b.x))·180/π`; `tilt_t2 = asin(clamp(−up_b.y))·180/π`.
  - Positive `tilt_t1` means the nose leans toward body +t1.

- [ ] **Step 1:** Write the failing tests:
  - `Frame_AllAxesAndSigns_RightHanded`: for each `a ∈ {0,1,2}` and `s ∈ {+1,−1}`, map the unit vectors ex and ey and assert `body(ex)×body(ey) == body(ex×ey)` within 1e-6.
  - `Attitude_PadLevel_ConvergesToRailTilt`: feed `acc = 9.81·(sin5°, 0, cos5°)` in body for 20 s at dt 0.01 with zero gyro. Assert `TiltDeg()[0] ≈ −5` (±0.2): the nose leans away from +t1 when gravity has a +x component.
  - `Attitude_PadBias_Learned`: constant gyro (0.3, −0.2, 0.1) °/s for 60 s with level acc. Assert the bias equals that within 0.02.
  - `Attitude_Flight_IntegratesRoll`: after `Reset`, call `UpdateFlight` with `gyro = (0,0,90)` for 1 s. Assert `RollRateDps() == 90` and tilt stays within 0.01°.
  - `Attitude_Flight_IntegratesTilt`: `gyro = (0, 10, 0)` for 0.5 s from level. Assert `TiltDeg()[0] ≈ +5` (±0.05): rotating about +t2 tips the nose toward +t1.
  - `Attitude_PadSkipsLevellingUnderAccel`: |acc| = 15 for 5 s from level. Assert tilt is unchanged.
- [ ] **Step 2:** Run `pio test -e native -f test_canard_attitude`. Expected: FAIL (undefined).
- [ ] **Step 3:** Implement `frame.cpp` and `attitude.cpp` per the Interfaces and the algorithm decisions above.
- [ ] **Step 4:** Run `pio test -e native -f test_canard_attitude`. Expected: PASS.
- [ ] **Step 5:** Commit: `feat(canard): body frame mapping and control attitude estimator`.

### Task 3: Gain schedule

**Files:**
- Create: `src/control/canard/schedule.{hpp,cpp}`, `test/test_canard_schedule/test_main.cpp`

**Interfaces:**
- `float IsaDensity(float altitude_agl_m);`
  - `1.225F * powf(1 − 2.2558e-5F·h, 4.2559F)`, with h clamped to [0, 11000].
- `float DynamicPressure(float v_mps, float altitude_agl_m);`
  - `0.5·ρ·v²`.
- `float GainScale(float v_mps, float altitude_agl_m, float v_ref, float v_enable);`
  - `q(v_ref, 0) / max(q(v, h), q(v_enable, h), 1.0F)`.

- [ ] **Step 1:** Tests:
  - `Schedule_AtVref_IsOne`: `GainScale(100, 0, 100, 30) == 1` (±1e-4).
  - `Schedule_HalfSpeed_IsFour`: `GainScale(50, 0, 100, 30) ≈ 4`.
  - `Schedule_BelowEnable_ClampsAtEnable`: `GainScale(10, 0, 100, 30) == GainScale(30, 0, 100, 30)`.
  - `Schedule_VrefBelowVenable_Finite`: `GainScale(0, 0, 20, 30)` is finite and > 0.
  - `Isa_1000m`: `IsaDensity(1000) ≈ 1.112` (±0.002).
- [ ] **Step 2:** Run `pio test -e native -f test_canard_schedule`. Expected: FAIL.
- [ ] **Step 3:** Implement per the Interfaces.
- [ ] **Step 4:** Run `pio test -e native -f test_canard_schedule`. Expected: PASS.
- [ ] **Step 5:** Commit: `feat(canard): dynamic-pressure gain schedule`.

### Task 4: Roll and tilt controllers

**Files:**
- Create: `src/control/canard/controllers.{hpp,cpp}`, `test/test_canard_controllers/test_main.cpp`

**Interfaces:**
- `class RollController`:
  - `void Reset();`
  - `float Update(float roll_rate_dps, float gain_scale, float kp, float ki, float limit_deg, float dt_s);`
  - Returns `u_roll` in degrees: `u = −scale·(kp·p + ki·I)`.
  - Integrator `I += p·dt`, frozen when the previous output was saturated in the same direction as the new error (anti-windup).
  - Clamp `I` so that `|scale·ki·I| ≤ limit`.
- `class TiltController`:
  - `void Update(const float tilt_deg[2], const float rate_t_dps[2], float gain_scale, float kp, float kd, float v_mps, float tilt_v_min, float out_u[2]);`
  - `u_t1 = −scale·(kp·tilt_t1 + kd·rate_t2)`; `u_t2 = −scale·(kp·tilt_t2 − kd·rate_t1)`.
  - Here `rate_t` holds the body transverse rates (ω_t1, ω_t2).
  - `out_u = {0,0}` when `v_mps < tilt_v_min`.

- [ ] **Step 1:** Tests:
  - `Roll_PositiveRate_NegativeCommand`: rate +50 with kp 0.05 and scale 1 gives −2.5.
  - `Roll_Integrator_AntiWindup`: hold rate +100 for 5 s with kp 0, ki 0.1, limit 8. The output stays ≥ −8. After the rate flips to −10, the output moves positive within 0.2 s.
  - `Tilt_ZeroBelowVmin`: v = 69 with `tilt_v_min` 70 gives out {0,0}.
  - `Tilt_PushesNoseBack`: tilt {+5, 0}, rates 0, kp 1, v 100 gives `u_t1 < 0` and `u_t2 == 0`.
  - `Tilt_RateDamping_Signs`: tilt 0, rates {0, +10}, kd 0.1 gives `u_t1 == −1`; rates {+10, 0} gives `u_t2 == +1`.
- [ ] **Step 2:** Run `pio test -e native -f test_canard_controllers`. Expected: FAIL.
- [ ] **Step 3:** Implement per the Interfaces.
- [ ] **Step 4:** Run `pio test -e native -f test_canard_controllers`. Expected: PASS.
- [ ] **Step 5:** Commit: `feat(canard): roll PI and tilt PD controllers`.

### Task 5: Mixer and pulse conversion

**Files:**
- Create: `src/control/canard/mixer.{hpp,cpp}`, `test/test_canard_mixer/test_main.cpp`

**Interfaces:**
- `uint8_t NumOutputs(Layout layout);`
  - `kOff` → 0, `kRoll2` → 2, `kFull4` → 4.
- `void Mix(Layout layout, float u_roll, float u_t1, float u_t2, float max_defl_deg, float out_deg[4]);`
  - `ROLL_2`: `out[0] = out[1] = clamp(u_roll)`; `out[2] = out[3] = 0`.
  - `FULL_4`: output k is the canard at body azimuth k·90° from +t1 toward +t2:
    `δ0 = u_roll + u_t2; δ1 = u_roll − u_t1; δ2 = u_roll − u_t2; δ3 = u_roll + u_t1`.
  - Saturation (roll priority):
    - clamp `u_roll` to ±max first;
    - if any |δ| > max, scale `u_t1` and `u_t2` by the largest factor s ∈ [0,1] that brings all |δ| ≤ max (closed form per output, take the minimum);
    - final clamp guards rounding.
- `uint16_t PulseUs(float defl_deg, uint16_t center_us, int16_t trim_us, bool reverse, float us_per_deg);`
  - Round to nearest; clamp to [800, 2200].

- [ ] **Step 1:** Tests:
  - `Mixer_Roll2_BothOutputsRoll`: `u_roll` 3 gives {3, 3, 0, 0}.
  - `Mixer_Full4_PureTilt`: u_t1 = 2 gives {0, −2, 0, +2}.
  - `Mixer_RollPriority`: `u_roll` 6, u_t1 6, max 8. All |δ| ≤ 8, the roll component (mean of the four) equals 6, and the tilt is scaled to 2.
  - `Mixer_ZeroMaxDefl_AllTrim`: max 0 gives all 0.
  - `Pulse_ReverseAndTrim`: `PulseUs(2, 1520, −10, true, 10) == 1490`.
  - `Pulse_Clamped`: `PulseUs(200, 1500, 0, false, 10) == 2200`.
- [ ] **Step 2:** Run `pio test -e native -f test_canard_mixer`. Expected: FAIL.
- [ ] **Step 3:** Implement per the Interfaces.
- [ ] **Step 4:** Run `pio test -e native -f test_canard_mixer`. Expected: PASS.
- [ ] **Step 5:** Commit: `feat(canard): layout mixer with roll-priority saturation`.

### Task 6: Envelope state machine, flight modes, STEP doublets, test override

**Files:**
- Create: `src/control/canard/envelope.{hpp,cpp}`, `test/test_canard_envelope/test_main.cpp`

**Interfaces:**
- `struct EnvelopeInputs`, with fields:
  - `Phase phase; float v_mps, roll_rate_dps, tilt_mag_deg, max_abs_gyro_dps;`
  - `bool imu_faulty, sim_running;`
  - `float roll_cmd_deg; bool roll_cmd_saturated; float dt_s;`
- `struct EnvelopeDecision`, with fields:
  - `EnvState state; LatchReason reason;`
  - `enum class Source : uint8_t { kTrim, kController, kStep, kTest } source;`
  - `float step_cmd_deg; StepAxis step_axis;`
- `class Envelope`:
  - `void Configure(const CanardParams&);`
  - `void Reset();` puts the envelope in `kNeutralPad`, or `kDisabled` when the layout is `kOff`.
  - `EnvelopeDecision Update(const EnvelopeInputs&);`
  - `bool StartTest(const TestCommand&, Phase phase);` is accepted only in kCalibrating/kReady.
  - `void CancelTest();`

Rules from the spec, which the tests pin:
- **Entering ACTIVE:**
  - `kNeutralPad` → `kActive` when phase ≥ kThrusting and v ≥ `v_enable`.
  - Before that, and in pad phases, the source is `kTrim`.
- **Exits from ACTIVE:** each latches `kNeutralLatched` with its reason.
  - `kDrogue`: phase == kRecovery.
  - `kLowSpeed`: v < `v_enable` while phase == kCoasting.
  - `kTilt`: tilt > `abort_tilt`.
  - `kGyroSat`: `max_abs_gyro` ≥ `gyro_sat_dps`.
  - `kImuFault`: `imu_faulty`.
  - `kAuthority`: |roll rate| > `authority_rate_dps` continuously for ≥ `authority_time_s`, while `roll_cmd_saturated` and `sign(roll_cmd) == −sign(roll_rate)`. Applies in CLOSED only.
  - `kSim`: `sim_running` at any time when the layout isn't `OFF`. This also applies from `kNeutralPad`.
- **Modes in ACTIVE:**
  - SHADOW: source `kTrim`, with the controller still evaluated by the caller.
  - CLOSED: source `kController`.
  - STEP:
    - Source `kStep`, timed from the first update with phase == kCoasting.
    - Doublet i (i = 0..count−1) starts at `delay + i·period`. It runs `+s_i·defl` for `step_s`, then `−s_i·defl` for `step_s`, with `s_i = +1, −1, +1, …`.
    - Between doublets `step_cmd_deg = 0`.
    - After the last doublet ends: `kNeutralLatched`, reason `kStepDone`.
    - In STEP, |roll rate| > `step_abort_rate_dps` latches `kStepRollRate`.
- **Test override:**
  - While running, the source is `kTest`.
  - Any phase change since `StartTest`, or elapsed ≥ duration, cancels it and returns the source to `kTrim`.

- [ ] **Step 1:** Tests, one per rule. Names:
  - `Envelope_StartsInNeutralPad_UntilThrusting`
  - `Envelope_Activates_AtVenable`
  - `Envelope_Latch_Drogue`
  - `Envelope_Latch_LowSpeedInCoast`
  - `Envelope_NoLowSpeedLatchInBoost` (v < `v_enable` during kThrusting after activation does not latch)
  - `Envelope_Latch_Tilt`
  - `Envelope_Latch_GyroSat`
  - `Envelope_Latch_ImuFault`
  - `Envelope_Authority_LatchesAfter0p5s`
  - `Envelope_Authority_NotWhenCommandAgreesWithRate`
  - `Envelope_SimRunning_LatchesNeutral`
  - `Envelope_Shadow_SourceTrim`
  - `Envelope_Step_DoubletTimingAndSigns`: with the defaults, assert `step_cmd` at coast t = 0.25 s → 0; 0.35 s → +3; 0.65 s → −3; 0.95 s → 0; 2.35 s → −3; 2.65 s → +3; 4.35 s → +3; after 4.9 s → latched `kStepDone`.
  - `Envelope_Step_RollRateAbort`
  - `Envelope_TestOverride_OnlyOnPad`
  - `Envelope_TestOverride_CancelledByPhaseChange`
  - `Envelope_TestOverride_TimesOut`
  - `Envelope_LatchIsPermanent` (no return to ACTIVE after any latch)
- [ ] **Step 2:** Run `pio test -e native -f test_canard_envelope`. Expected: FAIL.
- [ ] **Step 3:** Implement `Envelope` per the rules.
- [ ] **Step 4:** Run `pio test -e native -f test_canard_envelope`. Expected: PASS.
- [ ] **Step 5:** Commit: `feat(canard): control envelope, flight modes and STEP doublets`.

### Task 7: CanardControl facade and closed-loop simulation; set default gains

**Files:**
- Create: `src/control/canard/canard_control.{hpp,cpp}`, `test/test_canard_sim/test_main.cpp`, `test/test_canard_sim/vehicle.hpp`

**Interfaces:**
- Consumes: Tasks 2–6.
- Produces `class CanardControl`:
  - `void Configure(const CanardParams&);`
  - `void Reset();`
  - `ControlOutputs Update(const ControlInputs&);`
  - `bool StartTest(const TestCommand&);` and `void CancelTest();`
- `Update` order:
  1. Map gyro and acc to body (Task 2).
  2. Attitude: `UpdatePad` in kCalibrating/kReady, else `UpdateFlight`.
  3. Compute `scale`.
  4. Compute `u_roll` and tilt `u` (tilt only for kFull4). Controllers run in every state so SHADOW logs real commands.
  5. Envelope decision.
  6. Pick the command by source:
     - kTrim → zeros;
     - kController → `Mix`;
     - kStep → `Mix(step on the chosen axis)`;
     - kTest sweep → output k steps ±`max_defl` in turn, 1 s per position;
     - kTest live → controller with `velocity = forced_v`.
  7. Slew-limit each output at `slew_dps` (default 300 °/s).
  8. Fill `ControlOutputs`. `defl_deg` holds what is actually sent, so trim sends 0.
- `vehicle.hpp` holds constants for the camera-nose Peregrine from `~/MyLevel1Peregrine` `openrocket/PeregrineL2-CamNose.ork` (branch `feature/camnose-canards-ork`, commit f05c608) and Rocket-Parts `tools/canard_estimate.py` (commit 59cd4d5):
  - liftoff mass 4.32 kg;
  - roll and pitch inertia, from OpenRocket's component analysis (export once; record the values and source in comments);
  - canard roll moment per degree per unit q, and fin roll damping, from `canard_estimate.py`. It must reproduce "stopping 360 °/s takes 0.12 s at 100 m/s" with the canards alone;
  - canard normal-force slope and arm, and fin restoring slope, for tilt.

- [ ] **Step 1:** Write sim tests. The host model is 1-DOF roll plus a 2-DOF pitch/yaw rigid body at constant V per case; integrate at 1 ms; the controller is called every 10 ms with a one-sample delay and a servo slew of 0.23 s/60°.
  - `Sim_Roll_DampsAt100`: 360 °/s initial at 100 m/s, CLOSED, ROLL_2 → |p| < 20 °/s within 1 s.
  - `Sim_Roll_StableAcrossEffectiveness`: roll effectiveness ∈ {1.0, 0.5, 0.3} × nominal at 60, 100 and 138 m/s → no growth, settles below 20 °/s within 2 s.
  - `Sim_Roll_ReversedTripsAuthority`: effectiveness −0.5 with a 200 °/s initial roll → `reason == kAuthority` within 1.5 s and outputs at 0 afterwards.
  - `Sim_Full4_TiltAt100`: 5° tilt at 100 m/s → below 2° within 2 s.
  - `Sim_Full4_TiltAt70_NoGrowth`: 5° at 70 m/s → never above 5.1°.
  - `Sim_StuckOutput_Stable`: one output forced to +8° for 5 s at 138 m/s, with the controller output ignored on that channel → bounded tilt (< `abort_tilt`) and bounded roll.
  - `Sim_MassSweep`: repeat `Sim_Roll_DampsAt100` with inertia −15 % → passes.
- [ ] **Step 2:** Run `pio test -e native -f test_canard_sim`. Expected: FAIL (no `CanardControl`).
- [ ] **Step 3:** Implement `CanardControl` per the Interfaces and the order above.
- [ ] **Step 4:** Tune `roll_kp`, `roll_ki`, `tilt_kp` and `tilt_kd` at `v_ref` 100 until every sim test passes. Record the chosen values as the defaults constants `kDefaultRollKp`, `kDefaultRollKi`, `kDefaultTiltKp` and `kDefaultTiltKd` in `canard_control.hpp`, with the sim test names in a comment.
- [ ] **Step 5:** Run `pio test -e native`. Expected: all canard suites PASS.
- [ ] **Step 6:** Commit: `feat(canard): control facade, closed-loop sim and default gains`.

### Task 8: fl002 estimator replay

**Files:**
- Create: `tools/export_fl002_imu.py`, `test/test_canard_replay/fl002_imu.h` (generated array), `test/test_canard_replay/test_main.cpp`

**Interfaces:**
- Consumes: `ControlAttitude`, `BoardToBody`.
- Exporter:
  - Reads `fl002.cfl` with `~/MyLevel1Peregrine/docs/flight/data/generate_charts.py:parse_cfl` (path as argv[1], cfl as argv[2]).
  - Emits IMU rows from liftoff − 5 s to apogee (state transitions in `fl002.summary.json`) as a C array: `{ts_ms, ax, ay, az, gx, gy, gz}`, raw int16, plus `kFl002LiftoffIndex`.
  - Conversion: acc `9.81/1024` per LSB, gyro `0.07` °/s per LSB. Long axis 1 (y), sign from st002 calibration (positive).

- [ ] **Step 1:** Run `python3 -I tools/export_fl002_imu.py ~/MyLevel1Peregrine/docs/flight/data/generate_charts.py ~/MyLevel1Peregrine/docs/flight/data/fl002.cfl > test/test_canard_replay/fl002_imu.h`. Expected: about 1800 rows.
- [ ] **Step 2:** Test `Replay_Fl002_TiltSensible`:
  - Feed pad rows to `UpdatePad`, then flight rows to `UpdateFlight` up to apogee.
  - Assert the max |tilt| during boost < 15°, and the tilt at apogee < 60°. The real flight was a normal near-vertical climb.
  - Assert the roll-rate trace equals the gyro y column ×0.07 within 1e-3.
- [ ] **Step 3:** Run `pio test -e native -f test_canard_replay`. Expected: PASS. If the tilt bounds fail, inspect the trace before loosening anything; a sign error in Task 2 is the likely cause.
- [ ] **Step 4:** Commit: `test(canard): fl002 attitude replay`.

### Task 9: Configuration and CLI settings

**Files:**
- Modify:
  - `src/util/types.hpp` (add `canard_config_t`)
  - `src/config/cats_config.hpp` (field and `CONFIG_VERSION` 213)
  - `src/config/cats_config.cpp` (defaults)
  - `src/cli/settings.hpp` (`TABLE_CTRL_LAYOUT`, `TABLE_CTRL_MODE`, `TABLE_CTRL_AXIS`)
  - `src/util/enum_str_maps.{hpp,cpp}` (string maps)
  - `src/cli/settings.cpp` (value rows)

**Interfaces:**
- `struct canard_config_t`, in `types.hpp`, with fields:
  - `uint8_t layout; uint8_t flight_mode;`
  - `uint16_t v_enable_mps; uint16_t v_ref_mps; uint16_t tilt_v_min_mps;`
  - `uint8_t max_defl_ddeg;` (0.1°)
  - `uint8_t abort_tilt_deg;`
  - `uint16_t roll_kp_milli, roll_ki_milli, tilt_kp_milli, tilt_kd_milli;` (×1000)
  - `uint8_t step_axis; uint8_t step_defl_ddeg; uint16_t step_ms; uint16_t step_delay_ms; uint8_t step_count; uint16_t step_period_ms;`
  - `uint16_t us_per_deg_deci;` (×10)
  - `uint16_t center_us; uint16_t servo_hz;`
  - `int16_t trim_us[4]; uint8_t reverse[4];`
- `cats_config_t` gains `canard_config_t canard{};`, appended before `buzzer_volume`.
- Defaults:
  - layout 0, mode 0 (SHADOW), v_enable 30, v_ref 100, tilt_v_min 70, max_defl 80, abort_tilt 30;
  - gains from Task 7 ×1000;
  - step axis 0, step 30, step_ms 300, delay 300, count 3, period 2000;
  - us_per_deg 100 (10 µs/°), center 1500, servo_hz 50, trims 0, reverse 0.
- CLI names (`ctrl_` prefix), types and ranges:

  | Name | Type | Range |
  |---|---|---|
  | `ctrl_layout` | UINT8 lookup | `OFF`/`ROLL_2`/`FULL_4` |
  | `ctrl_flight_mode` | UINT8 lookup | `SHADOW`/`STEP`/`CLOSED` |
  | `ctrl_v_enable` | UINT16 | 10–100 |
  | `ctrl_v_ref` | UINT16 | 20–300 |
  | `ctrl_tilt_v_min` | UINT16 | 20–300 |
  | `ctrl_max_defl` | UINT8 | 0–150 (0.1°) |
  | `ctrl_abort_tilt` | UINT8 | 5–60 |
  | `ctrl_roll_kp`, `ctrl_roll_ki`, `ctrl_tilt_kp`, `ctrl_tilt_kd` | UINT16 | 0–65535 |
  | `ctrl_step_axis` | UINT8 lookup | `ROLL`/`T1`/`T2` |
  | `ctrl_step_defl` | UINT8 | 0–100 |
  | `ctrl_step_ms` | UINT16 | 50–2000 |
  | `ctrl_step_delay_ms` | UINT16 | 0–10000 |
  | `ctrl_step_count` | UINT8 | 1–10 |
  | `ctrl_step_period_ms` | UINT16 | 200–10000 |
  | `ctrl_us_per_deg` | UINT16 | 10–1000 |
  | `ctrl_center_us` | UINT16 | 1000–2000 |
  | `ctrl_servo_hz` | UINT16 | 50–333 |
  | `ctrl_trim` | INT16 array[4] | ±300 |
  | `ctrl_reverse` | UINT8 array[4] | 0–1 |

- [ ] **Step 1:** Implement the struct, defaults, tables and rows. Follow the `TABLE_BATTERY` pattern for lookups and the `ev_*` rows for arrays.
- [ ] **Step 2:** Run `pio run -e release`. Expected: builds, no new warnings.
- [ ] **Step 3:** On the board, check the CLI:
  - After flashing, the first boot logs the config-version reset.
  - `get ctrl_` lists every setting with its default; `set ctrl_layout = ROLL_2` then `save` and `reboot` persists; `set ctrl_v_enable = 5` is rejected.
- [ ] **Step 4:** Commit: `feat(config): canard control settings (CONFIG_VERSION 213)`.

### Task 10: Servo output driver, timer setup and output ownership

**Files:**
- Create: `src/drivers/servo_output.{hpp,cpp}`
- Modify:
  - `src/target/VEGA/target.{hpp,cpp}` (TIM2 handle, `MX_TIM2_Init`, `target_switch_uart2_to_servo_outputs()`)
  - `src/target/VEGA/stm32f4xx_hal_msp.cpp` (TIM2 clock and PA2/PA3 AF1 in `HAL_TIM_MspPostInit`)
  - `src/main.cpp`, `src/util/actions.cpp`

**Interfaces:**
- `class driver::ServoOutput`:
  - `ServoOutput(TIM_HandleTypeDef& timer, uint32_t channel);`
  - `bool Configure(uint16_t frame_hz);` sets PSC so one tick is 1 µs from the actual timer clock (2× PCLK1 when the APB1 prescaler is not 1) and `ARR = 1'000'000/frame_hz − 1`. Returns false if `frame_hz` ∉ [50, 333].
  - `void Start();`
  - `void SetPulseUs(uint16_t us);` uses `__HAL_TIM_SET_COMPARE`, with no channel reconfiguration.
- `void target_switch_uart2_to_servo_outputs();` calls `HAL_UART_DeInit(&huart2)`, `MX_TIM2_Init()` and the PA2/PA3 AF1 setup.
- Globals: `driver::ServoOutput* global_canard_outputs[4]`, null when unused.
- `bool canard_outputs_claimed(uint8_t servo_index)` is declared in `control/canard_io.hpp` (Task 11) and stubbed here to read `global_cats_config.canard.layout`. Servo indices 0 and 1 are claimed for ROLL_2 and FULL_4.

Steps:
- [ ] **Step 1:** In `main.cpp` after `load_and_set_config()`:
  - **Layout OFF:** the existing servo code runs unchanged.
  - **Otherwise:**
    - Build `ServoOutput` on TIM3 CH1/CH2 (and on TIM2 CH3/CH4 after `target_switch_uart2_to_servo_outputs()` for FULL_4).
    - Run `Configure(servo_hz)`, then `SetPulseUs(center + trim[k])` before `Start()`.
    - Do not construct or start `driver::Servo` on the claimed channels, and leave `global_servo1`/`global_servo2` null.
- [ ] **Step 2:** In `actions.cpp`, `servo_channel_one`/`servo_channel_two` return `true` without acting when `canard_outputs_claimed(0/1)` is true.
- [ ] **Step 3:** Run `pio run -e release`. Expected: builds, no new warnings.
- [ ] **Step 4:** On the bench, with a scope or servo tester on the outputs:
  - ROLL_2 with HV6160 settings (center 1520, 333 Hz): SERVO1/SERVO2 output 1520 µs pulses at 333 Hz.
  - FULL_4: PA2/PA3 output the same. UART2 no longer echoes.
  - OFF: SERVO1/2 output `servo1_init_pos` as before.
- [ ] **Step 5:** Commit: `feat(drivers): compare-register servo outputs and TIM2 outputs on UART2 pins`.

### Task 11: Firmware glue, CONTROL_INFO record and reader

**Files:**
- Create: `src/control/canard_io.{hpp,cpp}`
- Modify:
  - `src/tasks/task_state_est.{hpp,cpp}` (own a `CanardIo`, call it after `kalman_step`)
  - `src/tasks/task_preprocessing.{hpp,cpp}` (`[[nodiscard]] bool IsImuFaulty() const noexcept;` returning `m_sensor_elimination.faulty_imu[0] != 0`)
  - `src/flash/recorder.{hpp,cpp}`, `src/tasks/task_recorder.cpp`, `src/flash/reader.cpp`, `src/cli/cli_commands.cpp` (rec filter name)

**Interfaces:**
- `class CanardIo`:
  - `void Init();` converts `global_cats_config.canard` to `CanardParams` (0.1° → °, ×1000 → gains, ms → s) and calls `CanardControl::Configure`/`Reset`.
  - `void Step(uint32_t tick, flight_fsm_e fsm, const SI_data_t& si, float velocity, float height_agl, bool imu_faulty);`
    - Builds `ControlInputs`, with `long_axis` and `long_sign` from `global_flight_stats.calibration_data` (`axis`, `sign(angle)`), `sim_running` = `simulation_started`, and `dt` 0.01.
    - Calls `Update`.
    - Writes `PulseUs(defl, center, trim[k], reverse[k], us_per_deg)` to each active `global_canard_outputs[k]`.
    - Records `CONTROL_INFO`.
  - `bool StartTest(const TestCommand&);`
- Record:
  - `CONTROL_INFO = 1U << 14U` in `rec_entry_type_e`.
  - `struct control_info_t` is 12 bytes, so the `rec_elem_u` union does not grow:
    - `int16_t roll_rate_ddps;` (0.1 °/s)
    - `int16_t tilt_cdeg[2];` (0.01°)
    - `int8_t defl_ddeg[4];` (0.1°, ±12.7°)
    - `uint8_t state; uint8_t reason;`
  - It is periodic: decimate like `FLIGHT_INFO` with its own `skip_counter.control_info`.
- `static_assert(sizeof(control_info_t) == 12)` and `static_assert(sizeof(rec_elem_u) == <size before change>)`.

- [ ] **Step 1:** Implement the glue, the record type, the recorder switch case, the `task_recorder` element size, the `reader.cpp` print line `"%lu|CONTROL_INFO|%d|%d|%d|%d|%d|%d|%d|%u|%u"`, and the `cli_commands.cpp` filter string `"CONTROL_INFO"`.
- [ ] **Step 2:** In `StateEstimation::Run()` after `kalman_step`:
  - when `global_cats_config.canard.layout != 0`, call `m_canard.Step(tick_count, m_fsm_enum, m_task_preprocessing.GetSIData(), m_filter.x_bar_data[1], m_filter.x_bar_data[0], m_task_preprocessing.IsImuFaulty())`;
  - call `m_canard.Init()` once before the loop;
  - raise the task stack from 512 if the size check (`uxTaskGetStackHighWaterMark` logged once at 10 s) shows less than 25 % free.
- [ ] **Step 3:** Run `pio run -e release` and `pio check -e release`. Expected: builds; no new warnings or check findings.
- [ ] **Step 4:** On the bench, ROLL_2 + SHADOW, using `sim` (also covers "unchanged when off"):
  - The `flight_parse` output contains CONTROL_INFO lines with state `NEUTRAL_LATCHED` and reason `SIM` (9) during the sim. The servos never move.
  - Repeat with `ctrl_layout = OFF` and save the `flight_parse` output. Diff it against the same sim on the pre-change firmware (build `main` at `60c5f61`). Only the config version and size differ, and there are no CONTROL_INFO lines.
- [ ] **Step 5:** Commit: `feat(canard): state-estimation glue and CONTROL_INFO logging`.

### Task 12: Ground test command

**Files:**
- Modify: `src/cli/cli_commands.cpp`

**Interfaces:**
- Consumes `CanardIo::StartTest` through a global `CanardIo* global_canard_io` set in `StateEstimation`.
- `ctrl_test sweep`:
  - `TestCommand{kSweep, 0, 4·n_outputs·1 s + 1}`.
  - Also enables IMU logging (`log_enable` path) for the duration, so servo vibration can be checked against accelerometer noise.
- `ctrl_test live <speed_mps> [seconds=20]`: `TestCommand{kLive, speed, seconds}`.
- Errors printed:
  - `"ctrl_test: layout is OFF"`;
  - `"ctrl_test: only on the pad (CALIBRATING/READY)"`;
  - `"usage: ctrl_test sweep | live <speed> [s]"`.

- [ ] **Step 1:** Implement the command and the `CLI_COMMAND_DEF("ctrl_test", "canard bench test", "sweep | live <speed> [s]", cli_cmd_ctrl_test)` row in alphabetical position.
- [ ] **Step 2:** Run `pio run -e release`. Expected: builds, no new warnings.
- [ ] **Step 3:** On the bench (spec "Bench"):
  - `ctrl_test sweep` moves each output in turn through ±max and back to trim. Confirm signs and set `ctrl_reverse`; measure deflection per µs and set `ctrl_us_per_deg`.
  - `ctrl_test live 100` with ROLL_2: twisting the rocket by hand makes the canards oppose the rotation.
  - FULL_4: tilting makes them push back toward vertical.
  - Unplugging USB or a phase change returns the canards to trim.
- [ ] **Step 4:** Commit: `feat(cli): ctrl_test sweep/live bench commands`.

### Task 13: Documentation and final verification

**Files:**
- Create: `docs/CANARD_CONTROL.md` (user-facing: settings table, bench procedure from Task 12, flight plan SHADOW → STEP → CLOSED, CONTROL_INFO field units).
- Modify: `README.md` feature list (one line).

- [ ] **Step 1:** Write the doc from the spec sections Settings, Ground and bench test support, Flight plan and Logging. Copy the settings defaults from Task 9.
- [ ] **Step 2:** Run the full verification:
  - `pio test -e native`: all PASS.
  - `pio run -e release` and `pio run -e debug`: no new warnings.
  - `pio check -e release`: no new findings.
- [ ] **Step 3:** Commit: `docs: canard control user guide`.

---

## Out of this plan (spec follow-ups in other repos)

- `generate_charts.py` in MyLevel1Peregrine must learn `CONTROL_INFO` (record id `1U << 14U`, 12 bytes, layout above) before it can read new logs.
- The CATS Configurator log converter has the same need.
