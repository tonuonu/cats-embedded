# Canard roll and vertical control for CATS Vega — design

Date: 2026-11-16 · Branch: `feature/canard-control` · Status: draft for owner approval

## Intent

Stop the rocket from rolling during boost and coast and, as a later stage, keep it climbing straight
up, using canards driven by the Vega's servo outputs.

- **Minimum hardware:** the first stage runs on a stock CATS Vega with its two servo outputs.
- **Upstream-friendly:** it should fit CATS's open-source philosophy, so other Vega users can use it
  and it could be offered upstream. It must be configuration-gated and off by default.
- **First vehicle:** the owner's Apogee Peregrine ("SIPSIK"). Specs: 100 mm body, 1.75 m long,
  2.35–3.1 kg, 38 mm motor mount. Peak speed on the J350 L2 flight was 173 m/s.
  - Roll on that L2 flight was 0.26 turns from liftoff to apogee, with a brief 114 °/s peak in boost.
  - So the Peregrine is a test bed for proving the control system, not a vehicle that badly needs it.
- **Later:** the same control code (estimator, control envelope, mixer, logging) should carry over
  to multi-motor thrust-vector control on a separate airframe.

## Goals

1. **Roll stopping** (hold roll *rate* near zero) on the two stock servo outputs. This is stage 1.
2. **Vertical hold** (hold the body axis at true vertical) on four outputs. This is stage 2.
   - The extra two outputs are the UART2 connector pins, PA2/PA3, re-assigned to PWM.
   - Stage 2 needs no extra board.
3. **One firmware for every hardware layout.** A single setting selects it:
   - `OFF`;
   - `ROLL_2`: two outputs, both commanding roll. This covers both the owner's 4-canard,
     Y-cable-paired build and a plain 2-canard build;
   - `FULL_4`: four outputs, commanding roll and both tilt axes.
4. **Off means unchanged.** With the layout `OFF`, behaviour and logs are unchanged apart from the
   config version bump.
5. **Logging and testing:** everything the controller sees and commands goes into the flight log.
   The controller can be tested on the ground: a host simulation, a replay of a real flight log,
   and bench checks.

## Non-goals

- Thrust vectoring, including the extra servo-controller board it needs (more than 4 outputs).
- Holding a chosen roll *angle* or heading, trajectory shaping, steering to targets.
- Canard mechanics and CAD. That work lives in the Rocket-Parts repo; this doc only states the
  constraints it must meet (see Hardware).
- CATS Configurator UI support. The new settings are reachable through the CLI only, for now.
- Control data in the radio telemetry downlink.
- IMU FIFO sampling (issue #2). That's a separate improvement and this design doesn't depend on it.

## Anti-goals (failure even if it "works")

- **Less safe than having no canards.** On any doubt the canards go to neutral and stay there for
  the rest of the flight.
- **Leaking into users who don't enable it.** Nothing changes for a Vega user who doesn't turn
  it on.
- **Requiring a custom board or a PCB mod** for stage 1.

## Approaches considered

| | Approach | Trade-off | Decision |
|---|---|---|---|
| A | Four canards and four servos from the start. Stage 1 pairs opposite canards on one output each with Y-cables (0°+180° on SERVO1, 90°+270° on SERVO2), giving roll only. Stage 2 removes the Y-cables and moves two servos to PA2/PA3. | Four servos carried before they are needed. No airframe rebuild for stage 2. | **Chosen (owner)** |
| B | Two canards, two servos, stock outputs. | Least hardware. Stage 2 needs a new canard section. | Supported by the same firmware (`ROLL_2`), not the owner's build |
| C | External servo-controller board on UART2. | Only needed for more than 4 outputs. | Rejected for this work |
| — | Two outputs with bank-to-turn: roll the rocket deliberately to point the one tilt axis at the error. | Contradicts roll stopping. | Rejected |

**Key constraint behind the table:** each independent output controls exactly one axis. Two outputs
can never give roll *and* both tilt axes. Pairing servos on one output adds force and symmetry,
not an axis.

**Why the pairing gives roll with no reverser:** take two opposite canards whose servos are mounted
identically (rotated 180° about the body axis). The same signal deflects them in the same rotational
sense, which is roll.

## Hardware (decided)

- **Servos:** Savox SH-1250MG, four of them.
  - Specs: digital, coreless, metal gear, 4.8–6.0 V, 4.6 kg·cm and 0.11 s/60° at 6 V, 29 g,
    35 × 15 × 29.2 mm.
  - Frame rate stays at 50 Hz. Raising it is a later, bench-verified change.
- **Servo power:** from an external 5 V supply, sharing ground with the Vega. The Vega supplies the
  signal lines only; its servo rail is rated 3 A total.
- **Outputs:**
  - **SERVO1:** PB4, TIM3_CH1 (existing).
  - **SERVO2:** PB5, TIM3_CH2 (existing).
  - **OUT3:** PA2 (UART2_TX on the connector), TIM2_CH3 on AF1. `FULL_4` only.
  - **OUT4:** PA3 (UART2_RX on the connector), TIM2_CH4 on AF1. `FULL_4` only.
  - UART2 is unused by the current firmware: `drivers/uart.cpp` has no callers.
  - TIM2 is free (TIM1 is the HAL timebase, TIM3 the servos, TIM4 the buzzer).
- **Constraints the canard mechanics must meet (Rocket-Parts, out of scope here):**
  - Canards are forward of the CG.
  - Each pivot is near the canard's centre of pressure (~25 % chord), so the hinge moment stays well
    inside the servo torque at 173 m/s.
  - Mechanical stops sit just beyond the configured maximum deflection.
  - With every canard at full deflection, the fins must still win: the rocket trims to a small
    angle of attack and stays stable.
  - For `FULL_4`, canard positions are aligned with the Vega board's two transverse axes, within a
    few degrees.
  - For `ROLL_2`, the angular position of the canards doesn't matter.

## Firmware design

### Where it runs

- **Location:** a new HAL-free module under `src/control/`, which also makes it testable on the host:
  - attitude estimator for control;
  - gain schedule;
  - controller;
  - mixer;
  - control envelope state machine.
- **Calling it:** a thin wrapper, called from `StateEstimation::Run()` at 100 Hz right after
  `kalman_step()`. That point already has the Kalman velocity and altitude, the preprocessed
  gyro/accel and the FSM state.
- **Output path:** servo outputs are written through a new path that sets the timer compare
  register directly (`__HAL_TIM_SET_COMPARE`).
  - The existing `Pwm::UpdateDutyCycleTicks` re-configures and restarts the channel on every write.
    That is fine for one-off event actions but not for 100 Hz updates.
  - The existing `Servo::SetPosition(0..1000)` mapping stays unchanged for event actions.
- **Owner's call:** run inside the StateEstimation task, not a new task. It has all the inputs,
  avoids cross-task copies and keeps the delay to one loop.

### Body frame

- **Roll axis:** the long axis is the one calibration picks (`m_calibration.axis`). Its sign comes
  from the sign of `m_calibration.angle`. On SIPSIK it is board y.
- **Transverse axes:** the other two board axes.
- **Convention:** a fixed, documented mapping from board axes to body axes. Per-output
  reverse flags absorb any sign mismatch, and the bench check proves the signs.

### Attitude estimate for control

- **Separate from the existing filter:** the control module keeps its own quaternion. The existing
  `orientation_filter` and its `ORIENTATION_INFO` records stay exactly as they are, which goal 4
  requires.
- **On the pad (READY):** the estimate is levelled from the accelerometer, a slow complementary
  correction towards measured gravity. That gives true vertical including rail tilt.
  - Gyro bias is re-estimated as a running average while the rocket is still.
  - Levelling pauses whenever |acc| differs from g by more than a threshold.
- **From liftoff (FSM → THRUSTING):** levelling stops and the estimate is gyro-only, using the
  refined bias.
- **Outputs:** body roll rate, transverse rates, and a 2-component tilt vector (body axis versus
  vertical, expressed on the two transverse axes).
- **Reference:** true vertical, not the rail direction, because the owner asked for "straight up".
- **Stage 1:** tilt is computed and logged but not actuated. That collects stage-2 data on
  stage-1 flights.

### Gain schedule

- **Formula:** `K = K_ref · q_ref / max(q, q_enable)`.
  - Dynamic pressure `q = ½ ρ V²`.
  - V is the Kalman velocity.
  - ρ is ISA density at AGL altitude.
- **Reference points:** `q_ref` is the dynamic pressure at `ctrl_v_ref`, and `q_enable` the one at
  `ctrl_v_enable`.
- **Effect:** canard authority scales with q, so this keeps loop gain roughly constant across the
  flight.

### Control laws

- **Roll:** PI on roll rate towards zero, with anti-windup (the integrator is clamped and frozen
  while the output saturates). The integral term absorbs canard misalignment.
- **Tilt** (`FULL_4` only): PD per transverse axis, `u = −Kp·tilt − Kd·rate`, driving the tilt
  vector to zero.
- **Limits:** all commands are in canard degrees and clamped to `ctrl_max_defl`. A slew limit
  protects the servos.

### Mixer

- **Inputs:** `u_roll`, `u_t1`, `u_t2` in degrees.
- **Per output k:** `δ_k = a_k·u_roll + b_k·u_t1 + c_k·u_t2`, with coefficients from the layout
  preset:
  - `ROLL_2`: both outputs `(1,0,0)`.
  - `FULL_4`: the four outputs sit at the ±t1 and ±t2 positions. Each has roll `1`, plus its sideways
    force on the tilt axis it can push (`±1`). Exact signs follow the documented frame.
- **Conversion to pulse:** `pulse_us = 1500 + trim_k + rev_k · δ_k · ctrl_us_per_deg`.
  - `ctrl_us_per_deg` comes from the linkage geometry and is measured on the bench.
- **Generic by design:** the table supports N outputs, so a future thrust-vector layout is a new
  preset, not new code.

### Control envelope state machine

| State | Canards | Enter when |
|---|---|---|
| `DISABLED` | not driven by control (layout `OFF`) | config |
| `NEUTRAL_PAD` | trim positions | boot, CALIBRATING, READY |
| `ACTIVE` | controller output | FSM ≥ THRUSTING **and** V ≥ `ctrl_v_enable` |
| `NEUTRAL_LATCHED` | trim positions for the rest of the flight | any exit condition below |

Exit conditions from `ACTIVE`, each latching and logged with its reason:
- FSM reaches DROGUE;
- V drops below `ctrl_v_enable` during coast;
- tilt exceeds `ctrl_abort_tilt`;
- any gyro axis reaches ≥ 1900 °/s (near saturation);
- preprocessing reports the IMU as eliminated or faulty;
- **roll authority check (owner's call, after review):** |roll rate| stays above a threshold
  (default 180 °/s) for 0.5 s while the roll command is saturated in the opposing direction. That
  means the loop is not working, possibly because canard wake on the aft fins reverses the roll
  effect.

Other envelope rules:
- After landing, the servos keep holding neutral; PWM isn't stopped.
- **Shadow mode:** `ctrl_max_defl = 0` runs the whole loop and logs it, but never moves the canards.
  Intended for the first flight.

### Who owns the outputs

- **Claiming outputs:** when the layout isn't `OFF`, the outputs it uses belong to the controller.
  - Event actions `ACT_SERVO_ONE`/`ACT_SERVO_TWO` on a claimed output are ignored at runtime.
  - `initial_servo_position` is ignored for claimed outputs.
  - The CLI warns when such actions are configured.
- **`FULL_4` setup:** right after the config loads, USART2 is de-initialised and PA2/PA3 are switched
  to TIM2_CH3/CH4 at 50 Hz.

### Settings (CLI)

These are new fields in `cats_config_t`. `CONFIG_VERSION` goes from 212 to 213, which resets every
board's saved config to defaults once.

| Name | Meaning | Default |
|---|---|---|
| `ctrl_layout` | `OFF` / `ROLL_2` / `FULL_4` | `OFF` |
| `ctrl_v_enable` | m/s, activation and exit speed | 30 |
| `ctrl_v_ref` | m/s, gain reference speed | 100 |
| `ctrl_max_defl` | deg, command clamp (0 = shadow mode) | 8 |
| `ctrl_abort_tilt` | deg, latch-to-neutral tilt | 30 |
| `ctrl_roll_kp`, `ctrl_roll_ki` | roll gains at `v_ref` | from simulation |
| `ctrl_tilt_kp`, `ctrl_tilt_kd` | tilt gains at `v_ref` (`FULL_4`) | from simulation |
| `ctrl_us_per_deg` | linkage calibration | from bench |
| `ctrl_trim1..4` | µs offset from 1500 per output | 0 |
| `ctrl_rev1..4` | per-output reverse | off |

The builder chooses the fixed-point/scaling of each stored field.

### Logging

- **Record type:** new `CONTROL_INFO` (`1U << 14U`), recorded at the control rate while the layout
  isn't `OFF`. It holds:
  - roll rate;
  - the two tilt components;
  - the four commanded deflections;
  - envelope state and the latch reason.
- **Rec mask:** added to the default mask.
- **Readers:** the firmware's own `flight_parse` CLI is updated to print it.
- **Outside this repo:** `generate_charts.py` in MyLevel1Peregrine stops at the first unknown record
  type, so it must learn the new record before it can read new logs. That's a follow-up there.
  The CATS Configurator log converter is in the same position.

### Ground and bench test support

- **CLI `ctrl_test sweep`:** moves each claimed output through ±max deflection and back to trim, one
  at a time. Used for wiring, signs and `us_per_deg`.
- **CLI `ctrl_test live <speed>`:** runs the full controller on the ground for a fixed time with
  speed forced to `<speed>`, so twisting or tilting the rocket by hand shows the corrective
  deflections.
  - Times out back to neutral.
- **How the test commands are gated (owner's call, after review):**
  - The USB CLI *does* run alongside the flight tasks. `task_health_monitor` starts it when USB is
    detected, while sensors, estimation and FSM are already running.
  - So `ctrl_test` runs inside the live pipeline as a test override in the control module.
  - It is accepted only while the FSM is CALIBRATING or READY.
  - Any FSM transition, and the timeout, cancel it and return to neutral.
  - The override replaces the envelope's output; it is not a second writer to the timers.
- **CLI `sim`:** it drives the real FSM through THRUSTING/COASTING with a frozen gyro. While a
  simulation runs, the envelope is forced to `NEUTRAL_LATCHED` (reason "sim"), so the canards never
  move during `sim`. The `CONTROL_INFO` records are still logged.

## Verification

- **Host unit tests** of the control module: estimator, schedule, controller, mixer, envelope.
  - Uses a new PlatformIO `native` environment with Unity. The repo has no host tests today; adding
    them is the owner's call recorded here.
- **Host closed-loop simulation:**
  - Rotational dynamics of the Peregrine: inertia, fin restoring moment, canard moment, roll
    disturbance from canard misalignment, wind gusts.
  - Use the Peregrine's OpenRocket data (`~/MyLevel1Peregrine/openrocket/PeregrineL2.ork`).
    It has 3 aft fins and no canards yet.
  - Canard geometry and the servo bay mass come from the nosecone CAD work, which runs in parallel.
    Until then the simulation is parametric and the gains are placeholders. The pass criteria are
    checked once real geometry exists.
  - **Canard–fin interaction:** canard wake on the aft fins can cancel or reverse the roll effect,
    and that can't be quantified from existing data. So the roll effectiveness is a simulation
    parameter, swept from full to zero to reversed.
    - The controller must be stable across the expected range.
    - A reversed effect must trip the authority-check latch.
  - Gains come from here.
  - Pass criteria:
    - roll: 360 °/s at 100 m/s damped below 20 °/s within 1 s;
    - `FULL_4`: a 5° tilt at ≥ 60 m/s reduced below 1° within 2 s;
    - every abort condition latches to neutral;
    - a stuck full-deflection output keeps the vehicle stable.
- **Log replay:** feed `fl002.cfl` (the L2 J350 flight) IMU data through the control estimator and
  check the tilt stays physically sensible.
- **Unchanged when off:** with layout `OFF`, a `sim` run's log is identical to the pre-change
  firmware apart from config version and size.
- **Build:** zero new warnings, `platformio check` clean.
- **Bench:**
  - `sweep` with the signs confirmed;
  - twist test (`live`): the canards oppose the hand rotation;
  - `FULL_4` tilt test: the canards push back towards vertical.
- **Flight plan** (owner's call, suggested):
  1. Flight 1: `ROLL_2` in shadow mode.
  2. Flight 2: active with a small `ctrl_max_defl`.
  3. Then normal limits.

## Left to the builder

- Module, class and file names.
- Fixed-point formats of settings and log fields.
- The exact frame convention and mixer signs, as long as they are documented and the bench check
  proves them.
- Levelling and bias filter constants.
- Slew limit value.
- How the `native` test environment is wired.
- The authority-check threshold and duration, refined in the simulation.
- Simulation implementation language (C++ native test or Python).
