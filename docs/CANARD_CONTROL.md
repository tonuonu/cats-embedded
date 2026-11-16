# Canard roll and vertical control

Optional active control for the CATS Vega: small canards, driven by servos, damp the rocket's roll
(and, with four independent canards, hold it vertical). Control is OFF by default. With the layout
OFF nothing changes: the servo outputs, the log and the event actions behave as before.

Design background: `docs/superpowers/specs/2026-11-16-canard-control-design.md`.

## Known limitations

- The tilt (vertical hold, `FULL_4`) gains are **not validated by a flight-path simulation yet**.
  Stage 2 needs that before vertical hold is flown. Roll control is the stage 1 goal.
- The roll interaction between the canards and the aft fins is **not modelled**. The `STEP` flight
  measures it; the gains for `CLOSED` come from that measurement.
- `CONTROL_INFO` deflections are logged as 0.1 degree in an int8, so they saturate at +-12.7 degrees
  even though `ctrl_max_defl` allows 15.0.

## Hardware

- Servos: MKS HV6160 (4.8 to 8.4 V, 1520 us centre, frames up to 333 Hz).
- Servo power comes from the camera's 2S pack, through a fuse, with a capacitor at the servos. The
  Vega servo connectors carry **signal and ground only**; leave the 5 V pin unconnected.
- Layouts:

| Layout | Outputs | Wiring |
|---|---|---|
| `OFF` (default) | none | SERVO1/SERVO2 behave as before (events, `servo1_init_pos`) |
| `ROLL_2` | 2 | SERVO1 and SERVO2. Four canards on Y-cables: 0 and 180 deg on SERVO1, 90 and 270 deg on SERVO2. Roll only. |
| `FULL_4` | 4 | SERVO1, SERVO2, plus outputs 3 and 4 on the UART2 pins PA2 (TX) and PA3 (RX). UART2 is then not available as a serial port. |

A layout other than `OFF` claims its outputs: event actions `ACT_SERVO_ONE`/`ACT_SERVO_TWO` on a
claimed output are ignored.

## Settings

Set them in the CLI (`get ctrl_` lists all), then `save`.

**`ctrl_layout` takes effect only after `save` and a reboot.** The layout is latched at boot; a `set`
without a reboot changes nothing. The CLI prints a reminder.

| Setting | Range | Default | Unit / meaning |
|---|---|---|---|
| `ctrl_layout` | OFF, ROLL_2, FULL_4 | OFF | see above; save + reboot |
| `ctrl_flight_mode` | SHADOW, STEP, CLOSED | SHADOW | see Flight plan |
| `ctrl_v_enable` | 10 to 100 | 15 | m/s. Control starts above this speed. 15 m/s because the slow H motors burn out at low speed. |
| `ctrl_v_ref` | 20 to 300 | 100 | m/s. Speed at which the gains apply unscaled; gains are scheduled with 1/q |
| `ctrl_tilt_v_min` | 20 to 300 | 70 | m/s. Tilt control is only active above this speed |
| `ctrl_max_defl` | 0 to 150 | 80 | 0.1 deg, so 80 = 8.0 deg. Limit per canard |
| `ctrl_abort_tilt` | 5 to 60 | 30 | deg. Tilt above this latches neutral |
| `ctrl_roll_kp` | 0 to 65535 | 50 | x1000 (50 = 0.05). Roll proportional gain |
| `ctrl_roll_ki` | 0 to 65535 | 200 | x1000 (200 = 0.2). Roll integral gain |
| `ctrl_tilt_kp` | 0 to 65535 | 1000 | x1000. Tilt proportional gain |
| `ctrl_tilt_kd` | 0 to 65535 | 200 | x1000. Tilt derivative gain |
| `ctrl_step_axis` | ROLL, T1, T2 | ROLL | STEP axis. T1/T2 only with FULL_4 |
| `ctrl_step_defl` | 0 to 100 | 30 | 0.1 deg, so 30 = 3.0 deg. Doublet amplitude |
| `ctrl_step_ms` | 50 to 2000 | 300 | ms per side of a doublet |
| `ctrl_step_delay_ms` | 0 to 10000 | 300 | ms from entering COASTING to the first doublet |
| `ctrl_step_count` | 1 to 10 | 3 | number of doublets |
| `ctrl_step_period_ms` | 200 to 10000 | 2000 | ms between doublet starts |
| `ctrl_us_per_deg` | 10 to 1000 | 100 | x10 (100 = 10.0 us per degree). Measure on the bench |
| `ctrl_center_us` | 1000 to 2000 | 1500 | us. Servo centre pulse. **1520 for the MKS HV6160** |
| `ctrl_servo_hz` | 50 to 333 | 50 | Hz. Servo frame rate. **333 for the HV6160 once the bench check passes** |
| `ctrl_trim` | 4 values, us | 0,0,0,0 | us added per output (clamped to +-300 us in use). Not range-checked by the CLI |
| `ctrl_reverse` | 4 values, 0/1 | 0,0,0,0 | 1 reverses an output's direction |

Example for the HV6160, roll only:

```
set ctrl_layout = ROLL_2
set ctrl_center_us = 1520
set ctrl_servo_hz = 333
save
reboot
```

The per-flight config file grows from 382 to 428 bytes and `config_version` goes from 212 to 213.
The first boot with this firmware resets the stored config to defaults (it logs the version reset), so
re-enter your settings. External tools such as the CATS Configurator need the new layout to read it.

## Envelope: when the canards move

| State | Value | Canards |
|---|---|---|
| DISABLED | 0 | not driven by control (layout OFF) |
| NEUTRAL_PAD | 1 | trim positions: boot, calibrating, ready |
| ACTIVE | 2 | controller output (or STEP doublets) |
| NEUTRAL_LATCHED | 3 | trim for the rest of the flight |

ACTIVE needs the flight state THRUSTING or later and speed at or above `ctrl_v_enable`. Any exit
condition latches NEUTRAL_LATCHED for the rest of the flight and records its reason:

| Reason | Value | Cause |
|---|---|---|
| NONE | 0 | no latch |
| DROGUE | 1 | drogue phase reached |
| LOW_SPEED | 2 | speed below `ctrl_v_enable` while in COASTING only (not checked during thrust) |
| TILT | 3 | tilt above `ctrl_abort_tilt` |
| GYRO_SAT | 4 | a gyro axis reached 1900 deg/s |
| IMU_FAULT | 5 | IMU eliminated or faulty |
| AUTHORITY | 6 | CLOSED only. Roll authority check: roll rate above 90 deg/s for 0.5 s while the roll command is saturated against it (loop not working) |
| STEP_DONE | 7 | last STEP doublet finished |
| STEP_ROLL_RATE | 8 | STEP only. Roll rate above 360 deg/s |
| SIM | 9 | `sim` run (the servos never move in a sim) |

## Flight plan

Fly in three steps. Change only `ctrl_flight_mode` between flights.

1. **SHADOW** (default): canards stay at trim; the whole loop runs and is logged. Checks the
   estimator, loop timing and the barometer error from the ogive ports.
2. **STEP**: three roll doublets in coast (`ctrl_step_count`), each +-`ctrl_step_defl` for
   `ctrl_step_ms` per side, then trim. With the defaults they start 0.3 s, 2.3 s and 4.3 s into
   coast. Then NEUTRAL_LATCHED (reason STEP_DONE). This measures real roll authority, sign and size,
   including the canard-fin interaction. Fit the measured roll acceleration against the logged speed
   at each doublet.
3. **CLOSED**: the controller drives the canards, with gains updated from the STEP flight.

The exits SIM, IMU_FAULT, GYRO_SAT, DROGUE and TILT apply in every mode. DROGUE latches
immediately when the drogue phase is entered. AUTHORITY is checked in CLOSED only, so the authority
check does **not** protect a STEP flight; a STEP flight is protected by the 360 deg/s roll-rate exit
(STEP_ROLL_RATE) instead. LOW_SPEED applies only in COASTING.

## Logging: CONTROL_INFO

New record type `CONTROL_INFO` (`1U << 14U`, 12 bytes), recorded each control tick (10 ms) while the
layout latched at boot is not OFF, and added to the default record mask. `flight_parse` prints:

```
<ts>|CONTROL_INFO|roll|tilt1|tilt2|d0|d1|d2|d3|state|reason
```

| Field | Unit |
|---|---|
| roll | 0.1 deg/s |
| tilt1, tilt2 | 0.01 deg |
| d0 to d3 | 0.1 deg. Commanded (would-be, in SHADOW) deflection per output, +-12.7 deg |
| state | number, see Envelope |
| reason | number, see Reason table |

Log budget: with a layout on, the pre-liftoff history shrinks from about 768 ms to about 640 ms and
flash writes rise about 20 percent. With layout OFF nothing changes.

External log tools (CATS Configurator log converter, chart scripts) must learn the new record before
they can read logs from a canard build; they stop at the first unknown record type.

## Bench checklist

Do these in order, with the board on the bench and the servo horns free.

1. **Config.** After flashing, the first boot logs the config-version reset. `get ctrl_` lists every
   setting with its default. `set ctrl_layout = ROLL_2`, `save`, `reboot` persists it.
   `set ctrl_v_enable = 5` is rejected. `set ctrl_trim = 400,0,0,0` is accepted (arrays are not
   range-checked).
2. **Servo signal.** With a scope or servo tester and `ctrl_center_us 1520`, `ctrl_servo_hz 333`:
   - ROLL_2: SERVO1/SERVO2 output 1520 us pulses at 333 Hz.
   - FULL_4: PA2/PA3 output the same, and UART2 no longer echoes. (PA2 idles high for a few hundred
     ms at boot until the config loads.)
   - OFF: SERVO1/2 output `servo1_init_pos` as before.
   - Check the servos run from the Vega's 3.3 V signal and measure stall current at 8.4 V.
3. **Layout change needs a reboot.** `set ctrl_layout = ROLL_2` prints the reboot note. Without a
   reboot SERVO1/2 keep the OFF behaviour and the event actions still drive them.
4. **Sim, ROLL_2 + SHADOW.** Run `sim`. `flight_parse` shows CONTROL_INFO lines with state 3
   (NEUTRAL_LATCHED) and reason 9 (SIM). The servos never move.
5. **Sim, layout OFF.** Repeat with `ctrl_layout = OFF` and save the `flight_parse` output. Diff it
   against the same sim on the pre-change firmware (build `main` at `60c5f61`). Only the config
   version and size differ and there are no CONTROL_INFO lines.
6. **Stack check.** On a debug build with ROLL_2, read the line
   `StateEstimation stack: N of 512 words never used` 10 s after boot. Raise the stack if N < 128.
7. **Sweep.** `ctrl_test` works only in READY: wait about 10 s after boot and hold the board still.
   `ctrl_test sweep` moves each output in turn through +max, -max and back to trim. Confirm the
   signs and set `ctrl_reverse`; measure the deflection per us and set `ctrl_us_per_deg`.
8. **IMU noise during the sweep.** `flash_start_write`, `ctrl_test sweep`, `flash_stop_write`,
   `flight_parse`. Check the gyro and accelerometer noise with the servos moving.
9. **Live.** `ctrl_test live 100` (speed in m/s, optional duration in s, default 20). ROLL_2:
   twisting the rocket by hand makes the canards oppose the rotation. FULL_4: tilting makes them
   push back toward vertical.
10. **Safety returns.** Unplugging USB, or a phase change, returns the canards to trim.
