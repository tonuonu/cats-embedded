/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include <unity.h>

#include "control/canard/envelope.hpp"

using namespace canard;
using Source = EnvelopeDecision::Source;

void setUp() {}
void tearDown() {}

namespace {

CanardParams Params(FlightMode mode = FlightMode::kClosed) {
  CanardParams p;
  p.layout = Layout::kRoll2;
  p.mode = mode;
  p.v_enable = 40.0F;
  p.abort_tilt_deg = 20.0F;
  p.step_axis = StepAxis::kRoll;
  p.step_defl_deg = 3.0F;
  p.step_s = 0.3F;
  p.step_delay_s = 0.3F;
  p.step_period_s = 2.0F;
  p.step_count = 3;
  return p;
}

EnvelopeInputs Coast(float v = 100.0F) {
  EnvelopeInputs in;
  in.phase = Phase::kCoasting;
  in.v_mps = v;
  in.dt_s = 0.05F;
  return in;
}

Envelope Active(const CanardParams& p, Phase phase = Phase::kCoasting) {
  Envelope e;
  e.Configure(p);
  e.Reset();
  EnvelopeInputs in = Coast();
  in.phase = phase;
  EnvelopeDecision d = e.Update(in);
  TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kActive), static_cast<int>(d.state));
  return e;
}

void AssertLatched(const EnvelopeDecision& d, LatchReason r) {
  TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kNeutralLatched), static_cast<int>(d.state));
  TEST_ASSERT_EQUAL(static_cast<int>(r), static_cast<int>(d.reason));
  TEST_ASSERT_EQUAL(static_cast<int>(Source::kTrim), static_cast<int>(d.source));
}

}  // namespace

void Envelope_StartsInNeutralPad_UntilThrusting() {
  Envelope e;
  e.Configure(Params());
  e.Reset();
  EnvelopeInputs in = Coast();
  in.phase = Phase::kReady;
  EnvelopeDecision d = e.Update(in);
  TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kNeutralPad), static_cast<int>(d.state));
  TEST_ASSERT_EQUAL(static_cast<int>(Source::kTrim), static_cast<int>(d.source));

  Envelope off;
  CanardParams po = Params();
  po.layout = Layout::kOff;
  off.Configure(po);
  off.Reset();
  in.phase = Phase::kCoasting;
  d = off.Update(in);
  TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kDisabled), static_cast<int>(d.state));
  TEST_ASSERT_EQUAL(static_cast<int>(LatchReason::kNone), static_cast<int>(d.reason));
  TEST_ASSERT_EQUAL(static_cast<int>(Source::kTrim), static_cast<int>(d.source));
  TEST_ASSERT_FALSE(off.StartTest({TestCommand::Kind::kSweep, 0.0F, 5.0F}, Phase::kReady));
}

void Envelope_Activates_AtVenable() {
  Envelope e;
  e.Configure(Params());
  e.Reset();
  EnvelopeInputs in = Coast(39.9F);
  in.phase = Phase::kThrusting;
  EnvelopeDecision d = e.Update(in);
  TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kNeutralPad), static_cast<int>(d.state));
  in.v_mps = 40.0F;
  d = e.Update(in);
  TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kActive), static_cast<int>(d.state));
  TEST_ASSERT_EQUAL(static_cast<int>(Source::kController), static_cast<int>(d.source));
}

void Envelope_Latch_Drogue() {
  Envelope e = Active(Params());
  EnvelopeInputs in = Coast();
  in.phase = Phase::kRecovery;
  AssertLatched(e.Update(in), LatchReason::kDrogue);
}

void Envelope_Latch_LowSpeedInCoast() {
  Envelope e = Active(Params());
  AssertLatched(e.Update(Coast(39.0F)), LatchReason::kLowSpeed);
}

void Envelope_NoLowSpeedLatchInBoost() {
  Envelope e = Active(Params(), Phase::kThrusting);
  EnvelopeInputs in = Coast(10.0F);
  in.phase = Phase::kThrusting;
  EnvelopeDecision d = e.Update(in);
  TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kActive), static_cast<int>(d.state));
}

void Envelope_Latch_Tilt() {
  Envelope e = Active(Params());
  EnvelopeInputs in = Coast();
  in.tilt_mag_deg = 20.0F;
  TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kActive), static_cast<int>(e.Update(in).state));
  in.tilt_mag_deg = 20.5F;
  AssertLatched(e.Update(in), LatchReason::kTilt);
}

void Envelope_Latch_GyroSat() {
  Envelope e = Active(Params());
  EnvelopeInputs in = Coast();
  in.max_abs_gyro_dps = 1899.0F;
  TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kActive), static_cast<int>(e.Update(in).state));
  in.max_abs_gyro_dps = 1900.0F;
  AssertLatched(e.Update(in), LatchReason::kGyroSat);
}

void Envelope_Latch_ImuFault() {
  Envelope e = Active(Params());
  EnvelopeInputs in = Coast();
  in.imu_faulty = true;
  AssertLatched(e.Update(in), LatchReason::kImuFault);
}

void Envelope_Authority_LatchesAfter0p5s() {
  Envelope e = Active(Params());
  EnvelopeInputs in = Coast();
  in.dt_s = 0.01F;
  in.roll_rate_dps = 200.0F;
  in.roll_cmd_deg = -8.0F;
  in.roll_cmd_saturated = true;
  EnvelopeDecision d;
  for (int i = 0; i < 40; ++i) d = e.Update(in);
  TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kActive), static_cast<int>(d.state));
  for (int i = 0; i < 20; ++i) d = e.Update(in);
  AssertLatched(d, LatchReason::kAuthority);
}

void Envelope_Authority_NotWhenCommandAgreesWithRate() {
  Envelope e = Active(Params());
  EnvelopeInputs in = Coast();
  in.dt_s = 0.01F;
  in.roll_rate_dps = 200.0F;
  in.roll_cmd_deg = 8.0F;
  in.roll_cmd_saturated = true;
  EnvelopeDecision d;
  for (int i = 0; i < 100; ++i) d = e.Update(in);
  TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kActive), static_cast<int>(d.state));
  // Not in SHADOW either (CLOSED only), and the timer restarts when the condition breaks.
  Envelope s = Active(Params(FlightMode::kShadow));
  in.roll_cmd_deg = -8.0F;
  for (int i = 0; i < 100; ++i) d = s.Update(in);
  TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kActive), static_cast<int>(d.state));
  Envelope r = Active(Params());
  for (int i = 0; i < 40; ++i) d = r.Update(in);
  in.roll_cmd_saturated = false;
  d = r.Update(in);
  in.roll_cmd_saturated = true;
  for (int i = 0; i < 40; ++i) d = r.Update(in);
  TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kActive), static_cast<int>(d.state));
}

void Envelope_SimRunning_LatchesNeutral() {
  Envelope e = Active(Params());
  EnvelopeInputs in = Coast();
  in.sim_running = true;
  AssertLatched(e.Update(in), LatchReason::kSim);

  Envelope pad;
  pad.Configure(Params());
  pad.Reset();
  in.phase = Phase::kReady;
  AssertLatched(pad.Update(in), LatchReason::kSim);
}

void Envelope_Shadow_SourceTrim() {
  Envelope e = Active(Params(FlightMode::kShadow));
  EnvelopeDecision d = e.Update(Coast());
  TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kActive), static_cast<int>(d.state));
  TEST_ASSERT_EQUAL(static_cast<int>(Source::kTrim), static_cast<int>(d.source));
}

void Envelope_Step_DoubletTimingAndSigns() {
  Envelope e;
  e.Configure(Params(FlightMode::kStep));
  e.Reset();
  EnvelopeInputs in = Coast();
  in.dt_s = 0.05F;
  // Update k is evaluated at coast time t = k * dt (the first coasting update is t = 0).
  EnvelopeDecision d;
  float at_0p25 = 9, at_0p35 = 9, at_0p65 = 9, at_0p95 = 9, at_2p35 = 9, at_2p65 = 9, at_4p35 = 9;
  for (int k = 0; k <= 110; ++k) {
    d = e.Update(in);
    if (k == 5) at_0p25 = d.step_cmd_deg;
    if (k == 7) at_0p35 = d.step_cmd_deg;
    if (k == 13) at_0p65 = d.step_cmd_deg;
    if (k == 19) at_0p95 = d.step_cmd_deg;
    if (k == 47) at_2p35 = d.step_cmd_deg;
    if (k == 53) at_2p65 = d.step_cmd_deg;
    if (k == 87) {
      at_4p35 = d.step_cmd_deg;
      TEST_ASSERT_EQUAL(static_cast<int>(Source::kStep), static_cast<int>(d.source));
    }
    if (k == 96) TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kActive), static_cast<int>(d.state));
  }
  TEST_ASSERT_FLOAT_WITHIN(1e-6F, 0.0F, at_0p25);
  TEST_ASSERT_FLOAT_WITHIN(1e-6F, 3.0F, at_0p35);
  TEST_ASSERT_FLOAT_WITHIN(1e-6F, -3.0F, at_0p65);
  TEST_ASSERT_FLOAT_WITHIN(1e-6F, 0.0F, at_0p95);
  TEST_ASSERT_FLOAT_WITHIN(1e-6F, -3.0F, at_2p35);
  TEST_ASSERT_FLOAT_WITHIN(1e-6F, 3.0F, at_2p65);
  TEST_ASSERT_FLOAT_WITHIN(1e-6F, 3.0F, at_4p35);
  AssertLatched(d, LatchReason::kStepDone);
}

void Envelope_Step_RollRateAbort() {
  Envelope e = Active(Params(FlightMode::kStep));
  EnvelopeInputs in = Coast();
  in.roll_rate_dps = 360.0F;
  TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kActive), static_cast<int>(e.Update(in).state));
  in.roll_rate_dps = -361.0F;
  AssertLatched(e.Update(in), LatchReason::kStepRollRate);
}

void Envelope_TestOverride_OnlyOnPad() {
  Envelope e;
  e.Configure(Params());
  e.Reset();
  TEST_ASSERT_FALSE(e.StartTest({TestCommand::Kind::kSweep, 0.0F, 5.0F}, Phase::kThrusting));
  TEST_ASSERT_FALSE(e.StartTest({TestCommand::Kind::kSweep, 0.0F, 5.0F}, Phase::kCoasting));
  TEST_ASSERT_TRUE(e.StartTest({TestCommand::Kind::kSweep, 0.0F, 5.0F}, Phase::kReady));
  EnvelopeInputs in;
  in.phase = Phase::kReady;
  in.dt_s = 0.1F;
  EnvelopeDecision d = e.Update(in);
  TEST_ASSERT_EQUAL(static_cast<int>(Source::kTest), static_cast<int>(d.source));
  TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kNeutralPad), static_cast<int>(d.state));
  e.CancelTest();
  TEST_ASSERT_EQUAL(static_cast<int>(Source::kTrim), static_cast<int>(e.Update(in).source));
  Envelope a = Active(Params());
  TEST_ASSERT_FALSE(a.StartTest({TestCommand::Kind::kSweep, 0.0F, 5.0F}, Phase::kReady));
}

void Envelope_TestOverride_CancelledByPhaseChange() {
  Envelope e;
  e.Configure(Params());
  e.Reset();
  TEST_ASSERT_TRUE(e.StartTest({TestCommand::Kind::kLive, 50.0F, 5.0F}, Phase::kCalibrating));
  EnvelopeInputs in;
  in.phase = Phase::kCalibrating;
  in.dt_s = 0.1F;
  TEST_ASSERT_EQUAL(static_cast<int>(Source::kTest), static_cast<int>(e.Update(in).source));
  in.phase = Phase::kReady;
  TEST_ASSERT_EQUAL(static_cast<int>(Source::kTrim), static_cast<int>(e.Update(in).source));
  in.phase = Phase::kCalibrating;
  TEST_ASSERT_EQUAL(static_cast<int>(Source::kTrim), static_cast<int>(e.Update(in).source));
}

void Envelope_TestOverride_TimesOut() {
  Envelope e;
  e.Configure(Params());
  e.Reset();
  TEST_ASSERT_TRUE(e.StartTest({TestCommand::Kind::kSweep, 0.0F, 1.0F}, Phase::kReady));
  EnvelopeInputs in;
  in.phase = Phase::kReady;
  in.dt_s = 0.25F;
  TEST_ASSERT_EQUAL(static_cast<int>(Source::kTest), static_cast<int>(e.Update(in).source));  // 0.25
  TEST_ASSERT_EQUAL(static_cast<int>(Source::kTest), static_cast<int>(e.Update(in).source));  // 0.5
  TEST_ASSERT_EQUAL(static_cast<int>(Source::kTest), static_cast<int>(e.Update(in).source));  // 0.75
  TEST_ASSERT_EQUAL(static_cast<int>(Source::kTrim), static_cast<int>(e.Update(in).source));  // 1.0
}

void Envelope_LatchIsPermanent() {
  Envelope e = Active(Params());
  EnvelopeInputs bad = Coast();
  bad.imu_faulty = true;
  AssertLatched(e.Update(bad), LatchReason::kImuFault);
  // Everything healthy again, still latched with the original reason.
  AssertLatched(e.Update(Coast()), LatchReason::kImuFault);
  EnvelopeInputs th = Coast();
  th.phase = Phase::kThrusting;
  AssertLatched(e.Update(th), LatchReason::kImuFault);
  e.Reset();
  TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kActive), static_cast<int>(e.Update(Coast()).state));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(Envelope_StartsInNeutralPad_UntilThrusting);
  RUN_TEST(Envelope_Activates_AtVenable);
  RUN_TEST(Envelope_Latch_Drogue);
  RUN_TEST(Envelope_Latch_LowSpeedInCoast);
  RUN_TEST(Envelope_NoLowSpeedLatchInBoost);
  RUN_TEST(Envelope_Latch_Tilt);
  RUN_TEST(Envelope_Latch_GyroSat);
  RUN_TEST(Envelope_Latch_ImuFault);
  RUN_TEST(Envelope_Authority_LatchesAfter0p5s);
  RUN_TEST(Envelope_Authority_NotWhenCommandAgreesWithRate);
  RUN_TEST(Envelope_SimRunning_LatchesNeutral);
  RUN_TEST(Envelope_Shadow_SourceTrim);
  RUN_TEST(Envelope_Step_DoubletTimingAndSigns);
  RUN_TEST(Envelope_Step_RollRateAbort);
  RUN_TEST(Envelope_TestOverride_OnlyOnPad);
  RUN_TEST(Envelope_TestOverride_CancelledByPhaseChange);
  RUN_TEST(Envelope_TestOverride_TimesOut);
  RUN_TEST(Envelope_LatchIsPermanent);
  return UNITY_END();
}
