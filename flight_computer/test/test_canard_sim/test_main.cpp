/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include <unity.h>

#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "control/canard/canard_control.hpp"
#include "vehicle.hpp"

using namespace canard;

void setUp() {}
void tearDown() {}

namespace {

constexpr double kDegToRad = vehicle::kPi / 180.0;
constexpr double kRadToDeg = 180.0 / vehicle::kPi;
constexpr double kGravity = 9.81;
constexpr double kPlantDtS = 0.001;
constexpr int kStepsPerTick = 10;  // controller at 100 Hz
constexpr float kTickS = 0.01F;
constexpr double kPadS = 12.0;  // six levelling time constants before liftoff
/// Board mounting used throughout: board x along the rocket, pointing aft (long_axis 0, long_sign -1).
constexpr uint8_t kLongAxis = 0;
constexpr float kLongSign = -1.0F;

/// Inverse of BoardToBody for kLongAxis/kLongSign: body (t1, t2, roll) -> board.
void BodyToBoard(const double body[3], float board[3]) {
  board[0] = static_cast<float>(kLongSign * body[2]);
  board[1] = static_cast<float>(body[0]);
  board[2] = static_cast<float>(kLongSign * body[1]);
}

/// Tilt gains for the FULL_4 sims: the tuning-harness values (kd for a well-damped weathercock mode, gain margin
/// ~15; kp = 1 deg/deg). They are not the firmware defaults: those stay 0 until the stage-2 flight-path sim exists
/// (Ruling R23), so the tilt sims set these explicitly to keep the tilt path exercised.
constexpr float kSimTiltKp = 1.0F;
constexpr float kSimTiltKd = 0.2F;

CanardParams SimParams(Layout layout) {
  CanardParams p;
  p.layout = layout;
  p.mode = FlightMode::kClosed;
  p.v_enable = 15.0F;
  p.v_ref = 100.0F;
  p.tilt_v_min = 70.0F;
  p.max_defl_deg = 8.0F;
  p.abort_tilt_deg = 30.0F;
  p.roll_kp = kDefaultRollKp;
  p.roll_ki = kDefaultRollKi;
  p.tilt_kp = kSimTiltKp;
  p.tilt_kd = kSimTiltKd;
  p.slew_dps = kDefaultSlewDps;
  return p;
}

struct Case {
  CanardParams params = SimParams(Layout::kRoll2);
  double v_mps = 100.0;
  double eff = 1.0;            // canard roll effectiveness (canard-fin interaction)
  double inertia_scale = 1.0;  // scales both moments of inertia
  double i_roll = vehicle::kIrollLiftoff;
  double i_pitch = vehicle::kIpitchLiftoff;
  double cg_m = vehicle::kCgLiftoffM;
  double p0_dps = 0.0;
  double tilt0_deg = 0.0;
  double tilt0_azimuth_deg = 30.0;  // lean direction, from +t1 toward +t2
  double disturbance_eq_deg = 0.0;  // q-proportional roll moment, as an equivalent all-canard deflection
  int stuck_output = -1;
  double stuck_deg = 0.0;
  double duration_s = 3.0;
};

struct Trace {
  std::vector<double> t, p_dps, tilt_deg;  // plant truth at every 1 ms step (flight only)
  std::vector<ControlOutputs> out;         // controller outputs at every tick (flight only)
  std::vector<double> out_t;
  double est_tilt_at_liftoff_deg[2] = {0.0, 0.0};
  double true_tilt_at_liftoff_deg[2] = {0.0, 0.0};
};

/// Plant: rigid body (quaternion body->world), 1-DOF roll aero plus 2-DOF pitch/yaw aero at constant V with the
/// velocity vertical. Canard k sits at body azimuth k * 90 deg; positive deflection gives +roll moment, i.e. a
/// side force along the tangential direction (-sin, cos). ROLL_2 output 0 drives canards 0 and 2, output 1
/// drives canards 1 and 3.
class Plant {
 public:
  explicit Plant(const Case& c) : m_c(c) {
    m_ir = c.i_roll * c.inertia_scale;
    m_it = c.i_pitch * c.inertia_scale;
    // Initial lean: rotate the body about a horizontal axis so the nose leans toward the given body azimuth.
    const double a = c.tilt0_azimuth_deg * kDegToRad;
    const double half = 0.5 * c.tilt0_deg * kDegToRad;
    // Rotating about axis z x d by +tilt moves the nose toward d = (cos a, sin a, 0).
    const double ax[3] = {-std::sin(a), std::cos(a), 0.0};
    m_q[0] = std::cos(half);
    for (int i = 0; i < 3; ++i) m_q[i + 1] = std::sin(half) * ax[i];
  }

  /// World up expressed in the body frame (third row of R).
  void UpBody(double up[3]) const {
    const double w = m_q[0], x = m_q[1], y = m_q[2], z = m_q[3];
    up[0] = 2.0 * (x * z - w * y);
    up[1] = 2.0 * (y * z + w * x);
    up[2] = 1.0 - 2.0 * (x * x + y * y);
  }
  double TiltMagDeg() const {
    double up[3];
    UpBody(up);
    return std::acos(std::fmax(-1.0, std::fmin(1.0, up[2]))) * kRadToDeg;
  }
  void TiltDeg(double out[2]) const {
    double up[3];
    UpBody(up);
    out[0] = std::asin(-up[0]) * kRadToDeg;
    out[1] = std::asin(-up[1]) * kRadToDeg;
  }
  double RollRateDps() const { return m_w[2] * kRadToDeg; }
  void GyroBodyDps(double g[3]) const {
    for (int i = 0; i < 3; ++i) g[i] = m_w[i] * kRadToDeg;
  }
  void SetRollRateDps(double p) { m_w[2] = p * kDegToRad; }

  /// Advances the servos toward their targets and the rigid body by dt.
  void Step(const double target_deg[4], double dt) {
    const int n = m_c.params.layout == Layout::kFull4 ? 4 : 2;
    const double max_step = vehicle::kServoRateDps * dt;
    for (int k = 0; k < n; ++k) {
      if (k == m_c.stuck_output) {
        m_servo[k] = m_c.stuck_deg;
        continue;
      }
      const double e = target_deg[k] - m_servo[k];
      m_servo[k] += std::fmax(-max_step, std::fmin(max_step, e));
    }
    double delta[4];
    for (int k = 0; k < 4; ++k) delta[k] = (n == 4) ? m_servo[k] : m_servo[k % 2];

    const double v = m_c.v_mps;
    const double q = 0.5 * vehicle::kRho * v * v;
    const double qa = q * vehicle::kRefAreaM2;
    const double arm = vehicle::CanardArmM(m_c.cg_m);
    double m[3] = {0.0, 0.0, 0.0};
    double fx = 0.0, fy = 0.0;
    for (int k = 0; k < 4; ++k) {
      const double phi = k * 0.5 * vehicle::kPi;
      m[2] += m_c.eff * q * (vehicle::kCanardRollNmPerDegPa / 4.0) * delta[k];
      const double f = qa * vehicle::kCnDeltaPerCanard * delta[k] * kDegToRad;
      fx += -std::sin(phi) * f;
      fy += std::cos(phi) * f;
    }
    // Canard side force applied at +arm along the nose axis: M = (0, 0, arm) x (fx, fy, 0).
    m[0] += -arm * fy;
    m[1] += arm * fx;
    m[2] += q * vehicle::kCanardRollNmPerDegPa * m_c.disturbance_eq_deg;
    m[2] -= (q / v) * vehicle::kRollDampM4 * m_w[2];
    // Weathercock restoring moment toward the (vertical) velocity: k * (e_z x up), and pitch damping.
    double up[3];
    UpBody(up);
    const double k_rest = qa * vehicle::RestoringM(m_c.cg_m);
    m[0] += k_rest * (-up[1]);
    m[1] += k_rest * up[0];
    const double c_pitch = qa * vehicle::PitchDampM2(m_c.cg_m) / v;
    m[0] -= c_pitch * m_w[0];
    m[1] -= c_pitch * m_w[1];
    // Euler's equations, axisymmetric body (It, It, Ir); semi-implicit Euler.
    m_w[0] += dt * (m[0] - (m_ir - m_it) * m_w[1] * m_w[2]) / m_it;
    m_w[1] += dt * (m[1] - (m_it - m_ir) * m_w[2] * m_w[0]) / m_it;
    m_w[2] += dt * m[2] / m_ir;
    Integrate(dt);
  }

 private:
  void Integrate(double dt) {
    const double hx = 0.5 * m_w[0] * dt, hy = 0.5 * m_w[1] * dt, hz = 0.5 * m_w[2] * dt;
    const double w = m_q[0], x = m_q[1], y = m_q[2], z = m_q[3];
    double n[4] = {w - x * hx - y * hy - z * hz, x + w * hx + y * hz - z * hy, y + w * hy - x * hz + z * hx,
                   z + w * hz + x * hy - y * hx};
    const double norm = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2] + n[3] * n[3]);
    for (int i = 0; i < 4; ++i) m_q[i] = n[i] / norm;
  }

  Case m_c;
  double m_ir = 0.0, m_it = 0.0;
  double m_q[4] = {1.0, 0.0, 0.0, 0.0};
  double m_w[3] = {0.0, 0.0, 0.0};
  double m_servo[4] = {0.0, 0.0, 0.0, 0.0};
};

ControlInputs Inputs(const Plant& plant, Phase phase, double v_mps, bool on_pad) {
  ControlInputs in;
  double g[3];
  plant.GyroBodyDps(g);
  BodyToBoard(g, in.gyro_board_dps);
  double acc[3] = {0.0, 0.0, -5.0};  // coast: drag deceleration only
  if (on_pad) {
    double up[3];
    plant.UpBody(up);
    for (int i = 0; i < 3; ++i) acc[i] = kGravity * up[i];
  }
  BodyToBoard(acc, in.acc_board_mps2);
  in.long_axis = kLongAxis;
  in.long_sign = kLongSign;
  in.velocity_mps = static_cast<float>(v_mps);
  in.altitude_agl_m = 0.0F;
  in.phase = phase;
  in.dt_s = kTickS;
  return in;
}

/// Pad levelling (kReady, at rest) then flight in kCoasting at constant V, with the initial roll rate applied at
/// liftoff. The controller runs every 10 ms; its output reaches the servos one tick later.
Trace Run(const Case& c) {
  Trace tr;
  Plant plant(c);
  CanardControl ctl;
  ctl.Configure(c.params);
  const int pad_ticks = static_cast<int>(kPadS / kTickS);
  for (int i = 0; i < pad_ticks; ++i) {
    (void)ctl.Update(Inputs(plant, Phase::kReady, 0.0, true));
  }
  plant.TiltDeg(tr.true_tilt_at_liftoff_deg);
  plant.SetRollRateDps(c.p0_dps);

  double target[4] = {0.0, 0.0, 0.0, 0.0};
  double pending[4] = {0.0, 0.0, 0.0, 0.0};
  const int steps = static_cast<int>(c.duration_s / kPlantDtS + 0.5);
  for (int s = 0; s < steps; ++s) {
    const double t = s * kPlantDtS;
    if (s % kStepsPerTick == 0) {
      for (int k = 0; k < 4; ++k) target[k] = pending[k];
      const ControlOutputs out = ctl.Update(Inputs(plant, Phase::kCoasting, c.v_mps, false));
      if (s == 0) {
        tr.est_tilt_at_liftoff_deg[0] = out.tilt_deg[0];
        tr.est_tilt_at_liftoff_deg[1] = out.tilt_deg[1];
      }
      for (int k = 0; k < 4; ++k) pending[k] = out.defl_deg[k];
      tr.out.push_back(out);
      tr.out_t.push_back(t);
    }
    tr.t.push_back(t);
    tr.p_dps.push_back(plant.RollRateDps());
    tr.tilt_deg.push_back(plant.TiltMagDeg());
    plant.Step(target, kPlantDtS);
  }
  return tr;
}

double MaxAbsFrom(const std::vector<double>& t, const std::vector<double>& x, double t0, double t1 = 1e9) {
  double m = 0.0;
  for (size_t i = 0; i < t.size(); ++i) {
    if (t[i] >= t0 && t[i] <= t1) m = std::fmax(m, std::fabs(x[i]));
  }
  return m;
}

/// First time after which |x| stays below the limit to the end of the trace (or a large value if never).
double SettleTime(const std::vector<double>& t, const std::vector<double>& x, double limit) {
  double ts = 0.0;
  for (size_t i = 0; i < t.size(); ++i) {
    if (std::fabs(x[i]) >= limit) ts = (i + 1 < t.size()) ? t[i + 1] : 1e9;
  }
  return ts;
}

void AssertStillActive(const Trace& tr) {
  const ControlOutputs& last = tr.out.back();
  TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kActive), static_cast<int>(last.state));
  TEST_ASSERT_EQUAL(static_cast<int>(LatchReason::kNone), static_cast<int>(last.reason));
}

/// No oscillation growth: the peak over the last second is not above the peak over the second before it
/// (plus a small floor for numerical noise), and never above the initial value.
void AssertNoLateGrowth(const Trace& tr, const std::vector<double>& x, double floor) {
  const double end = tr.t.back();
  const double last = MaxAbsFrom(tr.t, x, end - 1.0);
  const double before = MaxAbsFrom(tr.t, x, end - 2.0, end - 1.0);
  TEST_ASSERT_TRUE(last <= before + floor);
}

void AssertNoGrowth(const Trace& tr, const std::vector<double>& x, double x0, double floor) {
  AssertNoLateGrowth(tr, x, floor);
  TEST_ASSERT_TRUE(MaxAbsFrom(tr.t, x, 0.0) <= std::fabs(x0) + floor);
}

/// The disturbance roll is removed: |p| over the last 0.2 s of the run is below 2 deg/s. With zero gains the
/// disturbance leaves a steady 4-23 deg/s, so this pins the controller rather than the aerodynamic damping.
constexpr double kResidualRollDps = 2.0;
void AssertResidualRoll(const Trace& tr, const char* name) {
  const double residual = MaxAbsFrom(tr.t, tr.p_dps, tr.t.back() - 0.2);
  std::printf("SIM %s: residual %.3f deg/s (criterion < %.1f)\n", name, residual, kResidualRollDps);
  TEST_ASSERT_TRUE(residual < kResidualRollDps);
}

/// Largest tilt part of the FULL_4 command over the trace: a pure roll command moves all four canards alike, so any
/// spread around their mean is tilt. Shows the tilt path is exercised (with zero tilt gains this stays 0).
double MaxTiltCommandDeg(const Trace& tr) {
  double m = 0.0;
  for (const ControlOutputs& o : tr.out) {
    const double mean = (o.cmd_deg[0] + o.cmd_deg[1] + o.cmd_deg[2] + o.cmd_deg[3]) / 4.0;
    for (const float d : o.cmd_deg) m = std::fmax(m, std::fabs(d - mean));
  }
  return m;
}

void Report(const char* name, double metric, const char* unit) { std::printf("SIM %s: %.3f %s\n", name, metric, unit); }

Case RollCase(double v_mps, double p0_dps) {
  Case c;
  c.params = SimParams(Layout::kRoll2);
  c.v_mps = v_mps;
  c.p0_dps = p0_dps;
  c.disturbance_eq_deg = vehicle::kDisturbanceEqDeg;
  return c;
}

void CheckDampsAt100(double inertia_scale, const char* name) {
  Case c = RollCase(100.0, 360.0);
  c.inertia_scale = inertia_scale;
  c.duration_s = 3.0;
  const Trace tr = Run(c);
  const double ts = SettleTime(tr.t, tr.p_dps, 20.0);
  Report(name, ts, "s to |p| < 20 deg/s (criterion <= 1 s)");
  TEST_ASSERT_TRUE(ts <= 1.0);
  AssertResidualRoll(tr, name);
  AssertStillActive(tr);
}

}  // namespace

// ---------------------------------------------------------------- vehicle model

void Vehicle_CanardsAloneStop360In0p12s() {
  // canard_estimate.py: 8 deg at 100 m/s, "stops 360 deg/s in 0.12 s" (canards alone, no damping). rho 1.2 is the
  // script's own density, used here to reproduce its figure; the plant uses ISA 1.225 like the firmware schedule.
  const double q = 0.5 * 1.2 * 100.0 * 100.0;
  const double accel = q * vehicle::kCanardRollNmPerDegPa * 8.0 / vehicle::kIrollLiftoff;
  const double t_stop = 2.0 * vehicle::kPi / accel;
  Report("Vehicle_CanardsAloneStop360In0p12s", t_stop, "s");
  TEST_ASSERT_TRUE(std::fabs(t_stop - 0.12) < 0.005);
}

/// Holds the board still and upright on the pad (READY) for the given time, as the firmware does before liftoff:
/// the frame check needs pad samples with the body z specific force at +1 g before control may activate.
void PadUpright(CanardControl& ctl, double seconds = 2.0) {
  ControlInputs in;
  const double acc[3] = {0.0, 0.0, kGravity};
  BodyToBoard(acc, in.acc_board_mps2);
  in.long_axis = kLongAxis;
  in.long_sign = kLongSign;
  in.phase = Phase::kReady;
  in.dt_s = kTickS;
  for (int i = 0; i < static_cast<int>(seconds / kTickS); ++i) (void)ctl.Update(in);
}

/// Pad (READY, at rest) with the given body-frame specific force, then one coasting step at 100 m/s.
/// long_sign is what the glue passes; the board is mounted with kLongSign.
ControlOutputs PadThenActivate(const double acc_body[3], float long_sign, double pad_s) {
  CanardControl ctl;
  ctl.Configure(SimParams(Layout::kRoll2));
  ControlInputs pad;
  BodyToBoard(acc_body, pad.acc_board_mps2);
  pad.long_axis = kLongAxis;
  pad.long_sign = long_sign;
  pad.phase = Phase::kReady;
  pad.dt_s = kTickS;
  for (int i = 0; i < static_cast<int>(pad_s / kTickS); ++i) {
    const ControlOutputs out = ctl.Update(pad);
    TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kNeutralPad), static_cast<int>(out.state));
  }
  ControlInputs fly = pad;
  fly.acc_board_mps2[0] = fly.acc_board_mps2[1] = fly.acc_board_mps2[2] = 0.0F;
  fly.phase = Phase::kCoasting;
  fly.velocity_mps = 100.0F;
  return ctl.Update(fly);
}

// ---------------------------------------------------------------- defaults

void Defaults_TiltGainsZero_UntilStage2Sim() {
  // Ruling R23: FULL_4 stops roll only until the stage-2 flight-path sim validates tilt gains.
  TEST_ASSERT_EQUAL_FLOAT(0.0F, kDefaultTiltKp);
  TEST_ASSERT_EQUAL_FLOAT(0.0F, kDefaultTiltKd);
}

// ---------------------------------------------------------------- facade

void Control_Shadow_LogsWouldBeCommand_SendsZero() {
  CanardParams p = SimParams(Layout::kRoll2);
  p.mode = FlightMode::kShadow;
  CanardControl ctl;
  ctl.Configure(p);
  PadUpright(ctl);
  ControlInputs in;
  in.long_axis = kLongAxis;
  in.long_sign = kLongSign;
  in.phase = Phase::kCoasting;
  in.velocity_mps = 100.0F;
  in.dt_s = kTickS;
  const double body[3] = {0.0, 0.0, 50.0};
  BodyToBoard(body, in.gyro_board_dps);
  const ControlOutputs out = ctl.Update(in);
  TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kActive), static_cast<int>(out.state));
  TEST_ASSERT_EQUAL_UINT8(2, out.n_outputs);
  TEST_ASSERT_FLOAT_WITHIN(1e-3F, 50.0F, out.roll_rate_dps);
  TEST_ASSERT_TRUE(out.cmd_deg[0] < -0.1F);  // opposes +roll
  TEST_ASSERT_EQUAL_FLOAT(out.cmd_deg[0], out.cmd_deg[1]);
  for (const float d : out.defl_deg) TEST_ASSERT_EQUAL_FLOAT(0.0F, d);
}

void Control_Closed_SlewLimitedAtDefaultWhenUnset() {
  CanardParams p = SimParams(Layout::kRoll2);
  p.slew_dps = 0.0F;  // not set: the default 300 deg/s applies
  CanardControl ctl;
  ctl.Configure(p);
  PadUpright(ctl);
  ControlInputs in;
  in.long_axis = kLongAxis;
  in.long_sign = kLongSign;
  in.phase = Phase::kCoasting;
  in.velocity_mps = 100.0F;
  in.dt_s = kTickS;
  const double body[3] = {0.0, 0.0, 1000.0};
  BodyToBoard(body, in.gyro_board_dps);
  ControlOutputs out = ctl.Update(in);
  TEST_ASSERT_FLOAT_WITHIN(1e-4F, -8.0F, out.cmd_deg[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4F, -kDefaultSlewDps * kTickS, out.defl_deg[0]);
  out = ctl.Update(in);
  TEST_ASSERT_FLOAT_WITHIN(1e-4F, -2.0F * kDefaultSlewDps * kTickS, out.defl_deg[1]);
  for (int i = 0; i < 5; ++i) out = ctl.Update(in);
  TEST_ASSERT_FLOAT_WITHIN(1e-4F, -8.0F, out.defl_deg[0]);
}

void Control_NonFinite_ZerosAndLatchesImuFault() {
  CanardControl ctl;
  ctl.Configure(SimParams(Layout::kFull4));
  PadUpright(ctl);
  ControlInputs in;
  in.long_axis = kLongAxis;
  in.long_sign = kLongSign;
  in.phase = Phase::kCoasting;
  in.velocity_mps = 100.0F;
  in.dt_s = kTickS;
  const double body[3] = {20.0, -10.0, 300.0};
  BodyToBoard(body, in.gyro_board_dps);
  for (int i = 0; i < 5; ++i) (void)ctl.Update(in);
  ControlInputs bad = in;
  bad.acc_board_mps2[1] = std::nanf("");
  ControlOutputs out = ctl.Update(bad);
  TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kNeutralLatched), static_cast<int>(out.state));
  TEST_ASSERT_EQUAL(static_cast<int>(LatchReason::kImuFault), static_cast<int>(out.reason));
  for (int k = 0; k < 4; ++k) {
    TEST_ASSERT_EQUAL_FLOAT(0.0F, out.defl_deg[k]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, out.cmd_deg[k]);
  }
  // The estimator never saw the NaN: the next finite step still reports finite values.
  out = ctl.Update(in);
  TEST_ASSERT_TRUE(std::isfinite(out.roll_rate_dps) && std::isfinite(out.tilt_deg[0]) &&
                   std::isfinite(out.tilt_deg[1]) && std::isfinite(out.cmd_deg[0]));
  for (const float d : out.defl_deg) TEST_ASSERT_EQUAL_FLOAT(0.0F, d);
}

void Control_Latched_LogsWouldBeCommand_SendsZero() {
  CanardControl ctl;
  ctl.Configure(SimParams(Layout::kRoll2));
  PadUpright(ctl);
  ControlInputs in;
  in.long_axis = kLongAxis;
  in.long_sign = kLongSign;
  in.phase = Phase::kCoasting;
  in.velocity_mps = 100.0F;
  in.dt_s = kTickS;
  const double body[3] = {0.0, 0.0, 50.0};
  BodyToBoard(body, in.gyro_board_dps);
  ControlOutputs out = ctl.Update(in);
  TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kActive), static_cast<int>(out.state));
  TEST_ASSERT_TRUE(out.defl_deg[0] < -0.1F);
  ControlInputs faulty = in;
  faulty.imu_faulty = true;
  out = ctl.Update(faulty);
  TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kNeutralLatched), static_cast<int>(out.state));
  TEST_ASSERT_EQUAL(static_cast<int>(LatchReason::kImuFault), static_cast<int>(out.reason));
  for (int i = 0; i < 5; ++i) out = ctl.Update(in);
  TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kNeutralLatched), static_cast<int>(out.state));
  TEST_ASSERT_TRUE(out.cmd_deg[0] < -0.1F);  // the controller still runs and logs its would-be command
  TEST_ASSERT_EQUAL_FLOAT(out.cmd_deg[0], out.cmd_deg[1]);
  for (const float d : out.defl_deg) TEST_ASSERT_EQUAL_FLOAT(0.0F, d);
}

void Control_Off_NoOutputs() {
  CanardParams p = SimParams(Layout::kOff);
  CanardControl ctl;
  ctl.Configure(p);
  ControlInputs in;
  in.phase = Phase::kCoasting;
  in.velocity_mps = 100.0F;
  in.dt_s = kTickS;
  in.gyro_board_dps[0] = 300.0F;
  const ControlOutputs out = ctl.Update(in);
  TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kDisabled), static_cast<int>(out.state));
  TEST_ASSERT_EQUAL_UINT8(0, out.n_outputs);
  for (int k = 0; k < 4; ++k) {
    TEST_ASSERT_EQUAL_FLOAT(0.0F, out.defl_deg[k]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, out.cmd_deg[k]);
  }
}

void Control_Step_MixesChosenAxis() {
  CanardParams p = SimParams(Layout::kFull4);
  p.mode = FlightMode::kStep;
  p.step_axis = StepAxis::kT1;
  p.step_defl_deg = 3.0F;
  p.step_s = 0.3F;
  p.step_delay_s = 0.0F;
  p.step_period_s = 2.0F;
  p.step_count = 1;
  CanardControl ctl;
  ctl.Configure(p);
  PadUpright(ctl);
  ControlInputs in;
  in.long_axis = kLongAxis;
  in.long_sign = kLongSign;
  in.phase = Phase::kCoasting;
  in.velocity_mps = 100.0F;
  in.dt_s = kTickS;
  const ControlOutputs out = ctl.Update(in);
  // +3 deg on t1: tilt components {u_t2, -u_t1, -u_t2, u_t1}.
  const float expect[4] = {0.0F, -3.0F, 0.0F, 3.0F};
  for (int k = 0; k < 4; ++k) {
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, expect[k], out.cmd_deg[k]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, expect[k], out.defl_deg[k]);  // 3 deg is exactly one 300 deg/s tick
  }
}

void Control_TestSweep_StepsEachOutput() {
  CanardControl ctl;
  ctl.Configure(SimParams(Layout::kRoll2));
  ControlInputs in;
  in.long_axis = kLongAxis;
  in.long_sign = kLongSign;
  in.phase = Phase::kReady;
  in.dt_s = kTickS;
  in.acc_board_mps2[0] = static_cast<float>(kLongSign * kGravity);  // body +roll axis up
  (void)ctl.Update(in);
  TestCommand cmd;
  cmd.kind = TestCommand::Kind::kSweep;
  cmd.duration_s = 10.0F;
  TEST_ASSERT_TRUE(ctl.StartTest(cmd));
  // Per output: +max, -max, trim for 1 s each, in turn. Sample in the middle of every second.
  const float expect[7][2] = {{8, 0}, {-8, 0}, {0, 0}, {0, 8}, {0, -8}, {0, 0}, {0, 0}};
  ControlOutputs out;
  for (int i = 0; i < 700; ++i) {
    out = ctl.Update(in);
    if (i % 100 == 50) {
      const int sec = i / 100;
      TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kNeutralPad), static_cast<int>(out.state));
      TEST_ASSERT_FLOAT_WITHIN(1e-4F, expect[sec][0], out.defl_deg[0]);
      TEST_ASSERT_FLOAT_WITHIN(1e-4F, expect[sec][1], out.defl_deg[1]);
      TEST_ASSERT_FLOAT_WITHIN(1e-4F, expect[sec][0], out.cmd_deg[0]);
    }
  }
  ctl.CancelTest();
  out = ctl.Update(in);
  for (int i = 0; i < 5; ++i) out = ctl.Update(in);
  TEST_ASSERT_EQUAL_FLOAT(0.0F, out.defl_deg[0]);
}

void Control_TestLive_UsesForcedVelocity() {
  CanardParams p = SimParams(Layout::kRoll2);
  CanardControl ctl;
  ctl.Configure(p);
  ControlInputs in;
  in.long_axis = kLongAxis;
  in.long_sign = kLongSign;
  in.phase = Phase::kReady;
  in.dt_s = kTickS;
  in.acc_board_mps2[0] = static_cast<float>(kLongSign * kGravity);  // body +roll axis up
  (void)ctl.Update(in);
  TestCommand cmd;
  cmd.kind = TestCommand::Kind::kLive;
  cmd.forced_v_mps = 100.0F;
  cmd.duration_s = 1.0F;
  TEST_ASSERT_TRUE(ctl.StartTest(cmd));
  const double body[3] = {0.0, 0.0, 30.0};
  BodyToBoard(body, in.gyro_board_dps);
  ControlOutputs out = ctl.Update(in);
  // At v_ref the gain scale is 1: the first command is -kp * p (plus the first integral sample).
  const float expect = -(kDefaultRollKp * 30.0F + kDefaultRollKi * 30.0F * kTickS);
  TEST_ASSERT_FLOAT_WITHIN(1e-3F, expect, out.cmd_deg[0]);
  TEST_ASSERT_TRUE(out.defl_deg[0] < 0.0F);
  for (int i = 0; i < 120; ++i) out = ctl.Update(in);  // past the 1 s timeout
  TEST_ASSERT_EQUAL_FLOAT(0.0F, out.defl_deg[0]);
  TEST_ASSERT_FALSE(ctl.StartTest(TestCommand{}));  // kind kNone is refused
}

void Control_StartTest_RejectsBadCommands() {
  CanardControl ctl;
  ctl.Configure(SimParams(Layout::kRoll2));
  PadUpright(ctl, 0.1);
  const float inf = std::numeric_limits<float>::infinity();
  const float nan = std::nanf("");
  using Kind = TestCommand::Kind;
  // Duration must be finite and > 0; a live test's forced speed must be finite.
  for (const float duration : {nan, inf, -inf, 0.0F, -1.0F}) {
    TEST_ASSERT_FALSE(ctl.StartTest(TestCommand{Kind::kSweep, 0.0F, duration}));
    TEST_ASSERT_FALSE(ctl.StartTest(TestCommand{Kind::kLive, 100.0F, duration}));
  }
  for (const float v : {nan, inf, -inf}) {
    TEST_ASSERT_FALSE(ctl.StartTest(TestCommand{Kind::kLive, v, 5.0F}));
  }
  // Nothing was started: the canards stay at trim.
  ControlInputs in;
  const double acc[3] = {0.0, 0.0, kGravity};
  BodyToBoard(acc, in.acc_board_mps2);
  const double body[3] = {0.0, 0.0, 30.0};
  BodyToBoard(body, in.gyro_board_dps);
  in.long_axis = kLongAxis;
  in.long_sign = kLongSign;
  in.phase = Phase::kReady;
  in.dt_s = kTickS;
  const ControlOutputs out = ctl.Update(in);
  for (const float d : out.defl_deg) TEST_ASSERT_EQUAL_FLOAT(0.0F, d);
  // A valid command is still accepted afterwards.
  TEST_ASSERT_TRUE(ctl.StartTest(TestCommand{Kind::kLive, 100.0F, 5.0F}));
}

void Control_InvertedMount_LatchesImuFaultAtActivation() {
  // Ruling R24: long_sign wrong (the board reads -1 g on body z at rest). The pad stays NEUTRAL_PAD; activation
  // latches IMU_FAULT instead of flying an upside-down frame.
  const double up[3] = {0.0, 0.0, kGravity};
  const ControlOutputs out = PadThenActivate(up, -kLongSign, 3.0);
  TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kNeutralLatched), static_cast<int>(out.state));
  TEST_ASSERT_EQUAL(static_cast<int>(LatchReason::kImuFault), static_cast<int>(out.reason));
  for (const float d : out.defl_deg) TEST_ASSERT_EQUAL_FLOAT(0.0F, d);
}

void Control_TiltedRail10_ActivatesNormally() {
  // A 10 deg rail: body z specific force g cos 10 = 0.98 g, above the 0.8 g frame check.
  const double a = 10.0 * kDegToRad;
  const double acc[3] = {kGravity * std::sin(a), 0.0, kGravity * std::cos(a)};
  const ControlOutputs out = PadThenActivate(acc, kLongSign, kPadS);
  TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kActive), static_cast<int>(out.state));
  TEST_ASSERT_EQUAL(static_cast<int>(LatchReason::kNone), static_cast<int>(out.reason));
  TEST_ASSERT_FLOAT_WITHIN(0.3F, -10.0F, out.tilt_deg[0]);
}

void Control_NoPadSamples_LatchesImuFaultAtActivation() {
  // Fail safe: without pad samples the frame was never checked, so activation latches IMU_FAULT.
  CanardControl ctl;
  ctl.Configure(SimParams(Layout::kRoll2));
  ControlInputs in;
  in.long_axis = kLongAxis;
  in.long_sign = kLongSign;
  in.phase = Phase::kThrusting;
  in.velocity_mps = 50.0F;
  in.dt_s = kTickS;
  const ControlOutputs out = ctl.Update(in);
  TEST_ASSERT_EQUAL(static_cast<int>(EnvState::kNeutralLatched), static_cast<int>(out.state));
  TEST_ASSERT_EQUAL(static_cast<int>(LatchReason::kImuFault), static_cast<int>(out.reason));
}

// ---------------------------------------------------------------- closed loop

void Sim_Roll_DampsAt100() { CheckDampsAt100(1.0, "Sim_Roll_DampsAt100"); }

void Sim_Roll_StableAcrossEffectiveness() {
  for (const double eff : {1.0, 0.5, 0.3}) {
    for (const double v : {60.0, 100.0, 138.0}) {
      Case c = RollCase(v, 360.0);
      c.eff = eff;
      c.duration_s = 5.0;
      const Trace tr = Run(c);
      const double ts = SettleTime(tr.t, tr.p_dps, 20.0);
      std::printf("SIM Sim_Roll_StableAcrossEffectiveness eff %.1f v %.0f: settle %.3f s, last-1s peak %.2f deg/s\n",
                  eff, v, ts, MaxAbsFrom(tr.t, tr.p_dps, 4.0));
      TEST_ASSERT_TRUE(ts <= 2.0);
      AssertNoGrowth(tr, tr.p_dps, c.p0_dps, 0.5);
      AssertResidualRoll(tr, "Sim_Roll_StableAcrossEffectiveness");
      AssertStillActive(tr);
    }
  }
}

void Sim_Roll_ReversedTripsAuthority() {
  // The saturated reversed loop settles at 0.5 * Kc * 8 deg * V / D = 1.14 deg/s per m/s (114 deg/s at 100 m/s),
  // ~20 % above the default authority_rate_dps of 90 (see Sim_Roll_Reversed_180DoesNotTrip for why not 180).
  Case c = RollCase(100.0, 200.0);
  c.eff = -0.5;
  c.disturbance_eq_deg = 0.0;
  c.duration_s = 3.0;
  const Trace tr = Run(c);
  size_t latch = tr.out.size();
  for (size_t i = 0; i < tr.out.size(); ++i) {
    if (tr.out[i].state == EnvState::kNeutralLatched) {
      latch = i;
      break;
    }
  }
  TEST_ASSERT_TRUE(latch < tr.out.size());
  Report("Sim_Roll_ReversedTripsAuthority", tr.out_t[latch], "s to latch (criterion <= 1.5 s)");
  TEST_ASSERT_TRUE(tr.out_t[latch] <= 1.5);
  TEST_ASSERT_EQUAL(static_cast<int>(LatchReason::kAuthority), static_cast<int>(tr.out[latch].reason));
  // Outputs slew to trim at 300 deg/s (8 deg in 3 ticks) and stay there.
  for (size_t i = latch + 3; i < tr.out.size(); ++i) {
    TEST_ASSERT_EQUAL(static_cast<int>(LatchReason::kAuthority), static_cast<int>(tr.out[i].reason));
    for (const float d : tr.out[i].defl_deg) TEST_ASSERT_EQUAL_FLOAT(0.0F, d);
  }
}

void Sim_Roll_Reversed_180DoesNotTrip() {
  // Why the default authority_rate_dps is 90: at the former 180 deg/s the authority check cannot see a -0.5
  // reversal on this vehicle (steady 114 deg/s at 100 m/s, 157 deg/s at 138 m/s).
  for (const double v : {100.0, 138.0}) {
    Case c = RollCase(v, 200.0);
    c.params.authority_rate_dps = 180.0F;
    c.eff = -0.5;
    c.disturbance_eq_deg = 0.0;
    c.duration_s = 3.0;
    const Trace tr = Run(c);
    const double steady = MaxAbsFrom(tr.t, tr.p_dps, 2.0);
    std::printf("SIM Sim_Roll_Reversed_180DoesNotTrip v %.0f: steady %.1f deg/s\n", v, steady);
    TEST_ASSERT_TRUE(steady > 90.0 && steady < 180.0);
    AssertStillActive(tr);
  }
}

void Sim_Full4_TiltAt100() {
  Case c;
  c.params = SimParams(Layout::kFull4);
  c.v_mps = 100.0;
  c.tilt0_deg = 5.0;
  c.disturbance_eq_deg = vehicle::kDisturbanceEqDeg;
  c.duration_s = 4.0;
  const Trace tr = Run(c);
  // The estimator was levelled on the pad: it must agree with the plant before the criteria mean anything.
  TEST_ASSERT_TRUE(std::fabs(tr.true_tilt_at_liftoff_deg[0] - tr.est_tilt_at_liftoff_deg[0]) < 0.05);
  TEST_ASSERT_TRUE(std::fabs(tr.true_tilt_at_liftoff_deg[1] - tr.est_tilt_at_liftoff_deg[1]) < 0.05);
  const double ts = SettleTime(tr.t, tr.tilt_deg, 2.0);
  Report("Sim_Full4_TiltAt100", ts, "s to tilt < 2 deg (criterion <= 2 s)");
  TEST_ASSERT_TRUE(ts <= 2.0);
  Report("Sim_Full4_TiltAt100 tilt command", MaxTiltCommandDeg(tr), "deg peak (criterion > 0.1)");
  TEST_ASSERT_TRUE(MaxTiltCommandDeg(tr) > 0.1);
  AssertStillActive(tr);
}

void Sim_Full4_TiltAt70_NoGrowth() {
  Case c;
  c.params = SimParams(Layout::kFull4);
  c.v_mps = 70.0;
  c.tilt0_deg = 5.0;
  c.disturbance_eq_deg = vehicle::kDisturbanceEqDeg;
  c.duration_s = 5.0;
  const Trace tr = Run(c);
  const double peak = MaxAbsFrom(tr.t, tr.tilt_deg, 0.0);
  Report("Sim_Full4_TiltAt70_NoGrowth", peak, "deg peak tilt (criterion <= 5.1)");
  TEST_ASSERT_TRUE(peak <= 5.1);
  Report("Sim_Full4_TiltAt70_NoGrowth tilt command", MaxTiltCommandDeg(tr), "deg peak (criterion > 0.1)");
  TEST_ASSERT_TRUE(MaxTiltCommandDeg(tr) > 0.1);
  AssertStillActive(tr);
}

void Sim_StuckOutput_Stable() {
  // Bounded roll: half the 360 deg/s STEP roll abort. AssertStillActive also shows no authority latch at 90 deg/s.
  constexpr double kBoundedRollDps = 180.0;
  for (const Layout layout : {Layout::kFull4, Layout::kRoll2}) {
    Case c;
    c.params = SimParams(layout);
    c.v_mps = 138.0;
    c.stuck_output = 0;
    c.stuck_deg = 8.0;
    c.disturbance_eq_deg = vehicle::kDisturbanceEqDeg;
    c.duration_s = 5.0;
    const Trace tr = Run(c);
    const double tilt = MaxAbsFrom(tr.t, tr.tilt_deg, 0.0);
    const double roll = MaxAbsFrom(tr.t, tr.p_dps, 0.0);
    std::printf("SIM Sim_StuckOutput_Stable %s: peak tilt %.2f deg, peak roll %.1f deg/s, last-1s roll %.1f\n",
                layout == Layout::kFull4 ? "FULL_4" : "ROLL_2", tilt, roll, MaxAbsFrom(tr.t, tr.p_dps, 4.0));
    TEST_ASSERT_TRUE(tilt < c.params.abort_tilt_deg);
    TEST_ASSERT_TRUE(roll < kBoundedRollDps);
    // Roll starts at 0 and is driven by the stuck output, so only late growth is checked; the peak is bounded above.
    AssertNoLateGrowth(tr, tr.p_dps, 0.5);
    AssertStillActive(tr);
  }
}

void Sim_Roll_LowSpeed() {
  Case c = RollCase(25.0, 120.0);
  c.i_roll = vehicle::kIrollH;
  c.i_pitch = vehicle::kIpitchH;
  c.cg_m = vehicle::kCgHM;
  c.params.v_enable = 15.0F;
  c.duration_s = 6.0;
  Trace tr = Run(c);
  const double ts = SettleTime(tr.t, tr.p_dps, 20.0);
  Report("Sim_Roll_LowSpeed 25 m/s", ts, "s to |p| < 20 deg/s (criterion <= 3 s)");
  TEST_ASSERT_TRUE(ts <= 3.0);
  AssertNoGrowth(tr, tr.p_dps, c.p0_dps, 0.5);
  AssertResidualRoll(tr, "Sim_Roll_LowSpeed 25 m/s");
  AssertStillActive(tr);

  c.v_mps = 15.0;
  c.duration_s = 10.0;
  tr = Run(c);
  const double tail = MaxAbsFrom(tr.t, tr.p_dps, 6.0);
  Report("Sim_Roll_LowSpeed 15 m/s", tail, "deg/s peak over 6-10 s (criterion <= 5)");
  TEST_ASSERT_TRUE(tail <= 5.0);
  AssertNoGrowth(tr, tr.p_dps, c.p0_dps, 0.5);
  AssertStillActive(tr);
}

void Sim_Roll_GainMargin() {
  // Gain-margin regression for the roll gains: with the canard roll effectiveness x3, a 1 deg/s probe (small
  // enough to stay unsaturated, max |u| ~2.4 deg at 15 m/s) must decay over the speed and inertia grid,
  // including the 10 ms delay and the servo. H-class mass properties below 30 m/s.
  for (const double v : {15.0, 30.0, 60.0, 100.0, 138.0}) {
    for (const double inertia : {1.0, 0.85}) {
      Case c = RollCase(v, 1.0);
      c.disturbance_eq_deg = 0.0;
      c.eff = 3.0;
      c.inertia_scale = inertia;
      if (v <= 30.0) {
        c.i_roll = vehicle::kIrollH;
        c.i_pitch = vehicle::kIpitchH;
        c.cg_m = vehicle::kCgHM;
      }
      c.duration_s = 4.0;
      const Trace tr = Run(c);
      const double first = MaxAbsFrom(tr.t, tr.p_dps, 0.2, 1.2);
      const double last = MaxAbsFrom(tr.t, tr.p_dps, 3.0);
      std::printf("SIM Sim_Roll_GainMargin v %.0f I x%.2f: peak 0.2-1.2 s %.4f, 3-4 s %.6f deg/s\n", v, inertia, first,
                  last);
      TEST_ASSERT_TRUE(last < 0.1 * first);
      AssertStillActive(tr);
    }
  }
}

void Sim_MassSweep() { CheckDampsAt100(0.85, "Sim_MassSweep (inertia -15 %)"); }

int main() {
  UNITY_BEGIN();
  RUN_TEST(Vehicle_CanardsAloneStop360In0p12s);
  RUN_TEST(Defaults_TiltGainsZero_UntilStage2Sim);
  RUN_TEST(Control_Shadow_LogsWouldBeCommand_SendsZero);
  RUN_TEST(Control_Closed_SlewLimitedAtDefaultWhenUnset);
  RUN_TEST(Control_NonFinite_ZerosAndLatchesImuFault);
  RUN_TEST(Control_Latched_LogsWouldBeCommand_SendsZero);
  RUN_TEST(Control_Off_NoOutputs);
  RUN_TEST(Control_Step_MixesChosenAxis);
  RUN_TEST(Control_TestSweep_StepsEachOutput);
  RUN_TEST(Control_TestLive_UsesForcedVelocity);
  RUN_TEST(Control_StartTest_RejectsBadCommands);
  RUN_TEST(Control_InvertedMount_LatchesImuFaultAtActivation);
  RUN_TEST(Control_TiltedRail10_ActivatesNormally);
  RUN_TEST(Control_NoPadSamples_LatchesImuFaultAtActivation);
  RUN_TEST(Sim_Roll_DampsAt100);
  RUN_TEST(Sim_Roll_StableAcrossEffectiveness);
  RUN_TEST(Sim_Roll_ReversedTripsAuthority);
  RUN_TEST(Sim_Roll_Reversed_180DoesNotTrip);
  RUN_TEST(Sim_Full4_TiltAt100);
  RUN_TEST(Sim_Full4_TiltAt70_NoGrowth);
  RUN_TEST(Sim_StuckOutput_Stable);
  RUN_TEST(Sim_Roll_LowSpeed);
  RUN_TEST(Sim_Roll_GainMargin);
  RUN_TEST(Sim_MassSweep);
  return UNITY_END();
}
