/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// Rigid-body constants of the camera-nose Peregrine for the closed-loop canard simulation.
///
/// Sources
/// - OpenRocket model `openrocket/PeregrineL2-CamNose.ork` in ~/MyLevel1Peregrine, branch
///   feature/camnose-canards-ork. Mass properties and aerodynamics were computed headless with the OpenRocket
///   24.12 core API (OpenRocketCore.initialize(), GeneralRocketLoader, MassCalculator.calculateLaunch /
///   calculateBurnout, BarrowmanCalculator.getForceAnalysis at AoA 1 deg). The J350W-OLD-14 configuration
///   (simulation "Langtora Airfield 21 Feb 2026") gives identical numbers at commit f05c608 and at 899afa6;
///   the H115DM-14 configuration exists only from 266761e/9cd57dc on and was taken at 899afa6.
/// - Canard roll forcing and roll damping: Rocket-Parts `tools/canard_estimate.py` at commit 59cd4d5 (canard
///   geometry from PeregrineCamNose.scad: 4 panels, root 45, tip 25, span 25 mm, 875 mm^2 each, panel
///   CL_alpha 2.0645 /rad, roll arm 62.67 mm from the axis; 3 freeform fins, CL_alpha 2.0).
///
/// Model assumptions (stated, not hidden)
/// - Roll: 1-DOF, I_roll * p' = eff * q * kCanardRollNmPerDegPa * mean(delta) - (q / V) * kRollDampM4 * p.
///   The canard-fin interaction is the single factor eff on the canard term.
/// - Pitch/yaw: 2-DOF rotation at constant V with the velocity vector held vertical, so tilt equals angle of
///   attack and the fins restore it unaided. The FULL_4 cases therefore test the damping of the weathercock
///   mode and the loop's stability, not the authority to turn the flight path back to vertical (estimated
///   separately in the task report: about 1.6-2.2 deg/s of path turning at full deflection, 70-100 m/s).
namespace vehicle {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRho = 1.225;  // ISA sea level, as the firmware's schedule at 0 m AGL

// ---- OpenRocket, J350W-OLD-14 (the 21 Feb 2026 L2 flight conditions) ----
constexpr double kMassLiftoffKg = 4.31868;   // calculateLaunch
constexpr double kIrollLiftoff = 0.0072329;  // kg m^2, rotational MOI at launch
constexpr double kIpitchLiftoff = 0.995047;  // kg m^2, longitudinal MOI at launch
constexpr double kCgLiftoffM = 0.97909;      // from the nose tip
// Burnout (coast) values, for reference: m 3.94281 kg, I_roll 0.0071650, I_pitch 0.886840, CG 0.93104 m.

// ---- OpenRocket, HP-H115DM-14 (H-class, 30.5 m/s peak) ----
constexpr double kMassHKg = 3.87274;
constexpr double kIrollH = 0.0071369;
constexpr double kIpitchH = 0.879491;
constexpr double kCgHM = 0.92459;

// ---- OpenRocket Barrowman aerodynamics, Mach 0.294 (100 m/s) ----
constexpr double kRefAreaM2 = 0.008107;    // reference diameter 101.6 mm
constexpr double kCnAlphaTotal = 12.0797;  // /rad, whole vehicle including the canards
constexpr double kCpTotalM = 1.21082;      // from the nose tip
constexpr double kCnAlphaFins = 8.9818;    // /rad, aft fin set
constexpr double kCpFinsM = 1.52044;
constexpr double kCnAlphaCanards = 0.7294;  // /rad, the 4-canard set at body AoA
constexpr double kCpCanardsM = 0.43510;     // canard CP from the nose tip

// ---- Rocket-Parts canard_estimate.py ----
/// Roll moment of all four canards per degree of (equal) deflection per Pa of dynamic pressure:
/// 4 * S * CL_alpha * (pi/180) * y_c = 4 * 875e-6 * 2.0645 * 0.017453 * 0.062673.
constexpr double kCanardRollNmPerDegPa = 7.9038e-6;
/// Roll damping integral sum(CL_alpha * c * r^2 * dr), fins (1.55974e-3) plus canards (2.80e-5), in m^4;
/// damping moment = (q / V) * kRollDampM4 * p. OpenRocket's own roll damping for the model is ~3x larger
/// (0.300 N m s/rad at 101 m/s vs 0.098 here); the script value is used as the nominal (less damped) case.
constexpr double kRollDampM4 = 1.58777e-3;

// ---- Tilt (FULL_4) ----
/// Side force of one canard per radian of its deflection, in units of q * kRefAreaM2: half the 4-canard
/// CN_alpha (a deflected opposite pair acts like the set at body AoA), body interference included.
constexpr double kCnDeltaPerCanard = kCnAlphaCanards / 2.0;
/// Pitch damping sum(CN_alpha_i * (x_i - x_cg)^2) over fins and canards, in m^2 (body terms left out, so
/// the model is less damped than the airframe); moment = (q * A / V) * this * omega.
inline double PitchDampM2(double cg_m) {
  return kCnAlphaFins * (kCpFinsM - cg_m) * (kCpFinsM - cg_m) +
         kCnAlphaCanards * (cg_m - kCpCanardsM) * (cg_m - kCpCanardsM);
}
/// Restoring moment slope per radian of AoA, in units of q * kRefAreaM2 (N m per Pa m^2).
inline double RestoringM(double cg_m) { return kCnAlphaTotal * (kCpTotalM - cg_m); }
/// Canard moment arm about the CG.
inline double CanardArmM(double cg_m) { return cg_m - kCpCanardsM; }

/// Measured roll disturbance (spec: the Peregrine rolls 5-17 deg/s in flight). Modelled as a q-proportional
/// roll moment (like fin cant) whose uncontrolled steady roll is 17 deg/s at 100 m/s; expressed as an
/// equivalent all-canard deflection in degrees: p_ss = Kc * d * V / D  ->  d = p_ss * D / (Kc * V).
constexpr double kDisturbanceSteadyDps = 17.0;
constexpr double kDisturbanceAtMps = 100.0;
constexpr double kDisturbanceEqDeg =
    kDisturbanceSteadyDps * kPi / 180.0 * kRollDampM4 / (kCanardRollNmPerDegPa * kDisturbanceAtMps);

// ---- Servo: MKS HV6160, 0.23 s / 60 deg ----
constexpr double kServoRateDps = 60.0 / 0.23;

}  // namespace vehicle
