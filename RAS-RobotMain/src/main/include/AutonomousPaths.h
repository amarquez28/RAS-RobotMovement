// ============================================================================
// AutonomousPaths.h – RAS 2026 | IEEE Region 5 Robotics Competition
// ============================================================================
//
// PURPOSE
// -------
// Central repository for all static autonomous paths.
// Each path is a sequence of {x_m, y_m, theta_rad} waypoints consumed by
// the PID controller in Robot.cpp.
//
// COORDINATE SYSTEM (field-relative, reset to zero at autonomous init)
// --------------------------------------------------------------------
// x : forward / backward axis. Positive = intake-forward direction.
// y : lateral axis. Positive = strafe RIGHT, negative = LEFT.
// theta : heading in radians. 0 = forward, π = reversed, positive CCW.
//
// ENCODER → METRE CONVERSION (from SweepController.h / BOM)
// Vertical (0x80 M1/M2) : 3323 ticks / m (751.8 PPR, 36.9:1, 72 mm wheel)
// Horizontal (0x81 M1)  : 1700 ticks / m (384.5 PPR, 13.7:1, 72 mm wheel)
//
// HOW TO ADD A NEW PATH / HOW PATHS ARE SELECTED
// -----------------------------------------------
// See GetPath() at the bottom of this file.
//
// ============================================================================
#pragma once
#include <vector>
#include <numbers>  // std::numbers::pi  (C++20)

// ── Waypoint structure ────────────────────────────────────────────────────────
struct Setpoint {
    double x_trgt;         // Target x position (metres, forward positive)
    double y_trgt;         // Target y position (metres, right positive)
    double theta_rad_trgt; // Target heading (radians, CCW positive, 0 = fwd)
};

// ── Named constants ───────────────────────────────────────────────────────────
namespace PathConst {
    inline constexpr double pi     = std::numbers::pi;
    inline constexpr double pi_2   = std::numbers::pi / 2.0;
    inline constexpr double pi_4   = std::numbers::pi / 4.0;
    inline constexpr double two_pi = 2.0 * std::numbers::pi;

    inline constexpr double FIELD_LENGTH_M = 2.4384;  // 8 ft
    inline constexpr double FIELD_WIDTH_M  = 1.2192;  // 4 ft
}

namespace AutonomousPaths {

// ============================================================================
//  Path Default  –  DEFAULT  |  FULL COMPETITION PATH  (RAS 2026)
//
//  PRE-CONDITION
//  -------------
//  AutonomousInit() starts in CENTERING phase. The robot strafes until the
//  AprilTag is centered in the camera frame, then resets all encoders and IMU
//  to (0, 0, 0) before loading this path. All waypoints are relative to that
//  centered and reset origin.
//
//  SEQUENCE
//  --------
//   [0]        Start at centered + reset origin.
//   [1]        Reverse to cave wall — beacon deposits (arm already lowered).
//              Robot dwells kServoDwell_s, then ArmRaise() is called.
//   [2]        Return to sweep start line.
//   [3]–[9]    4-row decreasing-length sweep (spiral). Each row is ~10 cm
//              shorter at each end than the previous.
//              Strafe shifts are 30 cm between rows (minimal strafe use).
//   [9]        End of sweep — ore deposit (actuator extend/retract).
//   [10]       Transit to cave AprilTag zone → TAG_SEARCH handoff.
//              Robot reads tag ID here to select cave entry path (Path_1+).
//
//  TUNING NOTES
//  ------------
//  - x positive = away from cave (into field). Negative = toward cave.
//  - y negative = strafe right (toward field boundary on camera side).
//  - All distances are estimates. Measure the actual field and adjust.
//  - kTagHandoffWaypoint must equal 10 in Robot.h.
// ============================================================================
static std::vector<Setpoint> Path_Default() {
    using namespace PathConst;
    return {

        // ── Start (position locked by CENTERING phase) ────────────────────────
        {  0.00,  0.00,  0.0 },  // [0] origin — encoder/IMU reset by CENTERING

        // ── Beacon deposit ────────────────────────────────────────────────────
        //    Arm is already lowered from AutonomousInit. Robot reverses to the
        //    cave wall so the beacon rests on the ground. The waypoint completion
        //    block dwells kServoDwell_s then calls ArmRaise() before advancing.
        //    TODO: measure the distance to the cave wall and adjust -0.30.
        { -0.30,  0.00,  0.0 },  // [1] reverse 30 cm to cave wall (beacon drop)

        // ── Return to sweep start ─────────────────────────────────────────────
        {  0.00,  0.00,  0.0 },  // [2] drive back to centered origin

        // ── Spiral boustrophedon sweep ────────────────────────────────────────
        //    4 rows with decreasing length (10 cm shorter at each far-end turn).
        //    Only the 30 cm lateral shifts use the strafe motor — all long legs
        //    use the forward/backward drive motors only.
        //
        //    TODO: Adjust x_far (1.80 m) to match your actual field length.
        //    TODO: Adjust strip width (0.30 m) to match your robot brush width.
        {  1.80,  0.00,  0.0 },  // [3] row 1 forward  — full length
        {  1.80, -0.30,  0.0 },  // [4] shift right 30 cm
        {  0.10, -0.30,  0.0 },  // [5] row 2 backward — 10 cm shorter at far end
        {  0.10, -0.60,  0.0 },  // [6] shift right 30 cm
        {  1.70, -0.60,  0.0 },  // [7] row 3 forward  — 10 cm shorter at near end too
        {  1.70, -0.90,  0.0 },  // [8] shift right 30 cm

        // ── Ore deposit ───────────────────────────────────────────────────────
        //    Waypoint completion block triggers actuator extend → retract dwell.
        //    Robot holds this position while ores are dumped.
        //    TODO: Adjust x (0.20 m) so the actuator faces the collection goal.
        {  0.20, -0.90,  0.0 },  // [9] row 4 backward — ore deposit here

        // ── Transit to cave AprilTag zone → TAG_SEARCH handoff ───────────────
        //    Robot drives to the cave entrance and reads the AprilTag to select
        //    which cave sub-path to run (Path_1, Path_2, etc.).
        //    kTagHandoffWaypoint = 10 in Robot.h must match this index.
        //    TODO: Adjust x (2.20 m) and y (-0.45 m) to your cave entrance.
        {  2.20, -0.45,  0.0 },  // [10] transit to cave entrance — TAG_SEARCH handoff
    };
}


// ============================================================================
//  Path 1  –  TAG ID 1  |  CAVE ENTRY / SWEEP
//  Loaded after TAG_SEARCH reads tag ID 1 at the cave entrance.
//  All coordinates are relative to the encoder reset that happens in TAG_SEARCH.
//  Strategy notes:
//    - Enter cave forward, sweep inside, exit in reverse.
//    - Adjust all distances to your actual cave dimensions.
// ============================================================================
static std::vector<Setpoint> Path_1() {
    using namespace PathConst;
    return {
         // [0] Placeholder — robot holds cave entrance position
        {0.835, 0., 0},
        {0.79,0.2, pi_2},
        {0.79, 0.2, -pi_2}
    };
}


// ============================================================================
//  Path 2  –  TAG ID 2  |  PLACEHOLDER
// ============================================================================
static std::vector<Setpoint> Path_2() {
    using namespace PathConst;
    return {
        { 0.0, 0.0, 0.0 },  // [0] Placeholder
    };
}


// ============================================================================
//  Path 3  –  TAG ID 3  |  PLACEHOLDER
// ============================================================================
static std::vector<Setpoint> Path_3() {
    using namespace PathConst;
    return {
        { 0.0, 0.0, 0.0 },  // [0] Placeholder
    };
}


// ============================================================================
//  Path 4  –  TAG ID 4  |  PLACEHOLDER
// ============================================================================
static std::vector<Setpoint> Path_4() {
    using namespace PathConst;
    return {
        { 0.0, 0.0, 0.0 },  // [0] Placeholder
    };
}


// ============================================================================
//  GetPath  –  AprilTag ID dispatcher
// ============================================================================
inline std::vector<Setpoint> GetPath(int tagId) {
    switch (tagId) {
        case 1:  return Path_1();
        case 2:  return Path_2();
        case 3:  return Path_3();
        case 4:  return Path_4();
        case 0:
        default: return Path_Default();
    }
}

}  // namespace AutonomousPaths
