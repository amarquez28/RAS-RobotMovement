// ============================================================================
//  AutonomousPaths.h  –  RAS 2026  |  IEEE Region 5 Robotics Competition
// ============================================================================
//
//  PURPOSE
//  -------
//  Central repository for all static autonomous paths.
//  Each path is a sequence of {x_m, y_m, theta_rad} waypoints consumed by
//  the PID controller in Robot.cpp.
//
//  COORDINATE SYSTEM (field-relative, reset to zero at autonomous init)
//  --------------------------------------------------------------------
//    x      : forward / backward axis.  Positive = intake-forward direction.
//    y      : lateral axis.             Positive = strafe RIGHT, negative = LEFT.
//    theta  : heading in radians.       0 = forward, π = reversed, positive CCW.
//
//  ENCODER → METRE CONVERSION (from SweepController.h / BOM)
//    Vertical  (0x80 M1/M2) : 3323 ticks / m   (751.8 PPR, 36.9:1, 72 mm wheel)
//    Horizontal (0x81 M1)   : 1700 ticks / m   (384.5 PPR, 13.7:1, 72 mm wheel)
//
//  HOW TO ADD A NEW PATH
//  ---------------------
//  1. Write a new static function below following the PATH TEMPLATE.
//  2. Register it inside GetPath() with the corresponding AprilTag ID.
//  3. Update the header comment in GetPath() to reflect the new entry.
//
//  HOW PATHS ARE SELECTED
//  ----------------------
//  The robot reads an AprilTag ID at the start of autonomous.
//  AutonomousInit() calls:
//
//      m_setpoints = AutonomousPaths::GetPath(tagId);
//
//  If no tag is seen, pass tagId = 0 to get the default/fallback path.
//
//  PHASE 1 SCOPE (IMU + PID only, no live AprilTag feedback)
//  -----------------------------------------------------------
//  All paths below use dead-reckoning (encoder odometry + IMU heading).
//  AprilTag integration will be layered on top in a later sprint.
//
// ============================================================================



#pragma once

#include <vector>
#include <numbers>    // std::numbers::pi  (C++20)

// ── Waypoint structure ───────────────────────────────────────────────────────
// Mirrors the Setpoint struct in Robot.h.  Both files must stay in sync.
// If you change this definition, update Robot.h's Setpoint struct to match.
//
struct Setpoint {
    double x_trgt;         // Target x position (metres, forward positive)
    double y_trgt;         // Target y position (metres, right positive)
    double theta_rad_trgt; // Target heading    (radians, CCW positive, 0 = fwd)
};

// ── Named constants used across all paths ────────────────────────────────────
namespace PathConst {
    inline constexpr double pi    = std::numbers::pi;
    inline constexpr double pi_2  = std::numbers::pi / 2.0;
    inline constexpr double pi_4  = std::numbers::pi / 4.0;
    inline constexpr double two_pi = 2.0 * std::numbers::pi;

    // ── Field geometry ────────────────────────────────────────────────────────
    inline constexpr double FIELD_LENGTH_M = 2.4384;  // 8 ft
    inline constexpr double FIELD_WIDTH_M  = 1.2192;  // 4 ft
}

// ============================================================================
//  AutonomousPaths namespace
//  All functions are static; no instantiation needed.
// ============================================================================
namespace AutonomousPaths {

// ── PATH TEMPLATE ─────────────────────────────────────────────────────────────
//
//  Copy this block to create a new path.
//  Replace PATH_N and tagId=N in GetPath() below.
//
//  static std::vector<Setpoint> Path_N() {
//      using namespace PathConst;
//      return {
//          // ── Phase 1: Description ─────────────────────────────────────────
//          { x_m,  y_m,  heading_rad },  // [0] What the robot does here
//          { x_m,  y_m,  heading_rad },  // [1] ...
//      };
//  }
//
// ─────────────────────────────────────────────────────────────────────────────


// ============================================================================
//  Path 0  –  DEFAULT / FALLBACK
//  Used when no AprilTag is detected at autonomous start.
//  Safe minimal motion: drive forward to mid-field, stop.
//  Tune this first to verify basic PID → encoder dead-reckoning behaviour.
// ============================================================================
static std::vector<Setpoint> Path_Default() {
    using namespace PathConst;
    return {
        // ── Phase 1: Advance to mid-field ────────────────────────────────────
        { 0.60,  0.0,  0.0 },   // [0] Drive forward 60 cm (≈ half field)

        // ── Phase 2: Hold position ────────────────────────────────────────────
        // Add further waypoints here once basic motion is verified.
    };
}


// ============================================================================
//  Path 1  –  TAG ID 1  |  FULL COMPETITION PATH  (original strategy)
//
//  Sequence overview:
//    1.  Drive forward 38 cm.
//    2–5. 360° spin in place (four heading waypoints).
//    6.  Drive forward 63 cm (total x = 1.01 m).
//    7.  Reverse 30 cm (x = 0.71 m).
//    8.  180° in-place turn (theta = π).
//    9.  Reverse 30 cm; DROP BEACON (x = 0.41 m).
//    10–25. Boustrophedon sweep of field in ≈14 cm strips.
//    26. Strafe left 83 cm; DEPOSIT payload (x = 0.52, y = −0.595).
//    27. Strafe right 44 cm (recovery).
//    28. Drive forward 180 cm — EXIT ZONE.
//
//  NOTE on waypoints [1]→[4]: The smooth spin through 0 → π/2 → π → −π/2
//  then back to π/2 avoids the WrapAngle discontinuity at ±π.  If the robot
//  oscillates badly at [4]→[5], insert an extra {0.38, 0.0, 0.0} waypoint
//  to guide the PID back through 0 explicitly.
// ============================================================================
static std::vector<Setpoint> Path_1() {
    using namespace PathConst;
    return {
        // ── Phase 1: Initial advance ──────────────────────────────────────────
        {0.38, 0.0, 0.0}, // [0]  Drive forward 38 cm

        // ── Phase 2: Smooth 360° spin in place ───────────────────────────────
        {0.38, 0.0, pi_2},  // [1]  Rotate 90° CCW  (face left)
        {0.38, 0.0, pi},    // [2]  Rotate 90° more (face rearward)
        {0.38, 0.0, -pi_2}, // [3]  Rotate 90° more (face right / 270°)
        {0.38, 0.0, pi_2},  // [4]  Complete 360°   (back to left-facing)

        // ── Phase 3: Forward run, reverse, turn, drop beacon ─────────────────
        {1.01, 0.0, 0.0}, // [5]  Drive forward 63 cm (total 1.01 m)
        {0.71, 0.0, 0.0}, // [6]  Reverse 30 cm
        {0.71, 0.0, pi},  // [7]  180° turn (face rearward)
        {0.41, 0.0, pi},  // [8]  Reverse 30 cm  ← DROP BEACON

        // ── Phase 4: Boustrophedon sweep (3-cm strips, rearward-facing) ───────
        {0.41, -0.415, pi}, // [9]  Strafe left 41.5 cm (start col 1)
        {0.64, -0.415, pi}, // [10] Forward  23 cm
        {0.41, -0.415, pi}, // [11] Reverse  23 cm
        {0.41, -0.245, pi}, // [12] Strafe right 17 cm  (col 2)
        {1.03, -0.245, pi}, // [13] Forward  62 cm
        {1.03, -0.435, pi}, // [14] Strafe left  19 cm  (col 3)
        {1.35, -0.435, pi}, // [15] Forward  32 cm
        {0.42, -0.435, pi}, // [16] Reverse  93 cm
        {0.42, -0.245, pi}, // [17] Strafe right 19 cm  (col 4)
        {1.36, -0.245, pi}, // [18] Forward  94 cm
        {0.42, -0.245, pi}, // [19] Reverse  94 cm
        {0.42, 0.005, pi},  // [20] Strafe right 25 cm  (col 5)
        {1.35, 0.005, pi},  // [21] Forward  93 cm
        {0.42, 0.005, pi},  // [22] Reverse  93 cm
        {0.42, 0.235, pi},  // [23] Strafe right 23 cm  (col 6)
        {1.20, 0.235, pi},  // [24] Forward  78 cm
        {0.52, 0.235, pi},  // [25] Reverse  68 cm

        // ── Phase 5: Deposit payload, then exit zone ──────────────────────────
        {0.52, -0.595, pi}, // [26] Strafe left 83 cm  ← DEPOSIT
        {0.52, -0.155, pi}, // [27] Strafe right 44 cm (clear deposit zone)
        {2.32, -0.155, pi}, // [28] Drive 180 cm forward — EXIT ZONE

        // NOTE: Waypoint 5→6 transitions heading from π/2 back to 0 without an
        // explicit intermediate. The PID handles it but watch for oscillation;
        // add a dedicated spin waypoint here if the robot spins erratically.
        //
        // M_PI replaced with std::numbers::pi for C++20 / WPILib toolchain safety.
        // ─────────────────────────────────────────────────────────────────────────────
    };
}


// ============================================================================
//  Path 2  –  TAG ID 2  |  PLACEHOLDER
//  Replace this stub with your actual Path 2 waypoints.
//  Strategy notes:
//    - ...
// ============================================================================
static std::vector<Setpoint> Path_2() {
    using namespace PathConst;
    return {
        // ── TODO: Define Path 2 waypoints ────────────────────────────────────
        { 0.0,  0.0,  0.0 },   // [0] Placeholder — robot holds start position
    };
}


// ============================================================================
//  Path 3  –  TAG ID 3  |  PLACEHOLDER
//  Replace this stub with your actual Path 3 waypoints.
// ============================================================================
static std::vector<Setpoint> Path_3() {
    using namespace PathConst;
    return {
        // ── TODO: Define Path 3 waypoints ────────────────────────────────────
        { 0.0,  0.0,  0.0 },   // [0] Placeholder — robot holds start position
    };
}


// ============================================================================
//  Path 4  –  TAG ID 4  |  PLACEHOLDER
//  Replace this stub with your actual Path 4 waypoints.
// ============================================================================
static std::vector<Setpoint> Path_4() {
    using namespace PathConst;
    return {
        // ── TODO: Define Path 4 waypoints ────────────────────────────────────
        { 0.0,  0.0,  0.0 },   // [0] Placeholder — robot holds start position
    };
}


// ============================================================================
//  GetPath  –  AprilTag ID dispatcher
// ============================================================================
//
//  Called by Robot.cpp's LoadAutonomousSetpoints():
//
//      m_setpoints = AutonomousPaths::GetPath(tagId);
//
//  Tag ID → Path mapping:
//    0      : Default / no tag detected → Path_Default()
//    1      : Full competition path     → Path_1()
//    2      : Path 2 (TBD)             → Path_2()
//    3      : Path 3 (TBD)             → Path_3()
//    4      : Path 4 (TBD)             → Path_4()
//    other  : Falls back to Path_Default()
//
inline std::vector<Setpoint> GetPath(int tagId) {
    switch (tagId) {
        case 1:  return Path_1();
        case 2:  return Path_2();
        case 3:  return Path_3();
        case 4:  return Path_4();

        // ── Add new cases above this line ─────────────────────────────────
        // case 5: return Path_5();

        case 0:
        default:
            return Path_Default();
    }
}

} // namespace AutonomousPaths
