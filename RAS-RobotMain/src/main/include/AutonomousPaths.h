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
// Horizontal (0x81 M1) : 1700 ticks / m (384.5 PPR, 13.7:1, 72 mm wheel)
//
// HOW TO ADD A NEW PATH
// ---------------------
// 1. Write a new static function below following the PATH TEMPLATE.
// 2. Register it inside GetPath() with the corresponding AprilTag ID.
// 3. Update the header comment in GetPath() to reflect the new entry.
//
// HOW PATHS ARE SELECTED
// ----------------------
// The robot reads an AprilTag ID at the start of autonomous.
// AutonomousInit() calls:
//
//   m_setpoints = AutonomousPaths::GetPath(tagId);
//
// If no tag is seen, pass tagId = 0 to get the default/fallback path.
//
// PHASE 1 SCOPE (IMU + PID only, no live AprilTag feedback)
// -----------------------------------------------------------
// All paths below use dead-reckoning (encoder odometry + IMU heading).
// AprilTag integration will be layered on top in a later sprint.
//
// MIDPOINT INTERPOLATION RULE
// ----------------------------
// Each pair of consecutive waypoints is split into two equal steps UNLESS
// the x-distance between them is < 0.10 m OR the y-distance is < 0.05 m.
// The midpoint inherits the theta of the destination waypoint.
//
// ============================================================================
#pragma once
#include <vector>
#include <numbers>  // std::numbers::pi (C++20)

// ── Waypoint structure ─────────────────────────────────────────────────────────
// Mirrors the Setpoint struct in Robot.h. Both files must stay in sync.
// If you change this definition, update Robot.h's Setpoint struct to match.
//
struct Setpoint {
    double x_trgt;        // Target x position (metres, forward positive)
    double y_trgt;        // Target y position (metres, right positive)
    double theta_rad_trgt; // Target heading (radians, CCW positive, 0 = fwd)
};

// ── Named constants used across all paths ──────────────────────────────────────
namespace PathConst {
    inline constexpr double pi     = std::numbers::pi;
    inline constexpr double pi_2   = std::numbers::pi / 2.0;
    inline constexpr double pi_4   = std::numbers::pi / 4.0;
    inline constexpr double two_pi = 2.0 * std::numbers::pi;

    // ── Field geometry ──────────────────────────────────────────────────────────
    inline constexpr double FIELD_LENGTH_M = 2.4384;  // 8 ft
    inline constexpr double FIELD_WIDTH_M  = 1.2192;  // 4 ft
}

// ============================================================================
// AutonomousPaths namespace
// All functions are static; no instantiation needed.
// ============================================================================
namespace AutonomousPaths {

// ── PATH TEMPLATE ───────────────────────────────────────────────────────────────
//
// Copy this block to create a new path.
// Replace PATH_N and tagId=N in GetPath() below.
//
// static std::vector<Setpoint> Path_N() {
//     using namespace PathConst;
//     return {
//         // ── Phase 1: Description ───────────────────────────────────────────
//         { x_m, y_m, heading_rad },  // [0] What the robot does here
//         { x_m, y_m, heading_rad },  // [1] ...
//     };
// }
//
// ─────────────────────────────────────────────────────────────────────────────────

// ============================================================================
// Path Default – DEFAULT | FULL COMPETITION PATH (RAS 2026 field sweep)
//
// Midpoint interpolation applied: each move is split into 2 equal steps
// unless |dx| < 0.10 m OR |dy| < 0.05 m (those moves stay as single steps).
//
// All coordinates in metres. Theta expressed via PathConst (C++20 pi).
// ============================================================================
static std::vector<Setpoint> Path_Default() {
    using namespace PathConst;
    return {
        //Path #2
        {0.34, 0.0, 0.0},
        {0.68, 0.0, 0.0},
        {0.38, 0.0, 0.0},
        {0.38, 0.0, pi_2}
        // ── Start ─────────────────────────────────────────────────────────────────
        /*{ 0.00,  0.00,   0.0 },   // [0]  start
        { 0.00, -0.15,   0.0 },   // [1]  right 15 cm  (mid: [0]→[1] dy=0.30≥0.05)
        { 0.00, -0.30,   0.0 },   // [2]  right 15 cm  (second half)

        // ── Approach ──────────────────────────────────────────────────────────────
        { 0.27, -0.30,   0.0 },   // [3]  forward 27 cm (mid: dx=0.54≥0.10, dy<0.05 → skip split — dy=0 < 0.05, keep as 1 step)
        // NOTE: [2]→next has dy=0, so no split. Emitting as single step:
        { 0.54, -0.30,   0.0 },   // [4]  forward 54 cm (no split: dy=0 < 0.05) — restored to 1 step
        { 0.54, -0.30,  -pi_2 },  // [5]  turn right 90° (pure rotation, no split)
        { 0.905,-0.30,  -pi_2 },  // [6]  forward 36.5 cm (mid: dx=0.73≥0.10, dy=0 < 0.05 → no split)
        { 1.27, -0.30,  -pi_2 },  // [7]  forward 36.5 cm (no split: dy=0)
        { 1.105,-0.30,  -pi_2 },  // [8]  reverse 16.5 cm (mid: dx=0.33≥0.10, dy=0 < 0.05 → no split)
        { 0.94, -0.30,  -pi_2 },  // [9]  reverse 16.5 cm
        { 0.94, -0.30,  -pi_2 },  // [10] raise arm 4 s  (stationary)
        { 0.94, -0.30,  -pi   },  // [11] turn right 90° (pure rotation)
        { 0.94, -0.30,  -pi   },  // [12] lower arm 4 s  (stationary)

        // ── Lower field sweep ─────────────────────────────────────────────────────
        // [12]→[13]: dx=0, dy=0.39≥0.05 → split
        { 0.94, -0.105, -pi   },  // [13] left 19.5 cm
        { 0.94,  0.09,  -pi   },  // [14] left 19.5 cm
        // [14]→[15]: dx=0.17≥0.10, dy=0 < 0.05 → no split
        { 1.025, 0.09,  -pi   },  // [15] forward 8.5 cm
        { 1.11,  0.09,  -pi   },  // [16] forward 8.5 cm
        // [16]→[17]: dx=0.26≥0.10, dy=0 < 0.05 → no split
        { 0.98,  0.09,  -pi   },  // [17] reverse 13 cm
        { 0.85,  0.09,  -pi   },  // [18] reverse 13 cm
        // [18]→[19]: dx=0, dy=0.17≥0.05 → split (but dx=0 < 0.10 → no split)
        { 0.85, -0.08,  -pi   },  // [19] right 17 cm  (no split: dx=0 < 0.10)
        // [19]→[20]: dx=0.77≥0.10, dy=0 < 0.05 → no split
        { 1.235,-0.08,  -pi   },  // [20] forward 38.5 cm
        { 1.62, -0.08,  -pi   },  // [21] forward 38.5 cm
        // [21]→[22]: dx=0, dy=0.17≥0.05 → no split (dx=0 < 0.10)
        { 1.62,  0.09,  -pi   },  // [22] left 17 cm  (no split: dx=0 < 0.10)
        // [22]→[23]: dx=0.33≥0.10, dy=0 < 0.05 → no split
        { 1.785, 0.09,  -pi   },  // [23] forward 16.5 cm
        { 1.95,  0.09,  -pi   },  // [24] forward 16.5 cm
        // [24]→[25]: dx=0.92≥0.10, dy=0 < 0.05 → no split
        { 1.49,  0.09,  -pi   },  // [25] reverse 46 cm
        { 1.03,  0.09,  -pi   },  // [26] reverse 46 cm

        // ── Deposit (lower field) ──────────────────────────────────────────────────
        // [26]→[27]: dx=0, dy=0.24≥0.05 → no split (dx=0 < 0.10)
        { 1.03, -0.15,  -pi   },  // [27] right 24 cm  (no split: dx=0 < 0.10)

        // ── Upper field sweep ─────────────────────────────────────────────────────
        // [27]→[28]: dx=0.93≥0.10, dy=0 < 0.05 → no split
        { 1.495,-0.15,  -pi   },  // [28] forward 46.5 cm
        { 1.96, -0.15,  -pi   },  // [29] forward 46.5 cm
        // [29]→[30]: dx=1.10≥0.10, dy=0 < 0.05 → no split
        { 1.41, -0.15,  -pi   },  // [30] reverse 55 cm
        { 0.86, -0.15,  -pi   },  // [31] reverse 55 cm
        // [31]→[32]: dx=0, dy=0.24≥0.05 → no split (dx=0 < 0.10)
        { 0.86, -0.39,  -pi   },  // [32] right 24 cm  (no split: dx=0 < 0.10)
        // [32]→[33]: dx=1.07≥0.10, dy=0 < 0.05 → no split
        { 1.395,-0.39,  -pi   },  // [33] forward 53.5 cm
        { 1.93, -0.39,  -pi   },  // [34] forward 53.5 cm
        // [34]→[35]: dx=1.07≥0.10, dy=0 < 0.05 → no split
        { 1.395,-0.39,  -pi   },  // [35] reverse 53.5 cm
        { 0.86, -0.39,  -pi   },  // [36] reverse 53.5 cm
        // [36]→[37]: dx=0, dy=0.24≥0.05 → no split (dx=0 < 0.10)
        { 0.86, -0.63,  -pi   },  // [37] right 24 cm  (no split: dx=0 < 0.10)
        // [37]→[38]: dx=0.90≥0.10, dy=0 < 0.05 → no split
        { 1.31, -0.63,  -pi   },  // [38] forward 45 cm
        { 1.76, -0.63,  -pi   },  // [39] forward 45 cm
        // [39]→[40]: dx=0.90≥0.10, dy=0 < 0.05 → no split
        { 1.31, -0.63,  -pi   },  // [40] reverse 45 cm
        { 0.86, -0.63,  -pi   },  // [41] reverse 45 cm
        // [41]→[42]: dx=0, dy=0.09≥0.05 → no split (dx=0 < 0.10)
        { 0.86, -0.72,  -pi   },  // [42] right 9 cm  (no split: dx=0 < 0.10)
        // [42]→[43]: dx=0.90≥0.10, dy=0 < 0.05 → no split
        { 1.31, -0.72,  -pi   },  // [43] forward 45 cm
        { 1.76, -0.72,  -pi   },  // [44] forward 45 cm
        // [44]→[45]: dx=0.75≥0.10, dy=0 < 0.05 → no split
        { 1.385,-0.72,  -pi   },  // [45] reverse 37.5 cm
        { 1.01, -0.72,  -pi   },  // [46] reverse 37.5 cm

        // ── Deposit + transit to cave ──────────────────────────────────────────────
        // [46]→[47]: dx=0, dy=0.81≥0.05 → no split (dx=0 < 0.10)
        { 1.01,  0.09,  -pi   },  // [47] left 81 cm  (no split: dx=0 < 0.10)
        // [47]→[48]: dx=0, dy=0.40≥0.05 → no split (dx=0 < 0.10)
        { 1.01, -0.31,  -pi   },  // [48] right 40 cm  (no split: dx=0 < 0.10)
        // [48]→[49]: dx=1.88≥0.10, dy=0 < 0.05 → no split
        { 1.95, -0.31,  -pi   },  // [49] forward 94 cm
        { 2.89, -0.31,  -pi   },  // [50] forward 94 cm

        // ── Bucket grab sequence ───────────────────────────────────────────────────
        // [50]→[51]: dx=0.08 < 0.10 → no split
        { 2.81, -0.31,  -pi   },  // [51] reverse 8 cm  (no split: dx < 0.10)
        { 2.81, -0.31,  -pi_2 },  // [52] turn left 90°  (pure rotation)
        // [52]→[53]: dx=0.25≥0.10, dy=0 < 0.05 → no split
        { 2.685,-0.31,  -pi_2 },  // [53] reverse 12.5 cm
        { 2.56, -0.31,  -pi_2 },  // [54] reverse 12.5 cm
        // [54]→[55]: dx=0, dy=0.17≥0.05 → no split (dx=0 < 0.10)
        { 2.56, -0.48,  -pi_2 },  // [55] right 17 cm  (no split: dx=0 < 0.10)
        { 2.56, -0.48,  -pi_2 },  // [56] grab bucket  (stationary)
        // [56]→[57]: dx=0.15 ≥0.10, dy=0 < 0.05 → no split
        { 2.635,-0.48,  -pi_2 },  // [57] forward 7.5 cm
        { 2.71, -0.48,  -pi_2 },  // [58] forward 7.5 cm
        // [58]→[59]: dx=0.25≥0.10, dy=0 < 0.05 → no split
        { 2.835,-0.48,  -pi_2 },  // [59] forward 12.5 cm
        { 2.96, -0.48,  -pi_2 },  // [60] forward 12.5 cm
        // [60]→[61]: dx=0.25≥0.10, dy=0 < 0.05 → no split
        { 3.085,-0.48,  -pi_2 },  // [61] forward 12.5 cm
        { 3.21, -0.48,  -pi_2 },  // [62] forward 12.5 cm
        // [61]→[62]: dx=0.25≥0.10, dy=0 < 0.05 → no split
        { 3.335,-0.48,  -pi_2 },  // [63] forward 12.5 cm
        { 3.46, -0.48,  -pi_2 },  // [64] forward 12.5 cm

        // ── Cave april tag sweep row 1 ─────────────────────────────────────────────
        // [64]→[65]: dx/dy both significant → split
        { 2.97, -0.395, -pi   },  // [65] mid: move to cave entry (half of dx=0.98, dy=0.17)
        { 2.48, -0.31,  -pi   },  // [66] cave entry
        { 2.48, -0.31,  -3.0*pi/2.0 },  // [67] turn upward (pure rotation)
        // [67]→[68]: dx=0, dy=0.04 < 0.05 → no split
        { 2.48, -0.27,  -3.0*pi/2.0 },  // [68] shift left 4 cm  (no split: dy < 0.05)
        // [68]→[69]: dx=0.40≥0.10, dy=0 < 0.05 → no split
        { 2.68, -0.27,  -3.0*pi/2.0 },  // [69] forward 20 cm
        { 2.88, -0.27,  -3.0*pi/2.0 },  // [70] forward 20 cm
        // [70]→[71]: dx=0.40≥0.10, dy=0 < 0.05 → no split
        { 2.68, -0.27,  -3.0*pi/2.0 },  // [71] reverse 20 cm
        { 2.48, -0.27,  -3.0*pi/2.0 },  // [72] reverse 20 cm
        // [72]→[73]: dx=0, dy=0.25≥0.05 → no split (dx=0 < 0.10)
        { 2.48, -0.52,  -3.0*pi/2.0 },  // [73] right 25 cm  (no split: dx=0 < 0.10)
        // [73]→[74]: dx=0.40≥0.10, dy=0 < 0.05 → no split
        { 2.68, -0.52,  -3.0*pi/2.0 },  // [74] forward 20 cm
        { 2.88, -0.52,  -3.0*pi/2.0 },  // [75] forward 20 cm
        // [75]→[76]: dx=0.40≥0.10, dy=0 < 0.05 → no split
        { 2.68, -0.52,  -3.0*pi/2.0 },  // [76] reverse 20 cm
        { 2.48, -0.52,  -3.0*pi/2.0 },  // [77] reverse 20 cm
        // [77]→[78]: dx=0, dy=0.19≥0.05 → no split (dx=0 < 0.10)
        { 2.48, -0.71,  -3.0*pi/2.0 },  // [78] right 19 cm  (no split: dx=0 < 0.10)
        // [78]→[79]: dx=0.40≥0.10, dy=0 < 0.05 → no split
        { 2.68, -0.71,  -3.0*pi/2.0 },  // [79] forward 20 cm
        { 2.88, -0.71,  -3.0*pi/2.0 },  // [80] forward 20 cm
        // [80]→[81]: dx=0.40≥0.10, dy=0 < 0.05 → no split
        { 2.68, -0.71,  -3.0*pi/2.0 },  // [81] reverse 20 cm
        { 2.48, -0.71,  -3.0*pi/2.0 },  // [82] reverse 20 cm

        // ── Cave april tag sweep row 2 ─────────────────────────────────────────────
        // [82]→[83]: dx=0, dy=0.41≥0.05 → no split (dx=0 < 0.10)
        { 2.48, -0.30,  -3.0*pi/2.0 },  // [83] return center  (no split: dx=0 < 0.10)
        { 2.48, -0.30,  -pi_2 },         // [84] rotate downward (pure rotation)
        // [84]→[85]: dx=0, dy=0.04 < 0.05 → no split
        { 2.48, -0.34,  -pi_2 },          // [85] shift right 4 cm  (no split: dy < 0.05)
        // [85]→[86]: dx=0.40≥0.10, dy=0 < 0.05 → no split
        { 2.68, -0.34,  -pi_2 },          // [86] forward 20 cm
        { 2.88, -0.34,  -pi_2 },          // [87] forward 20 cm
        // [87]→[88]: dx=0.40≥0.10, dy=0 < 0.05 → no split
        { 2.68, -0.34,  -pi_2 },          // [88] reverse 20 cm
        { 2.48, -0.34,  -pi_2 },          // [89] reverse 20 cm
        // [89]→[90]: dx=0, dy=0.25≥0.05 → no split (dx=0 < 0.10)
        { 2.48, -0.09,  -pi_2 },          // [90] left 25 cm  (no split: dx=0 < 0.10)
        // [90]→[91]: dx=0.40≥0.10, dy=0 < 0.05 → no split
        { 2.68, -0.09,  -pi_2 },          // [91] forward 20 cm
        { 2.88, -0.09,  -pi_2 },          // [92] forward 20 cm
        // [92]→[93]: dx=0.40≥0.10, dy=0 < 0.05 → no split
        { 2.68, -0.09,  -pi_2 },          // [93] reverse 20 cm
        { 2.48, -0.09,  -pi_2 },          // [94] reverse 20 cm
        // [94]→[95]: dx=0, dy=0.19≥0.05 → no split (dx=0 < 0.10)
        { 2.48,  0.10,  -pi_2 },          // [95] left 19 cm  (no split: dx=0 < 0.10)
        // [95]→[96]: dx=0.40≥0.10, dy=0 < 0.05 → no split
        { 2.68,  0.10,  -pi_2 },          // [96] forward 20 cm
        { 2.88,  0.10,  -pi_2 },          // [97] forward 20 cm
        // [97]→[98]: dx=0.40≥0.10, dy=0 < 0.05 → no split
        { 2.68,  0.10,  -pi_2 },          // [98] reverse 20 cm
        { 2.48,  0.10,  -pi_2 },          // [99] reverse 20 cm

        // ── Final transit ──────────────────────────────────────────────────────────
        // [99]→[100]: dx=0, dy=0.05≥0.05 → no split (dx=0 < 0.10)
        { 2.48,  0.05,  -pi_2 },          // [100] center align  (no split: dx=0 < 0.10)
        { 2.48,  0.05,  -pi   },          // [101] rotate forward (pure rotation)
        // [101]→[102]: dx=1.80≥0.10, dy=0 < 0.05 → no split
        { 3.38,  0.05,  -pi   },          // [102] forward 90 cm
        { 4.28,  0.05,  -pi   },          // [103] forward 90 cm
        { 4.28,  0.05,   0.0  },          // [104] turn around  (pure rotation)
        // [104]→[105]: dx=1.00≥0.10, dy=0 < 0.05 → no split
        { 4.78,  0.05,   0.0  },          // [105] forward 50 cm
        { 5.28,  0.05,   0.0  },          // [106] forward 50 cm
        { 5.28,  0.05,  -pi_2 },          // [107] rotate right  (pure rotation)
        // [107]→[108]: dx=0.25≥0.10, dy=0 < 0.05 → no split
        { 5.155, 0.05,  -pi_2 },          // [108] reverse 12.5 cm
        { 5.03,  0.05,  -pi_2 },          // [109] reverse 12.5 cm
        // [109]→[110]: dx=0.25≥0.10, dy=0 < 0.05 → no split
        { 5.155, 0.05,  -pi_2 },          // [110] forward 12.5 cm
        { 5.28,  0.05,  -pi_2 },          // [111] forward 12.5 cm
        // [111]→[112]: dx=0, dy=0.40≥0.05 → no split (dx=0 < 0.10)
        { 5.28, -0.35,  -pi_2 },          // [112] right 40 cm  (no split: dx=0 < 0.10)
        { 5.28, -0.35,  -pi   },          // [113] rotate left  (pure rotation)
        // [113]→[114]: dx=0, dy=0.43≥0.05 → no split (dx=0 < 0.10)
        { 5.28,  0.08,  -pi   },          // [114] left 43 cm  (no split: dx=0 < 0.10)
        // [114]→[115]: dx=0.30≥0.10, dy=0 < 0.05 → no split
        { 5.13,  0.08,  -pi   },          // [115] reverse 15 cm
        { 4.98,  0.08,  -pi   }, */         // [116] reverse 15 cm
    };
}

// ============================================================================
// Path 1 – TAG ID 1 | PLACEHOLDER
// Replace this stub with your actual Path 1 waypoints.
// ============================================================================
static std::vector<Setpoint> Path_1() {
    using namespace PathConst;
    return {
        // ── TODO: Define Path 1 waypoints ──────────────────────────────────────
        { 0.0, 0.0, 0.0 },  // [0] Placeholder — robot holds start position
    };
}

// ============================================================================
// Path 2 – TAG ID 2 | PLACEHOLDER
// Replace this stub with your actual Path 2 waypoints.
// ============================================================================
static std::vector<Setpoint> Path_2() {
    using namespace PathConst;
    return {
        // ── TODO: Define Path 2 waypoints ──────────────────────────────────────
        { 0.0, 0.0, 0.0 },  // [0] Placeholder — robot holds start position
    };
}

// ============================================================================
// Path 3 – TAG ID 3 | PLACEHOLDER
// ============================================================================
static std::vector<Setpoint> Path_3() {
    using namespace PathConst;
    return {
        // ── TODO: Define Path 3 waypoints ──────────────────────────────────────
        { 0.0, 0.0, 0.0 },  // [0] Placeholder — robot holds start position
    };
}

// ============================================================================
// Path 4 – TAG ID 4 | PLACEHOLDER
// ============================================================================
static std::vector<Setpoint> Path_4() {
    using namespace PathConst;
    return {
        // ── TODO: Define Path 4 waypoints ──────────────────────────────────────
        { 0.0, 0.0, 0.0 },  // [0] Placeholder — robot holds start position
    };
}

// ============================================================================
// GetPath – AprilTag ID dispatcher
// ============================================================================
//
// Called by Robot.cpp's LoadAutonomousSetpoints():
//
//   m_setpoints = AutonomousPaths::GetPath(tagId);
//
// Tag ID → Path mapping:
//   0     : Default / no tag detected  → Path_Default()
//   1     : Path 1 (TBD)              → Path_1()
//   2     : Path 2 (TBD)              → Path_2()
//   3     : Path 3 (TBD)              → Path_3()
//   4     : Path 4 (TBD)              → Path_4()
//   other : Falls back to Path_Default()
//
inline std::vector<Setpoint> GetPath(int tagId) {
    switch (tagId) {
        case 1:  return Path_1();
        case 2:  return Path_2();
        case 3:  return Path_3();
        case 4:  return Path_4();
        // ── Add new cases above this line ─────────────────────────────────────
        // case 5: return Path_5();
        case 0:
        default: return Path_Default();
    }
}

}  // namespace AutonomousPaths