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

        // [0] Placeholder — robot holds cave entrance position
        {0.81, 0.16, -pi_2},      // 0 Origin
        {0.81, 0.30, 0.0},
        {0.81, 0.30, pi_2},    // 1 Reverse 40 cm
        {0.81, 0.65, pi_2},
        {0.81,0.57, pi_2},
        

        {0.32, 0.62, 0.0},      // 2 Pickup arm
        {0.19, 0.62, 0.0},      // 3 Reverse 21 cm
        {0.29, 0.62, 0.0},      // 4 Drop arm
        {0.29, 0.62, pi_2},     // 5 Turn left 90°
        {0.29, 0.96, pi_2},    // 6 Forward 30 cm along rotated x→y
        {0.29, 0.96, 0.0},      // 7 Turn right 90°
        {0.42, 0.96, 0.0},     // 8 Forward 20 cm
        {0.42, 0.96, -pi_2},  //k  // 9 Turn right 90°
        {0.42, 0.22, -pi/2}, //k  // 10 Forward 70 cm
        {0.42, 0.22, 0.0},   //k  // 11 Turn left 90°
        {0.62, 0.22, 0.0},     // 12 Forward 20 cm
        {0.62, 0.22, pi_2}, //k   // 13 Turn left 90°
        {0.62, 0.78, pi_2},   //k  // 14 Forward 51 cm
        {0.62, 0.22, pi_2},    // 15 Reverse 51 cm
        {0.62, 0.22, 0.0},     // 16 Turn right 90°
        {0.79, 0.22, 0.0},     // 17 Forward 17 cm
        {0.79, 0.22, pi_2},    // 18 Turn left 90°
        {0.79, 0.76, pi_2},  //k   // 19 Forward 51 cm
        {0.79, 0.76, 0.0},      // 20 Turn right 90°
        {0.79, 0.76, -pi_2}, //k   // 21 Turn right 90°
        {0.79, 0.85, -pi_2},    // 22 Raise arm //CONFIRM EITHER 79 OR 76
        {0.79, 0.85, -pi_2},  //k  // 23 Reverse 8 cm
        {0.79, 0.85, -pi_2},  //k  // 24 Drop arm
        {0.79, 0.59, -pi_2},    // 25 Forward 25 cm
        {0.79, 0.59, 0.0},   //k   // 26 Turn left 90°
        {0.30, 0.59, 0.0},      // 27 Reverse 43 cm
        {0.30, 0.59, 0.0},      // 28 Lift arm
        {0.76, 0.59, 0.0},   //k   // 29 Forward 43 cm
        {0.76, 0.59, -pi_2},    // 30 Turn right 90°
        {0.76, 0.46, -pi_2}, //k  // 31 Forward 13 cm
        {0.76, 0.46, 0.0}, //k    // 32 Turn left 90°
        {0.28, 0.37, 0.0},     // 33 Reverse 45 cm
        {0.29, 0.37, 0.0},     // 34 Deposit
        {1.12, 0.37, 0.0},     // 35 Forward 92 cm
        {0.62, 0.37, 0.0},     // 36 Reverse 50 cm
        {0.62, 0.37, -pi_2},   // 37 Turn right 90°
        {0.62, 0.2, -pi_2},   // 38 Forward 20 cm
        {0.62, 0.2, 0},  //k  // 39 Turn left 90°
        {1.00, 0.2, 0},    // 40 Forward 35 cm
        {0.90, 0.2, 0},    // 41 Reverse 35 cm //maybe dont reverse here
        {0.90, 0.2, pi_2}, //k   // 42 Turn left 90° //OPEN ARM
        {0.90, 0.92, pi_2},     // 43 Forward 40 cm
        {0.90, 0.57, pi_2},//k    // 44 Turn right 90°
        {0.90, 0.57, 0},  //k  // 45 Forward 52 cm //drop arm
        {1.08, 0.92, pi_2}, //k   // 46 Reverse 52 cm
        {1.20, 0.33, pi_2},  //k   // 47 Turn left 90°
        {1.20, 0.60, pi_2},  //k   // 48 Forward 20 cm
        {1.20, 0.60, 0},    // 49 Turn right 90° 
        {0.46, 0.54, 0},    // 50 Forward 52 cm //
        {0.66, 0.48, 0}, //Deposit   // 51 Reverse 52 cm
        {0.46, 0.42, 0.0},    //finish deposit  // 52 Turn right 90° //open
        {1.08, 0.59, 0.0},   //Cave Entrance   // 53 Forward 40 cm //closed
        //Cave
        {2.20, 0.57, 0},
        {1.75, 0.57, 0},
        {1.75, 0.57, pi_2},
        {1.75, 0.95, pi_2},
        {1.75, 0.57, pi_2},
        {1.75, 0.57, 0},
        {1.94,  0.57, 0},
        {1.94,  0.57, pi_2},
        {1.94,  0.95, pi_2},
        {1.94,  0.57, pi_2},
        {1.94,  0.57, 0},
        {2.15, 0.57, 0},
        {2.15, 0.57, pi_2},
        {2.15, 0.95, pi_2},
        {2.15, 0.57, pi_2},
        {2.15, 0.57, -pi_2},
        {2.15, 0.57, -pi_2},
        {2.15, 0.05, -pi_2},
        {2.15, 0.57, -pi_2},
        {2.15, 0.57, pi},
        {1.95,  0.57, pi},
        {1.95,  0.57, -pi_2},
        {1.95,  0.05, -pi_2},
        {1.95,  0.57, -pi_2},
        {1.95,  0.57, pi},
        {1.75,  0.57, pi},
        {1.75,  0.57, -pi_2},
        {1.75,  0.05, -pi_2},
        {1.75,  0.57, -pi_2},
        {1.75,  0.57, pi},
        {0.95,  0.57, pi},
        {0.95,  0.57, pi_2},
        {0.95, 0.57, 0},
        {0.46,  0.38, 0}, //DEPOSIT
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
        
        /*
        {0.69, 0.39, pi_2},     // 55 Reverse 42 cm
        {1.11, 0.39, pi_2},     // 56 Forward 42 cm
        {1.11, 0.39, pi_2},     // 57 Turn left 90°
        {1.26, 0.39, pi_2},     // 58 Forward 15 cm
        {1.26, 0.39, -pi_2},    // 59 Turn right 90°
        {2.76, 0.39, -pi_2},    // 60 Forward 150 cm
        {2.28, 0.39, -pi_2},    // 61 Reverse 48 cm
        {2.28, 0.39, pi_2},     // 62 Turn left 90°
        {2.68, 0.39, pi_2},     // 63 Forward 40 cm
        {2.28, 0.39, pi_2},     // 64 Reverse 40 cm
        {2.28, 0.39, -pi_2},    // 65 Turn right 90°
        {2.48, 0.39, -pi_2},    // 66 Forward 20 cm
        {2.48, 0.39, pi_2},     // 67 Turn left 90°
        {2.88, 0.39, pi_2},     // 68 Forward 40 cm
        {2.48, 0.39, pi_2},     // 69 Reverse 40 cm
        {2.48, 0.39, -pi_2},    // 70 Turn right 90°
        {2.67, 0.39, -pi_2},    // 71 Forward 19 cm
        {2.67, 0.39, pi_2},     // 72 Turn left 90°
        {3.07, 0.39, pi_2},     // 73 Forward 40 cm
        {2.67, 0.39, pi_2},     // 74 Reverse 40 cm
        {2.67, 0.39, pi_2},     // 75 Turn left 90°
        {2.67, 0.39, pi_2},     // 76 Turn left 90°
        {3.07, 0.39, pi_2},     // 77 Forward 40 cm
        {2.67, 0.39, pi_2},     // 78 Reverse 40 cm
        {2.67, 0.39, -pi_2},    // 79 Turn right 90°
        {2.86, 0.39, -pi_2},    // 80 Forward 19 cm
        {2.86, 0.39, pi_2},     // 81 Turn left 90°
        {3.26, 0.39, pi_2},     // 82 Forward 40 cm
        {2.86, 0.39, pi_2},     // 83 Reverse 40 cm
        {2.86, 0.39, -pi_2},    // 84 Turn right 90°
        {3.06, 0.39, -pi_2},    // 85 Forward 20 cm
        {3.06, 0.39, pi_2},     // 86 Turn left 90°
        {3.46, 0.39, pi_2},     // 87 Forward 40 cm
        {3.06, 0.39, pi_2},     // 88 Reverse 40 cm
        {3.06, 0.39, pi_2},     // 89 Turn left 90°
        {1.86, 0.39, pi_2},     // 90 Reverse 120 cm
        {1.86, 0.39, pi},       // 91 Turn left 90°
        {1.72, 0.39, pi},       // 92 Reverse 14 cm
        {1.46, 0.39, pi},       // 93 Reverse 26 cm
        {1.46, 0.39, pi},       // 94 Deposit
        {2.16, 0.39, pi},       // 95 Forward 70 cm
        {2.16, 0.39, pi_2},     // 96 Turn left 90° */
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
