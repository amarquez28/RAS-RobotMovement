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

       //First pick up
        {0.81, 0.20, -pi_2},   
        {0.81, 0.24, -pi_2},  
        {0.81, 0.24, 0},   //raise arm a little
        {0.81, 0.24, pi_2},  	
        {0.81, 0.62, pi_2},      
        {0.81, 0.95, pi_2},      
        {0.81, 0.62, pi_2},      
        {0.81, 0.62, 0}, //Raise arm pos - 8

        //Beacon  
        {0.25, 0.62, 0},    //Drop arm

        //first scout
        {0.25, 0.62, pi_2},     
        {0.29, 0.96, pi_2},    
        {0.29, 0.96, 0.0},     
        {0.42, 0.96, 0.0},     				
        {0.42, 0.96, -pi_2},  
        {0.42, 0.71, -pi_2},
        {0.42, 0.45, -pi_2},
        {0.42, 0.22, -pi/2}, 

        //Second bucket			
        {0.42, 0.22, 0.0},   
        {0.72, 0.22, 0.0},    
        {0.72, 0.22, -pi_2}, //Raise
        {0.72, 0.52, -pi_2},
        {0.72, 0.82, -pi_2}, //Drop arm
        {0.72, 0.52, -pi_2},   
        {0.72, 0.33, -pi_2},    
        {0.72, 0.33, 0.0},      
        {0.31, 0.33, 0.0},     //release arm

        //First deposit
	    {0.60, 0.26, 0.0},
	    {0.31, 0.22, 0.0}, //Activate actuator

        //Cave Points
	    {0.65, 0.38, pi_4},
        {1.0, 0.57, pi_4},    
	    {1.0, 0.57, 0.0},
        {1.2, 0.57, 0.0},
        {1.45, 0.57, 0.0},  
        {1.75, 0.57, 0.0},      

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
	    {1.48, 0.57, pi},
	    {1.18, 0.57, pi},

        //Outside cave
	    {1.18, 0.57, pi_2},
	    {1.18, 0.77, pi_2},
	    {1.18, 0.97, pi_2},
	    {1.18, 0.77, pi_2},
	    {1.18, 0.57, pi_2},
	    {1.00, 0.47, pi_4},
	    {0.80, 0.37, pi_4},
	    {0.60, 0.27, pi_4},
	    {0.31, 0.22, 0.0}, //Deposit
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
