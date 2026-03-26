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
//  Path Default  –  DEFAULT  |  FULL COMPETITION PATH  (RAS 2026 field sweep)
//
//  Sequence overview:
//    1.  Drive forward 54 cm.
//    2.  Strafe right 10 cm.
//    3.  Turn right 90° (theta = -π/2).
//    4.  Forward 73 cm, reverse 33 cm.
//    5.  Raise arm 4 s, turn right 90° (theta = -π), lower arm 4 s.
//    6.  Lower-field boustrophedon sweep.
//    7.  Deposit in bucket (x = 1.03, y = -0.23).
//    8.  Upper-field boustrophedon sweep (4 rows).
//    9.  Sweep done; deposit in bucket (x = 1.01, y = -0.23).
//   10.  Transit to cave april-tag zone.
//   11.  Cave sweep: left-side probes, 180° spin, right-side probes.
//   12.  Exit cave, forward 180 cm.
//
//  All coordinates in metres (original waypoints ÷ 100).
//  Theta expressed as multiples of pi (C++20 std::numbers::pi via PathConst).
// ============================================================================
static std::vector<Setpoint> Path_Default() {
    using namespace PathConst;
    return {
        // ── Start ─────────────────────────────────────────────────────────────
        { 0.00,  0.00,  0.0    },  // [0]  start

        // ── Approach ──────────────────────────────────────────────────────────
        { 0.54,  0.00,  0.0    },  // [1]  forward 54 cm
        { 0.54, -0.16,  0.0    },  // [2]  strafe right 16 cm
        { 0.54, -0.16, -pi_2   },  // [3]  turn right 90°
        { 1.27, -0.16, -pi_2   },  // [4]  forward 73 cm
        { 0.94, -0.16, -pi_2   },  // [5]  reverse 33 cm
        { 0.94, -0.16, -pi_2   },  // [6]  raise arm 4 s  (no movement)
        { 0.94, -0.16, -pi     },  // [7]  turn right 90°
        { 0.94, -0.16, -pi     },  // [8]  lower arm 4 s  (no movement) placing beacon

        // ── Lower field sweep ─────────────────────────────────────────────────
        { 0.94,  0.23, -pi     },  // [9]  left 39 cm
        { 1.11,  0.23, -pi     },  // [10] forward 17 cm
        { 0.85,  0.23, -pi     },  // [11] reverse 26 cm
        { 0.85,  0.06, -pi     },  // [12] right 17 cm
        { 1.62,  0.06, -pi     },  // [13] forward 77 cm
        { 1.62,  0.23, -pi     },  // [14] left 17 cm
        { 1.95,  0.23, -pi     },  // [15] forward 33 cm
        { 1.03,  0.23, -pi     },  // [16] reverse 92 cm

        // ── Upper field sweep ─────────────────────────────────────────────────
        { 1.03, -0.01, -pi     },  // [17] right 24 cm
        { 1.96, -0.01, -pi     },  // [18] forward 93 cm
        { 0.86, -0.01, -pi     },  // [19] reverse 110 cm
        { 0.86, -0.25, -pi     },  // [20] right 24 cm
        { 1.93, -0.25, -pi     },  // [21] forward 107 cm
        { 0.86, -0.25, -pi     },  // [22] reverse 107 cm
        { 0.86, -0.49, -pi     },  // [23] right 24 cm
        { 1.76, -0.49, -pi     },  // [24] forward 90 cm
        { 0.86, -0.49, -pi     },  // [25] reverse 90 cm
        { 0.86, -0.58, -pi     },  // [26] right 9 cm
        { 1.76, -0.58, -pi     },  // [27] forward 90 cm
        { 1.01, -0.58, -pi     },  // [28] reverse 75 cm

        // ── Deposit + transit to cave ─────────────────────────────────────────
        { 1.01,  0.23, -pi     },  // [29] left 82 cm
        { 1.01, -0.17, -pi     },  // [30] right 41 cm
        { 2.89, -0.17, -pi     },  // [31] forward 188 cm

        // ── Bucket grab sequence ──────────────────────────────────────────────
        { 2.81, -0.17, -pi     },  // [32] reverse 8 cm
        { 2.81, -0.17, -pi_2   },  // [33] turn left 90°
        { 2.56, -0.17, -pi_2   },  // [34] reverse 25 cm
        { 2.56, -0.34, -pi_2   },  // [35] strafe right 17 cm
        { 2.56, -0.34, -pi_2   },  // [36] grab bucket
        { 2.71, -0.34, -pi_2   },  // [37] forward 15 cm
        { 2.96, -0.34, -pi_2   },  // [38] forward 25 cm
        { 3.21, -0.34, -pi_2   },  // [39] forward 25 cm
        { 3.46, -0.34, -pi_2   },  // [40] forward 25 cm

        // ── Cave april tag ────────────────────────────────────────────────────
        { 2.48, -0.17, -pi           },  
        { 2.48, -0.17, -3.0*pi/2.0   },  
        { 2.48, -0.13, -3.0*pi/2.0   },  

        { 2.88, -0.13, -3.0*pi/2.0   },  
        { 2.48, -0.13, -3.0*pi/2.0   },  
        { 2.48, -0.38, -3.0*pi/2.0   },  
        { 2.88, -0.38, -3.0*pi/2.0   },  
        { 2.48, -0.38, -3.0*pi/2.0   },  
        { 2.48, -0.57, -3.0*pi/2.0   },  
        { 2.88, -0.57, -3.0*pi/2.0   },  
        { 2.48, -0.57, -3.0*pi/2.0   },  

        { 2.48, -0.16, -3.0*pi/2.0   },  
        { 2.48, -0.16, -pi_2         },  
        { 2.48, -0.20, -pi_2         },  

        { 2.88, -0.20, -pi_2         },  
        { 2.48, -0.20, -pi_2         },  
        { 2.48,  0.05, -pi_2         },  
        { 2.88,  0.05, -pi_2         },  
        { 2.48,  0.05, -pi_2         },  
        { 2.48,  0.24, -pi_2         },  
        { 2.88,  0.24, -pi_2         },  
        { 2.48,  0.24, -pi_2         },  

        { 2.48,  0.19, -pi_2         },  
        { 2.48,  0.19, -pi           },  
        { 4.28,  0.19, -pi           },  
        { 4.28,  0.19,  0.0          },  
        { 5.28,  0.19,  0.0          },  
        { 5.28,  0.19, -pi_2         },  
        { 5.03,  0.19, -pi_2         },  
        { 5.28,  0.19, -pi_2         },  
        { 5.28, -0.21, -pi_2         },  
        { 5.28, -0.21, -pi           },  
        { 5.28,  0.22, -pi           },  
        { 4.98,  0.22, -pi           },  
    };
}


// ============================================================================
//  Path 1  –  TAG ID 1  |  PLACEHOLDER
//  Replace this stub with your actual Path 1 waypoints.
//  Strategy notes:
//    - ...
// ============================================================================
static std::vector<Setpoint> Path_1() {
    using namespace PathConst;
    return {
        // ── TODO: Define Path 1 waypoints ────────────────────────────────────
        { 0.0,  0.0,  0.0 },   // [0] Placeholder — robot holds start position
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
//    1      : Path 1 (TBD)             → Path_1()
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