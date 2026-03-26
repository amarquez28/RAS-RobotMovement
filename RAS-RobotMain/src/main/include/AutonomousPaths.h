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
        { 0.00,  0.00,  0.0    },  // [0] start
        { 0.00, -0.15,  0.0    },  // [1] right half 30 cm
        { 0.00, -0.30,  0.0    },  // [2] right second half

        // ── Approach ──────────────────────────────────────────────────────────
        { 0.27, -0.30,  0.0    },  // [3] forward half 54 cm
        { 0.54, -0.30,  0.0    },  // [4] forward second half
        { 0.54, -0.30, -pi_2   },  // [5] turn right 90°
        { 0.905, -0.30, -pi_2  },  // [6] forward half 73 cm
        { 1.27, -0.30, -pi_2   },  // [7] forward second half
        { 1.105, -0.30, -pi_2  },  // [8] reverse half 33 cm
        { 0.94, -0.30, -pi_2   },  // [9] reverse second half
        { 0.94, -0.30, -pi_2   },  // [10] raise arm 4 s
        { 0.94, -0.30, -pi     },  // [11] turn right 90°
        { 0.94, -0.30, -pi     },  // [12] lower arm 4 s

        // ── Lower field sweep ─────────────────────────────────────────────────
        { 0.94, -0.105, -pi    },  // [13] left half 39 cm
        { 0.94,  0.09, -pi     },  // [14] left second half
        { 1.025, 0.09, -pi     },  // [15] forward half 17 cm
        { 1.11, 0.09, -pi      },  // [16] forward second half
        { 0.975, 0.09, -pi     },  // [17] reverse half 26 cm
        { 0.85, 0.09, -pi      },  // [18] reverse second half
        { 0.85, 0.005, -pi     },  // [19] right half 17 cm
        { 0.85, -0.08, -pi     },  // [20] right second half
        { 1.28, -0.08, -pi     },  // [21] forward half 77 cm
        { 1.62, -0.08, -pi     },  // [22] forward second half
        { 1.785, 0.01, -pi     },  // [23] left half 17 cm
        { 1.95, 0.09, -pi      },  // [24] left second half
        { 1.49, 0.09, -pi      },  // [25] forward half 33 cm
        { 1.95, 0.09, -pi      },  // [26] forward second half
        { 1.49, 0.09, -pi      },  // [27] reverse half 92 cm
        { 1.03, 0.09, -pi      },  // [28] reverse second half

        // ── Upper field sweep ─────────────────────────────────────────────────
        { 1.03, -0.075, -pi    },  // [29] right half 24 cm
        { 1.03, -0.15, -pi     },  // [30] right second half
        { 1.495, -0.15, -pi    },  // [31] forward half 93 cm
        { 1.96, -0.15, -pi     },  // [32] forward second half
        { 1.41, -0.15, -pi     },  // [33] reverse half 110 cm
        { 0.86, -0.15, -pi     },  // [34] reverse second half
        { 0.86, -0.27, -pi     },  // [35] right half 24 cm
        { 0.86, -0.39, -pi     },  // [36] right second half
        { 1.395, -0.39, -pi    },  // [37] forward half 107 cm
        { 1.93, -0.39, -pi     },  // [38] forward second half
        { 1.395, -0.39, -pi    },  // [39] reverse half 107 cm
        { 0.86, -0.39, -pi     },  // [40] reverse second half
        { 0.86, -0.51, -pi     },  // [41] right half 24 cm
        { 0.86, -0.63, -pi     },  // [42] right second half
        { 1.31, -0.63, -pi     },  // [43] forward half 90 cm
        { 1.76, -0.63, -pi     },  // [44] forward second half
        { 1.31, -0.63, -pi     },  // [45] reverse half 90 cm
        { 0.86, -0.63, -pi     },  // [46] reverse second half
        { 0.86, -0.675, -pi    },  // [47] right half 9 cm
        { 0.86, -0.72, -pi     },  // [48] right second half
        { 1.32, -0.72, -pi     },  // [49] forward half 90 cm
        { 1.76, -0.72, -pi     },  // [50] forward second half
        { 0.935, -0.72, -pi    },  // [51] reverse half 75 cm
        { 1.01, -0.72, -pi     },  // [52] reverse second half

        // ── Deposit + transit to cave ─────────────────────────────────────────
        { 1.01, -0.315, -pi    },  // [53] left half 81 cm
        { 1.01,  0.09, -pi     },  // [54] left second half
        { 1.01, -0.13, -pi     },  // [55] right half 40 cm
        { 1.01, -0.31, -pi     },  // [56] right second half
        { 2.59, -0.31, -pi     },  // [57] forward half 188 cm
        { 2.89, -0.31, -pi     },  // [58] forward second half

        // ── Bucket grab sequence ──────────────────────────────────────────────
        { 2.85, -0.31, -pi     },  // [59] reverse half 8 cm
        { 2.81, -0.31, -pi     },  // [60] reverse second half
        { 2.81, -0.31, -pi_2   },  // [61] turn left 90°
        { 2.685, -0.31, -pi_2  },  // [62] reverse half 25 cm
        { 2.56, -0.31, -pi_2   },  // [63] reverse second half
        { 2.56, -0.395, -pi_2  },  // [64] right half 17 cm
        { 2.56, -0.48, -pi_2   },  // [65] right second half
        { 2.56, -0.48, -pi_2   },  // [66] grab bucket
        { 2.635, -0.48, -pi_2  },  // [67] forward half 15 cm
        { 2.71, -0.48, -pi_2   },  // [68] forward second half
        { 2.835, -0.48, -pi_2  },  // [69] forward half 25 cm
        { 2.96, -0.48, -pi_2   },  // [70] forward second half
        { 3.085, -0.48, -pi_2  },  // [71] forward half 25 cm
        { 3.21, -0.48, -pi_2   },  // [72] forward second half
        { 3.335, -0.48, -pi_2  },  // [73] forward half 25 cm
        { 3.46, -0.48, -pi_2   },  // [74] forward second half

        // ── Cave april tag sweep row 1 ────────────────────────────────────────
        { 2.64, -0.395, -pi    },  // [75] move half to cave entry
        { 2.48, -0.31, -pi     },  // [76] move second half
        { 2.48, -0.31, -3.0*pi/2.0 },  // [77] turn upward
        { 2.48, -0.29, -3.0*pi/2.0 },  // [78] shift left half 4 cm
        { 2.48, -0.27, -3.0*pi/2.0 },  // [79] shift left second half
        { 2.68, -0.27, -3.0*pi/2.0 },  // [80] forward half 40 cm
        { 2.88, -0.27, -3.0*pi/2.0 },  // [81] forward second half
        { 2.68, -0.27, -3.0*pi/2.0 },  // [82] reverse half 40 cm
        { 2.48, -0.27, -3.0*pi/2.0 },  // [83] reverse second half
        { 2.48, -0.395, -3.0*pi/2.0 }, // [84] right half 25 cm
        { 2.48, -0.52, -3.0*pi/2.0 },  // [85] right second half
        { 2.68, -0.52, -3.0*pi/2.0 },  // [86] forward half 40 cm
        { 2.88, -0.52, -3.0*pi/2.0 },  // [87] forward second half
        { 2.68, -0.52, -3.0*pi/2.0 },  // [88] reverse half 40 cm
        { 2.48, -0.52, -3.0*pi/2.0 },  // [89] reverse second half
        { 2.48, -0.645, -3.0*pi/2.0 }, // [90] right half 19 cm
        { 2.48, -0.71, -3.0*pi/2.0 },  // [91] right second half
        { 2.68, -0.71, -3.0*pi/2.0 },  // [92] forward half 40 cm
        { 2.88, -0.71, -3.0*pi/2.0 },  // [93] forward second half
        { 2.68, -0.71, -3.0*pi/2.0 },  // [94] reverse half 40 cm
        { 2.48, -0.71, -3.0*pi/2.0 },  // [95] reverse second half

        // ── Cave april tag sweep row 2 ────────────────────────────────────────
        { 2.48, -0.315, -3.0*pi/2.0 }, // [96] return half to center
        { 2.48, -0.30, -3.0*pi/2.0  }, // [97] return second half
        { 2.48, -0.32, -pi_2       }, // [98] rotate half downward
        { 2.48, -0.30, -pi_2       }, // [99] rotate second half downward
        { 2.48, -0.32, -pi_2       }, // [100] shift right half 4 cm
        { 2.48, -0.34, -pi_2       }, // [101] shift right second half
        { 2.68, -0.34, -pi_2       }, // [102] forward half 40 cm
        { 2.88, -0.34, -pi_2       }, // [103] forward second half
        { 2.68, -0.34, -pi_2       }, // [104] reverse half 40 cm
        { 2.48, -0.34, -pi_2       }, // [105] reverse second half
        { 2.48, -0.215, -pi_2      }, // [106] left half 25 cm
        { 2.48, -0.09, -pi_2       }, // [107] left second half
        { 2.68, -0.09, -pi_2       }, // [108] forward half 40 cm
        { 2.88, -0.09, -pi_2       }, // [109] forward second half
        { 2.68, -0.09, -pi_2       }, // [110] reverse half 40 cm
        { 2.48, -0.09, -pi_2       }, // [111] reverse second half
        { 2.48, 0.005, -pi_2       }, // [112] left half 19 cm
        { 2.48, 0.10, -pi_2       },  // [113] left second half
        { 2.68, 0.10, -pi_2       },  // [114] forward half 40 cm
        { 2.88, 0.10, -pi_2       },  // [115] forward second half
        { 2.68, 0.10, -pi_2       },  // [116] reverse half 40 cm
        { 2.48, 0.10, -pi_2       },  // [117] reverse second half

        // ── Final transit ─────────────────────────────────────────────────────
        { 2.48, 0.075, -pi_2      }, // [118] center half
        { 2.48, 0.05, -pi_2       }, // [119] center second half
        { 2.48, 0.05, -pi         }, // [120] rotate forward
        { 3.38, 0.05, -pi         }, // [121] forward half 180 cm
        { 4.28, 0.05, -pi         }, // [122] forward second half
        { 4.28, 0.05, 0.0         }, // [123] turn around
        { 4.78, 0.05, 0.0         }, // [124] forward half 100 cm
        { 5.28, 0.05, 0.0         }, // [125] forward second half
        { 5.28, 0.05, -pi_2      }, // [126] rotate right
        { 5.155, 0.05, -pi_2     }, // [127] reverse half 25 cm
        { 5.03, 0.05, -pi_2      }, // [128] reverse second half
        { 5.155, 0.05, -pi_2     }, // [129] forward half 25 cm
        { 5.28, 0.05, -pi_2      }, // [130] forward second half
        { 5.28, -0.175, -pi_2    }, // [131] right half 40 cm
        { 5.28, -0.35, -pi_2     }, // [132] right second half
        { 5.28, -0.135, -pi      }, // [133] rotate left half
        { 5.28, -0.35, -pi       }, // [134] rotate left second half
        { 5.13, -0.135, -pi      }, // [135] left half 43 cm
        { 4.98, 0.08, -pi        }, // [136] left second half
        { 5.13, 0.08, -pi        }, // [137] reverse half 30 cm
        { 4.98, 0.08, -pi        }, // [138] reverse second half
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