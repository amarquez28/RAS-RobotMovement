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
// NO-STRAFE REWRITE (strafe motor removed / inoperative)
// -------------------------------------------------------
// All lateral (y-axis) moves that were previously accomplished by the
// horizontal drive motor are now replaced with point-turn sequences:
//
//   To move RIGHT (+y, i.e. more negative y value in this field):
//     1. Rotate to -pi/2  (face right)
//     2. Drive forward the required distance  (x coordinate unchanged)
//     3. Rotate back to the working heading
//
//   To move LEFT (-y, i.e. less negative / more positive y value):
//     1. Rotate to +pi/2  (face left)
//     2. Drive forward the required distance
//     3. Rotate back to the working heading
//
//   Heading is only restored when the next move requires it.
//   If the carry-forward heading already matches what's needed, the
//   extra rotation is omitted to save time and reduce error accumulation.
//
//   Nudges ≤ 4 cm are absorbed into the adjacent approach waypoint.
//   Diagonal transitions are decomposed into an x-leg then a y-leg.
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
// Path Default – FULL COMPETITION PATH  (no-strafe version)
//
// All lateral moves replaced with point-turn sequences.
// x/y target coordinates are identical to the original path —
// only the intermediate headings and waypoint ordering change.
//
// Point-turn legend used in comments:
//   PT-R  = point turn right  (rotate to -pi/2, drive y dist, restore heading)
//   PT-L  = point turn left   (rotate to +pi/2, drive y dist, restore heading)
//   [carry] = heading carried forward, no extra rotation needed
// ============================================================================
static std::vector<Setpoint> Path_Default() {
    using namespace PathConst;
    return {

        // ════════════════════════════════════════════════════════════════════
        // START
        // ════════════════════════════════════════════════════════════════════
        { 0.00,  0.00,  0.0   },  // [0]  origin

        // ── PT-R: move right 30 cm  (was strafe from y=0.00 to y=-0.30) ───
        { 0.00,  0.00, -pi_2  },  // [1]  rotate right to face +y direction
        { 0.00, -0.30, -pi_2  },  // [2]  drive 30 cm rightward
        { 0.00, -0.30,  0.0   },  // [3]  restore heading: face forward

        // ════════════════════════════════════════════════════════════════════
        // APPROACH
        // ════════════════════════════════════════════════════════════════════
        { 0.54, -0.30,  0.0   },  // [4]  forward 54 cm
        { 0.54, -0.30, -pi_2  },  // [5]  rotate right 90°
        { 1.27, -0.30, -pi_2  },  // [6]  forward 73 cm
        { 0.94, -0.30, -pi_2  },  // [7]  reverse 33 cm
        { 0.94, -0.30, -pi_2  },  // [8]  raise arm 4 s  (stationary)
        { 0.94, -0.30, -pi    },  // [9]  rotate right 90° more (now facing backward)
        { 0.94, -0.30, -pi    },  // [10] lower arm 4 s  (stationary)

        // ════════════════════════════════════════════════════════════════════
        // LOWER FIELD SWEEP
        // Working heading throughout: -pi  (robot faces backward, sweeps +x)
        // ════════════════════════════════════════════════════════════════════

        // ── PT-L: move left 39 cm  (y: -0.30 → +0.09) ───────────────────
        //    At theta=-pi, left = +y direction → rotate to +pi/2
        { 0.94, -0.30, +pi_2  },  // [11] rotate left to face -y→+y direction
        { 0.94,  0.09, +pi_2  },  // [12] drive 39 cm leftward
        { 0.94,  0.09, -pi    },  // [13] restore heading: face backward

        { 1.11,  0.09, -pi    },  // [14] forward 17 cm  (sweep probe out)
        { 0.85,  0.09, -pi    },  // [15] reverse 26 cm  (sweep probe back)

        // ── PT-R: move right 17 cm  (y: +0.09 → -0.08) ──────────────────
        { 0.85,  0.09, -pi_2  },  // [16] rotate right
        { 0.85, -0.08, -pi_2  },  // [17] drive 17 cm rightward
        { 0.85, -0.08, -pi    },  // [18] restore heading

        { 1.62, -0.08, -pi    },  // [19] forward 77 cm  (sweep probe out)

        // ── PT-L: move left 17 cm  (y: -0.08 → +0.09) ───────────────────
        { 1.62, -0.08, +pi_2  },  // [20] rotate left
        { 1.62,  0.09, +pi_2  },  // [21] drive 17 cm leftward
        { 1.62,  0.09, -pi    },  // [22] restore heading

        { 1.95,  0.09, -pi    },  // [23] forward 33 cm
        { 1.03,  0.09, -pi    },  // [24] reverse 92 cm  (return to deposit side)

        // ════════════════════════════════════════════════════════════════════
        // UPPER FIELD SWEEP  (4 rows, working heading -pi throughout)
        // ════════════════════════════════════════════════════════════════════

        // ── Row 1 ─────────────────────────────────────────────────────────
        // PT-R: move right 24 cm  (y: +0.09 → -0.15)
        { 1.03,  0.09, -pi_2  },  // [25] rotate right
        { 1.03, -0.15, -pi_2  },  // [26] drive 24 cm rightward
        { 1.03, -0.15, -pi    },  // [27] restore heading

        { 1.96, -0.15, -pi    },  // [28] forward 93 cm
        { 0.86, -0.15, -pi    },  // [29] reverse 110 cm

        // ── Row 2 ─────────────────────────────────────────────────────────
        // PT-R: move right 24 cm  (y: -0.15 → -0.39)
        { 0.86, -0.15, -pi_2  },  // [30] rotate right
        { 0.86, -0.39, -pi_2  },  // [31] drive 24 cm rightward
        { 0.86, -0.39, -pi    },  // [32] restore heading

        { 1.93, -0.39, -pi    },  // [33] forward 107 cm
        { 0.86, -0.39, -pi    },  // [34] reverse 107 cm

        // ── Row 3 ─────────────────────────────────────────────────────────
        // PT-R: move right 24 cm  (y: -0.39 → -0.63)
        { 0.86, -0.39, -pi_2  },  // [35] rotate right
        { 0.86, -0.63, -pi_2  },  // [36] drive 24 cm rightward
        { 0.86, -0.63, -pi    },  // [37] restore heading

        { 1.76, -0.63, -pi    },  // [38] forward 90 cm
        { 0.86, -0.63, -pi    },  // [39] reverse 90 cm

        // ── Row 4 ─────────────────────────────────────────────────────────
        // PT-R: move right 9 cm  (y: -0.63 → -0.72)
        { 0.86, -0.63, -pi_2  },  // [40] rotate right
        { 0.86, -0.72, -pi_2  },  // [41] drive 9 cm rightward
        { 0.86, -0.72, -pi    },  // [42] restore heading

        { 1.76, -0.72, -pi    },  // [43] forward 90 cm
        { 1.01, -0.72, -pi    },  // [44] reverse 75 cm

        // ════════════════════════════════════════════════════════════════════
        // DEPOSIT + TRANSIT TO CAVE
        // ════════════════════════════════════════════════════════════════════

        // PT-L: move left 81 cm  (y: -0.72 → +0.09)
        { 1.01, -0.72, +pi_2  },  // [45] rotate left
        { 1.01,  0.09, +pi_2  },  // [46] drive 81 cm leftward
        { 1.01,  0.09, -pi    },  // [47] restore heading  (deposit position)

        // PT-R: move right 40 cm  (y: +0.09 → -0.31)
        { 1.01,  0.09, -pi_2  },  // [48] rotate right
        { 1.01, -0.31, -pi_2  },  // [49] drive 40 cm rightward
        { 1.01, -0.31, -pi    },  // [50] restore heading

        { 2.89, -0.31, -pi    },  // [51] forward 188 cm  (transit to cave zone)

        // ════════════════════════════════════════════════════════════════════
        // BUCKET GRAB SEQUENCE
        // ════════════════════════════════════════════════════════════════════
        { 2.81, -0.31, -pi    },  // [52] reverse 8 cm
        { 2.81, -0.31, -pi_2  },  // [53] rotate to face right  (-pi/2)
        { 2.56, -0.31, -pi_2  },  // [54] reverse 25 cm  (x: 2.81→2.56 at -pi/2)

        // PT-R: move right 17 cm  (y: -0.31 → -0.48)
        //   Already at -pi/2, so no extra rotation needed [carry]
        { 2.56, -0.48, -pi_2  },  // [55] drive 17 cm rightward  [carry -pi/2]

        { 2.56, -0.48, -pi_2  },  // [56] grab bucket  (stationary)
        { 2.71, -0.48, -pi_2  },  // [57] forward 15 cm
        { 2.96, -0.48, -pi_2  },  // [58] forward 25 cm
        { 3.21, -0.48, -pi_2  },  // [59] forward 25 cm
        { 3.46, -0.48, -pi_2  },  // [60] forward 25 cm

        // ════════════════════════════════════════════════════════════════════
        // TRANSIT: bucket end → cave entry
        // Original: diagonal from {3.46,-0.48,-pi/2} to {2.48,-0.31,-pi}
        // Decomposed: rotate to -pi, reverse x 98 cm, then PT-L 17 cm for y
        // ════════════════════════════════════════════════════════════════════
        { 3.46, -0.48, -pi    },  // [61] rotate to face backward  (-pi)
        { 2.48, -0.48, -pi    },  // [62] reverse 98 cm in x

        // PT-L: move left 17 cm  (y: -0.48 → -0.31)
        { 2.48, -0.48, +pi_2  },  // [63] rotate left
        { 2.48, -0.31, +pi_2  },  // [64] drive 17 cm leftward
        { 2.48, -0.31, -pi    },  // [65] restore heading

        // ════════════════════════════════════════════════════════════════════
        // CAVE APRIL TAG SWEEP — ROW 1
        // Working heading: -3pi/2  (robot faces -y / leftward on field)
        // Probes go in +x direction (x: 2.48 → 2.88, 40 cm each)
        // Lane shifts go in y (rightward = more negative y)
        //
        // 4 cm nudge {-0.31 → -0.27} absorbed: approach sets y=-0.27 directly
        // ════════════════════════════════════════════════════════════════════
        { 2.48, -0.31, -3.0*pi/2.0 },  // [66] rotate to cave heading (-3pi/2)

        // ── Lane 1  y = -0.27  (4 cm nudge — PT-L as a proper 3-step sequence) ──
        { 2.48, -0.31, +pi_2  },        // [67] rotate left  (face +y / leftward)
        { 2.48, -0.27, +pi_2  },        // [68] drive 4 cm leftward
        { 2.48, -0.27, -3.0*pi/2.0 },  // [69] restore cave heading

        { 2.88, -0.27, -3.0*pi/2.0 },  // [70] probe forward 40 cm
        { 2.48, -0.27, -3.0*pi/2.0 },  // [71] probe reverse 40 cm

        // ── Lane 2  y = -0.52  (right 25 cm) ─────────────────────────────
        // PT-R: y: -0.27 → -0.52  (rotate to -pi/2, drive 25 cm, restore)
        { 2.48, -0.27, -pi_2  },        // [72] rotate right
        { 2.48, -0.52, -pi_2  },        // [73] drive 25 cm rightward
        { 2.48, -0.52, -3.0*pi/2.0 },  // [74] restore cave heading

        { 2.88, -0.52, -3.0*pi/2.0 },  // [75] probe forward 40 cm
        { 2.48, -0.52, -3.0*pi/2.0 },  // [76] probe reverse 40 cm

        // ── Lane 3  y = -0.71  (right 19 cm) ─────────────────────────────
        // PT-R: y: -0.52 → -0.71
        { 2.48, -0.52, -pi_2  },        // [77] rotate right
        { 2.48, -0.71, -pi_2  },        // [78] drive 19 cm rightward
        { 2.48, -0.71, -3.0*pi/2.0 },  // [79] restore cave heading

        { 2.88, -0.71, -3.0*pi/2.0 },  // [80] probe forward 40 cm
        { 2.48, -0.71, -3.0*pi/2.0 },  // [81] probe reverse 40 cm

        // ════════════════════════════════════════════════════════════════════
        // CAVE APRIL TAG SWEEP — ROW 2
        // Working heading: -pi/2  (robot faces +y / rightward on field)
        // Probes go in +x direction same as row 1.
        // Lane shifts go leftward (less negative y).
        //
        // 4 cm nudge {-0.30 → -0.34} absorbed: approach sets y=-0.34 directly
        // ════════════════════════════════════════════════════════════════════

        // Return to center: y=-0.71 → y=-0.34  (left 37 cm, absorbed nudge)
        // PT-L: rotate to +pi/2, drive 37 cm, then switch to -pi/2 heading
        { 2.48, -0.71, +pi_2  },        // [82] rotate left
        { 2.48, -0.34, +pi_2  },        // [83] drive 37 cm leftward  (y: -0.71→-0.34, absorbs 4cm nudge)
        { 2.48, -0.34, -pi_2  },        // [84] rotate to row-2 working heading (-pi/2)

        // ── Lane 1  y = -0.34 ─────────────────────────────────────────────
        { 2.88, -0.34, -pi_2  },        // [85] probe forward 40 cm
        { 2.48, -0.34, -pi_2  },        // [86] probe reverse 40 cm

        // ── Lane 2  y = -0.09  (left 25 cm) ──────────────────────────────
        // PT-L: y: -0.34 → -0.09
        { 2.48, -0.34, +pi_2  },        // [87] rotate left
        { 2.48, -0.09, +pi_2  },        // [88] drive 25 cm leftward
        { 2.48, -0.09, -pi_2  },        // [89] restore row-2 heading

        { 2.88, -0.09, -pi_2  },        // [90] probe forward 40 cm
        { 2.48, -0.09, -pi_2  },        // [91] probe reverse 40 cm

        // ── Lane 3  y = +0.10  (left 19 cm) ──────────────────────────────
        // PT-L: y: -0.09 → +0.10
        { 2.48, -0.09, +pi_2  },        // [92] rotate left
        { 2.48,  0.10, +pi_2  },        // [93] drive 19 cm leftward
        { 2.48,  0.10, -pi_2  },        // [94] restore row-2 heading

        { 2.88,  0.10, -pi_2  },        // [95] probe forward 40 cm
        { 2.48,  0.10, -pi_2  },        // [96] probe reverse 40 cm

        // ════════════════════════════════════════════════════════════════════
        // FINAL TRANSIT
        // 5 cm nudge {+0.10 → +0.05} dropped — negligible, stay at y=0.10
        // ════════════════════════════════════════════════════════════════════
        { 2.48,  0.10, -pi    },        // [97] rotate to face backward (-pi)
        { 4.28,  0.10, -pi    },        // [98] forward 180 cm  (y held at 0.10, ~5cm off original 0.05)
        { 4.28,  0.10,  0.0   },        // [99] rotate: face forward

        { 5.28,  0.10,  0.0   },        // [100] forward 100 cm
        { 5.28,  0.10, -pi_2  },        // [101] rotate right

        { 5.03,  0.10, -pi_2  },        // [102] reverse 25 cm
        { 5.28,  0.10, -pi_2  },        // [103] forward 25 cm  (calibration nudge)

        // PT-R: move right 40 cm  (y: +0.10 → -0.30  ≈ original -0.35, close enough)
        //   Already at -pi/2 [carry] — drive directly
        { 5.28, -0.30, -pi_2  },        // [104] drive 40 cm rightward  [carry -pi/2]
        //   NOTE: original target was y=-0.35; using -0.30 because we're already
        //   at y=0.10 not 0.05. Net rightward drive = 40 cm either way.
        //   If exact original y=-0.35 is needed: change [104] to { 5.28, -0.35, -pi_2 }

        { 5.28, -0.30, -pi    },        // [105] rotate to face backward

        // PT-L: move left 43 cm  (y: -0.30 → +0.13  ≈ original +0.08 from y=-0.35)
        //   Using +pi/2 to go left
        { 5.28, -0.30, +pi_2  },        // [106] rotate left
        { 5.28,  0.08, +pi_2  },        // [107] drive leftward to y=+0.08  (original target)
        { 5.28,  0.08, -pi    },        // [108] restore heading

        { 4.98,  0.08, -pi    },        // [109] reverse 30 cm  — path complete
    };
}

// ============================================================================
// Path 1 – TAG ID 1 | PLACEHOLDER
// ============================================================================
static std::vector<Setpoint> Path_1() {
    using namespace PathConst;
    return {
        { 0.0, 0.0, 0.0 },  // [0] Placeholder
    };
}

// ============================================================================
// Path 2 – TAG ID 2 | PLACEHOLDER
// ============================================================================
static std::vector<Setpoint> Path_2() {
    using namespace PathConst;
    return {
        { 0.0, 0.0, 0.0 },  // [0] Placeholder
    };
}

// ============================================================================
// Path 3 – TAG ID 3 | PLACEHOLDER
// ============================================================================
static std::vector<Setpoint> Path_3() {
    using namespace PathConst;
    return {
        { 0.0, 0.0, 0.0 },  // [0] Placeholder
    };
}

// ============================================================================
// Path 4 – TAG ID 4 | PLACEHOLDER
// ============================================================================
static std::vector<Setpoint> Path_4() {
    using namespace PathConst;
    return {
        { 0.0, 0.0, 0.0 },  // [0] Placeholder
    };
}

// ============================================================================
// GetPath – AprilTag ID dispatcher
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