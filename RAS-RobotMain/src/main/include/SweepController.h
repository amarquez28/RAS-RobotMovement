#pragma once

// ============================================================================
// SweepController.h
// Boustrophedon ("lawnmower") sweep state machine for RAS 2026
//
// Field: 4 ft (1.2192 m) wide  x  8 ft (2.4384 m) long
//
// Drivetrain layout:
//   RoboClaw 0x80 – M1 = Left drive,  M2 = Right drive   (forward / back)
//   RoboClaw 0x81 – M1 = Strafe motor                    (left / right)
//   Omni wheels: 72 mm diameter
//
// Encoder specs (from BOM / torque-budget sheet):
//   Vertical motors  (0x80): 751.8 PPR  @ 36.9:1 gearbox  (223 RPM no-load)
//   Horizontal motor (0x81): 384.5 PPR  @ 13.7:1 gearbox  (117 RPM no-load)
//
// Dead-reckoning conversion:
//   Wheel circumference = π × 0.072 m ≈ 0.22619 m
//   Vertical ticks/m  = (751.8) / 0.22619 ≈ 3323 ticks/m
//   Horizontal ticks/m= (384.5) / 0.22619 ≈ 1700 ticks/m
//
// Sweep strategy:
//   • Robot starts at one corner, intake facing the field.
//   • Drives forward (vertical) one full field length (8 ft).
//   • Strafes one strip width (robot width + small overlap).
//   • Drives backward the full field length.
//   • Repeats until the 4-ft width is covered (~3 strips for a ~18" robot).
//   • AprilTag zones: if a tag is seen during sweep, robot notes position
//     and finishes the current strip before homing to the tag zone.
//
// IMU heading correction:
//   • Gyro Z integrates yaw during every forward/backward leg.
//   • If yaw drifts beyond kHeadingTolerance_deg, a correction strafe
//     is blended in to keep the robot on a straight line.
// ============================================================================

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cstdint>
#include <optional>
#include "AprilTagReader.h"

// ── tuneable constants ───────────────────────────────────────────────────────

// Field dimensions
static constexpr double kFieldLength_m  = 2.4384;   // 8 ft
static constexpr double kFieldWidth_m   = 1.2192;   // 4 ft

// Robot / strip geometry
// Adjust kStripWidth_m to match your robot's intake width (plus ~10% overlap).
static constexpr double kStripWidth_m   = 0.35;     // ~14 inches per strip

// Drive speeds (0–127 for RoboClaw packet serial)
static constexpr uint8_t kDriveSpeed    = 55;   // forward / backward
static constexpr uint8_t kStrafeSpeed   = 50;   // left / right strafe
static constexpr uint8_t kCorrectSpeed  = 20;   // heading-correction strafe blend

// Encoder ticks per metre (calculated above)
static constexpr double kVertTicksPerM  = 3323.0;
static constexpr double kHorizTicksPerM = 1700.0;

// How far to travel per leg / strafe (in encoder ticks)
static constexpr int32_t kLegTicks      = static_cast<int32_t>(kFieldLength_m * kVertTicksPerM);   // ~8113
static constexpr int32_t kStrafeTicks   = static_cast<int32_t>(kStripWidth_m  * kHorizTicksPerM);  // ~595

// Heading correction threshold (degrees)
static constexpr double kHeadingTolerance_deg = 2.0;

// Settling delay after a strafe before the next forward leg (seconds)
static constexpr double kStrafeSettleTime_s = 0.3;

// Number of strips needed to cover the field width
// strips = ceil(kFieldWidth_m / kStripWidth_m)
static constexpr int kNumStrips = static_cast<int>((kFieldWidth_m / kStripWidth_m) + 0.999);

// ── state machine ─────────────────────────────────────────────────────────────

enum class SweepState {
    IDLE,
    DRIVE_FORWARD,      // Driving along field length (positive direction)
    STRAFE_OVER,        // Lateral shift to the next strip
    DRIVE_BACKWARD,     // Driving along field length (negative direction)
    STRAFE_BACK_OVER,   // Lateral shift to the next strip (same direction)
    SETTLE,             // Brief pause after a strafe
    COMPLETE            // All strips covered
};

// Direction flag so we can share one SETTLE state for both strafe cases
enum class AfterSettle {
    START_BACKWARD,
    START_FORWARD
};

// ── controller class ──────────────────────────────────────────────────────────

class SweepController {
public:
    SweepController() = default;

    // Call once when autonomous starts
    void Reset() {
        m_state          = SweepState::IDLE;
        m_stripsDone     = 0;
        m_legStartTicks  = 0;
        m_strafeStartTicks = 0;
        m_yawAtLegStart  = 0.0;
        m_tagSeenDuring  = false;
        m_settleTimer.Reset();
        m_settleTimer.Stop();
        m_afterSettle    = AfterSettle::START_FORWARD;
    }

    // Start the sweep
    void Start(int32_t currentVertTicks, int32_t currentHorizTicks, double currentYaw_deg) {
        m_state              = SweepState::DRIVE_FORWARD;
        m_legStartTicks      = currentVertTicks;
        m_strafeStartTicks   = currentHorizTicks;
        m_yawAtLegStart      = currentYaw_deg;
    }

    // ── main update – call every AutonomousPeriodic() tick ──────────────────
    //
    // Returns a DriveCommand the Robot should execute this tick.
    // The Robot is responsible for calling the actual RoboClaw functions.
    //
    // vertTicks   : encoder count from RoboClaw 0x80 M1 (left drive)
    // horizTicks  : encoder count from RoboClaw 0x81 M1 (strafe)
    // yaw_deg     : integrated gyro Z (degrees, positive = CCW)
    // aprilTag    : optional detected tag from AprilTagReader

    struct DriveCommand {
        // Each field is a signed speed in [-127, 127].
        // Positive forward = robot's intake-forward direction.
        int8_t  vertical  = 0;   // drives 0x80 M1 & M2
        int8_t  horizontal = 0;  // drives 0x81 M1
        bool    intakeOn  = true; // intake runs all sweep
        bool    done      = false;
        SweepState state  = SweepState::IDLE;
    };

    DriveCommand Update(int32_t vertTicks,
                        int32_t horizTicks,
                        double  yaw_deg,
                        std::optional<AprilTagData> aprilTag)
    {
        DriveCommand cmd;
        cmd.state    = m_state;
        cmd.intakeOn = (m_state != SweepState::IDLE &&
                        m_state != SweepState::COMPLETE);

        switch (m_state) {

        // ── DRIVE_FORWARD ───────────────────────────────────────────────────
        case SweepState::DRIVE_FORWARD: {
            int32_t traveled = vertTicks - m_legStartTicks;

            // Heading correction: if yaw has drifted, blend a strafe
            double yawError = yaw_deg - m_yawAtLegStart;
            int8_t corrH = 0;
            if (yawError > kHeadingTolerance_deg)       corrH = -kCorrectSpeed;
            else if (yawError < -kHeadingTolerance_deg) corrH =  kCorrectSpeed;

            cmd.vertical   = kDriveSpeed;
            cmd.horizontal = corrH;

            // Note any AprilTag seen during this leg
            if (aprilTag.has_value()) {
                m_tagSeenDuring = true;
                m_lastTagId     = aprilTag->id;
            }

            if (traveled >= kLegTicks) {
                // Finished this forward leg → strafe to next strip
                cmd.vertical  = 0;
                cmd.horizontal = 0;
                m_stripsDone++;
                if (m_stripsDone >= kNumStrips) {
                    m_state = SweepState::COMPLETE;
                } else {
                    m_strafeStartTicks = horizTicks;
                    m_state            = SweepState::STRAFE_OVER;
                }
            }
            break;
        }

        // ── DRIVE_BACKWARD ──────────────────────────────────────────────────
        case SweepState::DRIVE_BACKWARD: {
            int32_t traveled = m_legStartTicks - vertTicks; // negative direction

            double yawError = yaw_deg - m_yawAtLegStart;
            int8_t corrH = 0;
            if (yawError > kHeadingTolerance_deg)       corrH =  kCorrectSpeed;
            else if (yawError < -kHeadingTolerance_deg) corrH = -kCorrectSpeed;

            cmd.vertical   = -static_cast<int8_t>(kDriveSpeed);
            cmd.horizontal = corrH;

            if (aprilTag.has_value()) {
                m_tagSeenDuring = true;
                m_lastTagId     = aprilTag->id;
            }

            if (traveled >= kLegTicks) {
                cmd.vertical  = 0;
                cmd.horizontal = 0;
                m_stripsDone++;
                if (m_stripsDone >= kNumStrips) {
                    m_state = SweepState::COMPLETE;
                } else {
                    m_strafeStartTicks = horizTicks;
                    m_state            = SweepState::STRAFE_BACK_OVER;
                }
            }
            break;
        }

        // ── STRAFE_OVER (after a forward leg) ───────────────────────────────
        case SweepState::STRAFE_OVER: {
            int32_t traveled = horizTicks - m_strafeStartTicks;
            cmd.horizontal = kStrafeSpeed;

            if (traveled >= kStrafeTicks) {
                cmd.horizontal = 0;
                // Begin settle timer, then start backward leg
                m_settleTimer.Reset();
                m_settleTimer.Start();
                m_legStartTicks = vertTicks;
                m_yawAtLegStart = yaw_deg;
                m_afterSettle   = AfterSettle::START_BACKWARD;
                m_state         = SweepState::SETTLE;
            }
            break;
        }

        // ── STRAFE_BACK_OVER (after a backward leg) ─────────────────────────
        case SweepState::STRAFE_BACK_OVER: {
            int32_t traveled = horizTicks - m_strafeStartTicks;
            cmd.horizontal = kStrafeSpeed; // always strafe same direction

            if (traveled >= kStrafeTicks) {
                cmd.horizontal = 0;
                m_settleTimer.Reset();
                m_settleTimer.Start();
                m_legStartTicks = vertTicks;
                m_yawAtLegStart = yaw_deg;
                m_afterSettle   = AfterSettle::START_FORWARD;
                m_state         = SweepState::SETTLE;
            }
            break;
        }

        // ── SETTLE ──────────────────────────────────────────────────────────
        case SweepState::SETTLE: {
            cmd.vertical   = 0;
            cmd.horizontal = 0;
            if (m_settleTimer.Get().value() >= kStrafeSettleTime_s) {
                m_settleTimer.Stop();
                m_state = (m_afterSettle == AfterSettle::START_BACKWARD)
                          ? SweepState::DRIVE_BACKWARD
                          : SweepState::DRIVE_FORWARD;
            }
            break;
        }

        // ── COMPLETE ────────────────────────────────────────────────────────
        case SweepState::COMPLETE: {
            cmd.vertical   = 0;
            cmd.horizontal = 0;
            cmd.intakeOn   = false;
            cmd.done       = true;
            break;
        }

        case SweepState::IDLE:
        default:
            break;
        }

        // Push state to dashboard every tick
        PublishDashboard(vertTicks, horizTicks, yaw_deg);
        return cmd;
    }

    SweepState GetState()    const { return m_state; }
    bool       IsComplete()  const { return m_state == SweepState::COMPLETE; }
    bool       TagWasSeen()  const { return m_tagSeenDuring; }
    int        LastTagId()   const { return m_lastTagId; }
    int        StripsDone()  const { return m_stripsDone; }

private:
    void PublishDashboard(int32_t v, int32_t h, double yaw) {
        const char* stateStr = "UNKNOWN";
        switch (m_state) {
            case SweepState::IDLE:            stateStr = "IDLE";           break;
            case SweepState::DRIVE_FORWARD:   stateStr = "DRIVE_FORWARD";  break;
            case SweepState::STRAFE_OVER:     stateStr = "STRAFE_OVER";    break;
            case SweepState::DRIVE_BACKWARD:  stateStr = "DRIVE_BACKWARD"; break;
            case SweepState::STRAFE_BACK_OVER:stateStr = "STRAFE_BACK";    break;
            case SweepState::SETTLE:          stateStr = "SETTLE";         break;
            case SweepState::COMPLETE:        stateStr = "COMPLETE";       break;
        }
        frc::SmartDashboard::PutString("Sweep/State",        stateStr);
        frc::SmartDashboard::PutNumber("Sweep/StripsDone",   m_stripsDone);
        frc::SmartDashboard::PutNumber("Sweep/TotalStrips",  kNumStrips);
        frc::SmartDashboard::PutNumber("Sweep/VertTicks",    v);
        frc::SmartDashboard::PutNumber("Sweep/HorizTicks",   h);
        frc::SmartDashboard::PutNumber("Sweep/Yaw_deg",      yaw);
        frc::SmartDashboard::PutBoolean("Sweep/TagSeen",     m_tagSeenDuring);
        frc::SmartDashboard::PutBoolean("Sweep/Complete",    IsComplete());
    }

    SweepState  m_state          = SweepState::IDLE;
    int         m_stripsDone     = 0;
    int32_t     m_legStartTicks  = 0;
    int32_t     m_strafeStartTicks = 0;
    double      m_yawAtLegStart  = 0.0;
    bool        m_tagSeenDuring  = false;
    int         m_lastTagId      = -1;
    AfterSettle m_afterSettle    = AfterSettle::START_FORWARD;
    frc::Timer  m_settleTimer;
};
