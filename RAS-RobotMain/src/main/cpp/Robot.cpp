// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// ============================================================================
//  Robot.cpp  –  RAS 2026  |  IEEE Region 5 Robotics Competition
// ============================================================================
//
//  Autonomous behaviour:
//    1. Vision system (Pi) detects AprilTag → transitions from SEARCH to APPROACH.
//    2. APPROACH uses a full PID controller (x / y / theta) to drive toward
//       the tag using encoder dead-reckoning + IMU heading.
//    3. When the tag distance drops to ≤ kStopDistance_m the robot stops,
//       publishes task_done to NetworkTables, and enters DONE.
//
//  IMU integration:
//    • RobotPeriodic() calls IMUUpdate() every tick (consistent dt).
//    • AutonomousPeriodic() calls CalibrateGyroZBias() once at init, then
//      integrates bias-corrected gyro-Z in radians (m_thetaRad) for PID.
//
// ============================================================================

#include "Robot.h"

#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/print.h>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <iostream>
#include <numbers>
#include <units/angle.h>
#include <units/length.h>

// ── File-scope IMU helper ────────────────────────────────────────────────────
// Convert two big-endian bytes to a signed 16-bit integer.
// Defined first so every function below can use it without a forward declaration.
static int16_t ToInt16(uint8_t hi, uint8_t lo) {
    return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
}

// ── Constructor ──────────────────────────────────────────────────────────────

Robot::Robot() : frc::TimesliceRobot{5_ms, 10_ms} {
    // LiveWindow causes drastic overruns in robot periodic functions
    frc::LiveWindow::DisableAllTelemetry();

    // Timeslice scheduling: 5 ms robot periodic + 2 ms + 2 ms = 9 ms / 10 ms
    Schedule([=] {}, 2_ms);
    Schedule([=] {}, 2_ms);

    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

    // Power and enable all four servo channels
    m_servoBrush.SetPowered(true);   m_servoBrush.SetEnabled(true);
    m_servoRelease.SetPowered(true); m_servoRelease.SetEnabled(true);
    m_servoHall.SetPowered(true);    m_servoHall.SetEnabled(true);
    m_servoArm.SetPowered(true);     m_servoArm.SetEnabled(true);

    m_servoBrush.SetPulseWidth(BrushServoInitPos);
    m_servoRelease.SetPulseWidth(ReleaseServoInitPos);
    m_servoHall.SetPulseWidth(kHallServoInitPos);
    m_servoArm.SetPulseWidth(ArmServoInitPos);

    // Initialise IMU
    IMUInit();

    // Clean RoboClaw buffers and stop all motors on boot
    m_roboclaw.Reset();
    RoboClawStopAll();

    // NetworkTables publishers – Pi reads these to know task status
    auto inst  = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("Vision");
    m_taskDonePub  = table->GetBooleanTopic("task_done").Publish();
    m_sweepDonePub = table->GetBooleanTopic("sweep_done").Publish();
    m_taskDonePub.Set(false);
    m_sweepDonePub.Set(false);
}

// ============================================================================
//  RoboClaw low-level helpers
// ============================================================================

// CRC-16 / CCITT (polynomial 0x1021)
uint16_t Robot::RoboClawCRC16(const uint8_t* data, int len) {
    uint16_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}

// Discard any stale bytes sitting in the RX buffer
void Robot::RoboClawDrain() {
    while (m_roboclaw.GetBytesReceived() > 0) {
        char junk[64];
        int n = std::min<int>(sizeof(junk), m_roboclaw.GetBytesReceived());
        if (n <= 0) break;
        m_roboclaw.Read(junk, n);
    }
}

// Send a 3-byte command + CRC (used for all motor drive commands)
void Robot::RoboClawSend3(uint8_t addr, uint8_t cmd, uint8_t value) {
    uint8_t  pkt[3] = {addr, cmd, value};
    uint16_t crc    = RoboClawCRC16(pkt, 3);
    uint8_t  out[5] = {
        addr, cmd, value,
        static_cast<uint8_t>((crc >> 8) & 0xFF),
        static_cast<uint8_t>(crc & 0xFF)
    };
    m_roboclaw.Write(reinterpret_cast<const char*>(out), 5);
}

// Read exactly n bytes within a timeout; returns false on timeout
static bool ReadExact(frc::SerialPort& port, uint8_t* out, int n,
                      units::second_t timeout) {
    auto start = frc::Timer::GetFPGATimestamp();
    int  got   = 0;
    while (got < n) {
        char buf[32];
        int want = std::min(n - got, (int)sizeof(buf));
        int r    = port.Read(buf, want);
        if (r > 0) { std::memcpy(out + got, buf, r); got += r; }
        if ((frc::Timer::GetFPGATimestamp() - start) > timeout) return false;
    }
    return true;
}

// Generic encoder read: cmd 16 = M1, cmd 17 = M2
bool Robot::RoboClawReadEncoder(uint8_t addr, uint8_t cmd,
                                 int32_t& count, uint8_t& status) {
    RoboClawDrain();

    uint8_t  req[2]  = {addr, cmd};
    uint16_t reqCrc  = RoboClawCRC16(req, 2);
    uint8_t  out[4]  = {
        addr, cmd,
        static_cast<uint8_t>((reqCrc >> 8) & 0xFF),
        static_cast<uint8_t>(reqCrc & 0xFF)
    };
    m_roboclaw.Write(reinterpret_cast<const char*>(out), 4);

    // Response: 4-byte count + 1-byte status + 2-byte CRC = 7 bytes
    uint8_t resp[7];
    if (!ReadExact(m_roboclaw, resp, 7, 20_ms)) return false;

    uint32_t raw = (uint32_t(resp[0]) << 24) | (uint32_t(resp[1]) << 16) |
                   (uint32_t(resp[2]) <<  8) | (uint32_t(resp[3]) <<  0);
    count  = static_cast<int32_t>(raw);
    status = resp[4];

    uint8_t check[7];
    check[0] = addr;
    check[1] = cmd;
    std::memcpy(&check[2], resp, 5);

    uint16_t rxCrc   = (uint16_t(resp[5]) << 8) | uint16_t(resp[6]);
    uint16_t calcCrc = RoboClawCRC16(check, 7);
    return calcCrc == rxCrc;
}

bool Robot::RoboClawReadEncoderM1(uint8_t addr, int32_t& count, uint8_t& status) {
    return RoboClawReadEncoder(addr, 16, count, status);
}
bool Robot::RoboClawReadEncoderM2(uint8_t addr, int32_t& count, uint8_t& status) {
    return RoboClawReadEncoder(addr, 17, count, status);
}

// Reset encoder counts (command 20) on one RoboClaw address
void Robot::RoboClawResetEncoder(uint8_t addr) {
    uint8_t  pkt[2] = {addr, 20};
    uint16_t crc    = RoboClawCRC16(pkt, 2);
    uint8_t  out[4] = {
        addr, 20,
        static_cast<uint8_t>((crc >> 8) & 0xFF),
        static_cast<uint8_t>(crc & 0xFF)
    };
    m_roboclaw.Write(reinterpret_cast<const char*>(out), 4);
}

void Robot::RoboClawResetAllEncoders() {
    RoboClawResetEncoder(kRoboClawAddr_Drive);
    RoboClawResetEncoder(kRoboClawAddr_Strafe);
}

// ── Per-motor direction wrappers ─────────────────────────────────────────────
void Robot::RoboClawM1Forward (uint8_t addr, uint8_t speed) { RoboClawSend3(addr, 0, speed); }
void Robot::RoboClawM1Backward(uint8_t addr, uint8_t speed) { RoboClawSend3(addr, 1, speed); }
void Robot::RoboClawM2Forward (uint8_t addr, uint8_t speed) { RoboClawSend3(addr, 4, speed); }
void Robot::RoboClawM2Backward(uint8_t addr, uint8_t speed) { RoboClawSend3(addr, 5, speed); }

void Robot::RoboClawStop(uint8_t addr) {
    RoboClawM1Forward(addr, 0);
    RoboClawM2Forward(addr, 0);
}
void Robot::RoboClawStopAll() {
    RoboClawStop(kRoboClawAddr_Drive);
    RoboClawStop(kRoboClawAddr_Strafe);
}

// ============================================================================
//  High-level drive helpers
// ============================================================================

void Robot::DriveVertical(int8_t speed) {
    if (speed > 0) {
        uint8_t s = static_cast<uint8_t>(speed);
        RoboClawM1Forward(kRoboClawAddr_Drive, s);
        RoboClawM2Forward(kRoboClawAddr_Drive, s);
    } else if (speed < 0) {
        uint8_t s = static_cast<uint8_t>(-speed);
        RoboClawM1Backward(kRoboClawAddr_Drive, s);
        RoboClawM2Backward(kRoboClawAddr_Drive, s);
    } else {
        RoboClawStop(kRoboClawAddr_Drive);
    }
}

void Robot::DriveHorizontal(int8_t speed) {
    if (speed > 0) {
        RoboClawM1Forward(kRoboClawAddr_Strafe, static_cast<uint8_t>(speed));
    } else if (speed < 0) {
        RoboClawM1Backward(kRoboClawAddr_Strafe, static_cast<uint8_t>(-speed));
    } else {
        RoboClawM1Forward(kRoboClawAddr_Strafe, 0);
    }
}

void Robot::StopAllDrive() {
    RoboClawStopAll();
}

// DriveTankSteered – differential steering used during tag approach.
// baseSpeed: signed (-127..127). pixelError: tag.x − kCameraCenter_px.
void Robot::DriveTankSteered(int8_t baseSpeed, double pixelError) {
    double normError = pixelError / kCenterTolerance_px;
    if (normError >  1.0) normError =  1.0;
    if (normError < -1.0) normError = -1.0;

    double base       = static_cast<double>(baseSpeed);
    double leftSpeed  = base + base * normError * kSteerFactor;
    double rightSpeed = base - base * normError * kSteerFactor;

    auto clamp127 = [](double v) -> int8_t {
        if (v >  127.0) v =  127.0;
        if (v < -127.0) v = -127.0;
        return static_cast<int8_t>(v);
    };
    int8_t l = clamp127(leftSpeed);
    int8_t r = clamp127(rightSpeed);

    frc::SmartDashboard::PutNumber("Drive/LeftSpeed",  l);
    frc::SmartDashboard::PutNumber("Drive/RightSpeed", r);

    if (l >= 0) RoboClawM1Forward (kRoboClawAddr_Drive, static_cast<uint8_t>( l));
    else        RoboClawM1Backward(kRoboClawAddr_Drive, static_cast<uint8_t>(-l));
    if (r >= 0) RoboClawM2Forward (kRoboClawAddr_Drive, static_cast<uint8_t>( r));
    else        RoboClawM2Backward(kRoboClawAddr_Drive, static_cast<uint8_t>(-r));
}

// ============================================================================
//  IMU (MPU-6050) helpers
// ============================================================================

bool Robot::IMUWriteReg(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    return m_imu.WriteBulk(buf, 2); // WPILib: false = success
}

bool Robot::IMUReadRegs(uint8_t startReg, uint8_t* out, int len) {
    if (m_imu.WriteBulk(&startReg, 1)) return true; // true = error
    return m_imu.ReadOnly(len, out);                 // true = error
}

void Robot::IMUInit() {
    IMUWriteReg(0x6B, 0x00); // wake
    IMUWriteReg(0x1B, 0x00); // gyro ±250 dps
    IMUWriteReg(0x1C, 0x00); // accel ±2g
}
void Robot::ResetIMUState() {
    m_thetaRad = 0.0;
    m_yaw_rad = 0.0;
    m_lastGyroZ_radps = 0.0;
    m_lastIMUTime = frc::Timer::GetFPGATimestamp();
}

// IMUUpdate() – called every RobotPeriodic() tick.
// Performs ONE I2C burst read per tick and integrates gyro-Z in radians.
//
//   m_thetaRad  : bias-corrected heading used by the PID controller.
//   m_yaw_rad   : raw (no bias correction) heading used by sweep / dashboard.
//
// Having a single read here means AutonomousPeriodic() reads m_thetaRad
// directly instead of issuing a second I2C transaction.
void Robot::IMUUpdate() {
    uint8_t data[14] = {0};
    if (IMUReadRegs(0x3B, data, 14)) return; // I2C error → skip, keep last value

    int16_t gz_raw    = ToInt16(data[12], data[13]);
    double  gz_radps  = (gz_raw / 131.0) * (std::numbers::pi / 180.0);

    auto   now = frc::Timer::GetFPGATimestamp();
    double dt  = (now - m_lastIMUTime).value();
    m_lastIMUTime = now;
    if (dt <= 0.0 || dt > 0.05) dt = 0.005; // guard: clamp to 5 ms on first call

    // Trapezoidal integration (average of previous and current sample)
    double avg_radps = (m_lastGyroZ_radps + gz_radps) / 2.0;
    m_lastGyroZ_radps = gz_radps;

    // Raw yaw (no bias correction) – used by sweep and dashboard
    m_yaw_rad  += avg_radps * dt;
    m_yaw_rad   = WrapAngle(m_yaw_rad);

    // Bias-corrected heading used by PID
    m_thetaRad += (avg_radps - m_gyroZBiasRadps) * dt;
    m_thetaRad  = WrapAngle(m_thetaRad);
}

// ============================================================================
//  Gyro bias calibration
//  Call once in AutonomousInit() while the robot is stationary.
//  Averages 500 IMU samples (in rad/s) to estimate the static Z-axis bias.
// ============================================================================

void Robot::CalibrateGyroZBias() {
    constexpr int kSamples = 500;
    double sum   = 0.0;
    int    count = 0;

    for (int i = 0; i < kSamples; i++) {
        uint8_t data[14] = {0};
        if (!IMUReadRegs(0x3B, data, 14)) {  // false = success
            int16_t gz    = ToInt16(data[12], data[13]);
            double gz_rps = (gz / 131.0) * (std::numbers::pi / 180.0);
            sum += gz_rps;
            count++;
        }
        frc::Wait(0.002_s);
    }

    m_gyroZBiasRadps = (count > 0) ? (sum / count) : 0.0;
    frc::SmartDashboard::PutNumber("IMU/GyroZBias_radps", m_gyroZBiasRadps);
}

// ============================================================================
//  Encoder odometry update
// ============================================================================

void Robot::UpdateEncoders() {
    uint8_t s1, s2;
    int32_t raw1 = 0, raw2 = 0;

    bool ok1 = RoboClawReadEncoderM1(kRoboClawAddr_Drive,  raw1, s1);
    bool ok2 = RoboClawReadEncoderM1(kRoboClawAddr_Strafe, raw2, s2);

    m_encodersValid = ok1 && ok2;
    if (ok1) m_vertTicks  = raw1;
    if (ok2) m_horizTicks = raw2;

    frc::SmartDashboard::PutBoolean("Enc/DriveOK",   ok1);
    frc::SmartDashboard::PutBoolean("Enc/StrafeOK",  ok2);
    frc::SmartDashboard::PutNumber ("Enc/VertTicks",  m_vertTicks);
    frc::SmartDashboard::PutNumber ("Enc/HorizTicks", m_horizTicks);
}

// ============================================================================
//  PID helpers – setpoint list management
// ============================================================================

double Robot::WrapAngle(double angle) {
    while (angle >  std::numbers::pi) angle -= 2.0 * std::numbers::pi;
    while (angle < -std::numbers::pi) angle += 2.0 * std::numbers::pi;
    return angle;
}

void Robot::LoadAutonomousSetpoints() {
    int tag_id = 1;
    // int tag_id = m_aprilTagReader.GetPrimaryTag().id;
    m_setpoints = AutonomousPaths::GetPath(tag_id);
    m_currentSetpointIndex = 0;
    m_autoComplete = m_setpoints.empty();
}

void Robot::AdvanceToNextSetpoint() {
    ResetPidState();
    m_waypointStartTime_s = m_timer.Get().value();  // restart per-waypoint clock
    if (m_currentSetpointIndex + 1 < m_setpoints.size()) {
        m_currentSetpointIndex++;
        std::cout << "[Auto] Advanced to waypoint " << m_currentSetpointIndex << "\n";
    } else {
        m_autoComplete = true;
        std::cout << "[Auto] All waypoints complete\n";
    }
}

void Robot::ResetPidState() {
    x_integral     = 0.0;
    y_integral     = 0.0;
    theta_integral = 0.0;

    x_prevError     = 0.0;
    y_prevError     = 0.0;
    theta_prevError = 0.0;

    x_deriv = 0.0;
    y_deriv = 0.0;
    theta_deriv = 0.0;
}

//Reset pose estimator, this is going to help fix a position in the field and push actual pose to that position 
void Robot::ResetPoseEstimator(const frc::Pose2d& pose,
                               units::meter_t leftDist,
                               units::meter_t rightDist,
                               frc::Rotation2d gyroAngle) {
    field_poseEstimator.ResetPosition(
        gyroAngle,
        leftDist,
        rightDist,
        pose
    );
}

//Update every cycle the pose estimator with time values, IMU and encoder readings
frc::Pose2d Robot::UpdatePoseEstimator(units::second_t timestamp,
                                       units::meter_t leftDist,
                                       units::meter_t rightDist,
                                       frc::Rotation2d gyroAngle) {
    return field_poseEstimator.UpdateWithTime(
        timestamp,
        gyroAngle,
        leftDist,
        rightDist
    );
}

// ============================================================================
//  RobotPeriodic  – runs every packet regardless of mode
// ============================================================================

void Robot::RobotPeriodic() {
    // Single IMU read / integration source for all modes
    IMUUpdate();

    // Vision dashboard
    m_aprilTagReader.UpdateDashboard();

    // Hall sensor → hatch door servo
    // if (!m_hallDigital.Get()) {
        // m_servoHall.SetPulseWidth(kHallServoOpenPos);
    // } else {
    //     m_servoHall.SetPulseWidth(kHallServoInitPos);
    // }

    // Hall sensor dashboard
    frc::SmartDashboard::PutNumber("Hall/Voltage_V", m_hallAnalog.GetVoltage());
    frc::SmartDashboard::PutNumber("Hall/Raw",       m_hallAnalog.GetValue());
    frc::SmartDashboard::PutBoolean("Hall/Digital",  m_hallDigital.Get());

    // IMU dashboard using already-integrated state
    frc::SmartDashboard::PutNumber("IMU/Theta_rad", m_thetaRad);
    frc::SmartDashboard::PutNumber("IMU/YawRaw_rad", m_yaw_rad);
    frc::SmartDashboard::PutNumber("IMU/GyroZBias_radps", m_gyroZBiasRadps);
}

// ============================================================================
//  AutonomousInit
// ============================================================================

void Robot::AutonomousInit() {
    m_timer.Reset();
    m_timer.Start();

    // Reset all encoders to zero so dead-reckoning is relative to start pos
    RoboClawResetAllEncoders();

    // Reconfigure IMU and reset software-integrated heading state
    IMUInit();
    ResetIMUState();

    // Calibrate gyro Z bias while robot is stationary
    CalibrateGyroZBias();

    // Reset again so heading starts at zero after calibration
    ResetIMUState();
    frc::Rotation2d gyroAngle{units::radian_t{m_thetaRad}};
    ResetPoseEstimator(m_initialPose, 0_m, 0_m, gyroAngle);

    // m_thetaRad is already zeroed above; reset the PID first-loop flag
    m_firstPidLoop = true;

    // Load the setpoint sequence for the PID approach phase
    LoadAutonomousSetpoints();

    // Reset approach state machine — start by centering on the AprilTag
    m_autoPhase = AutoPhase::TEST;
    m_taskDone  = false;

    // Reset test sequencer
    m_testStep = 0;
    m_timer.Reset();
    m_timer.Start();

    // Clear NT completion flags so the Pi sees the reset
    m_taskDonePub.Set(false);
    m_sweepDonePub.Set(false);

    // Reset PID integrators
    ResetPidState();

    // Snapshot encoders then zero them (UpdateEncoders fills the members;
    // we then overwrite with 0 so sweep distances are relative)
    //UpdateEncoders();
    //m_vertTicks  = 0;
    //m_horizTicks = 0;

    // Close hatch door
    // m_servoHall.SetPulseWidth(kHallServoInitPos);

    // Raise arm immediately so beacon clears the arena during early turns
    m_armRaised = false;
    m_armDropped = false;
    ArmLower();

    // Safety stop
    RoboClawStopAll();
    RoboClawDrain();
    std::cout << "[Auto] AutonomousInit complete\n";
}

// ============================================================================
//  AutonomousPeriodic
//
//  State machine:
//
//   SEARCH ──(tag detected)──► APPROACH ──(distance ≤ kStopDistance_m)──► DONE
//
//  In APPROACH the PID controller runs against a pose-based setpoint.
//  The x/y setpoint is set to the tag's reported distance when the tag is
//  first acquired; the theta setpoint drives the robot to face the tag.
//
//  Motor conflict prevention:
//    Only one code path issues motor commands per tick. The PID output
//    is computed and applied inside the APPROACH case only. No motor
//    commands appear outside the switch.
//
// ============================================================================

void Robot::AutonomousPeriodic() {
    // ── 1. Raise Arm  ──────────────────────────────────────────────────
    // Raise arm once at start of autonomous
    /*if (!m_armRaised) {
        ArmRaise();
        m_armRaised = true;
    }

    // Lower arm once when waypoint 8 is reached
    if (m_currentSetpointIndex >= 8 && !m_armDropped) {
        ArmLower();
        m_armDropped = true;
    }*/

    // ── 2. Vision connection heartbeat ────────────────────────────────────
    // IsConnected() must be called every tick (it compares heartbeat counters).
    bool visionConnected = m_aprilTagReader.IsConnected();
    frc::SmartDashboard::PutBoolean("Vision/PiConnected", visionConnected);
    frc::SmartDashboard::PutBoolean("Auto/Alive", true);
    frc::SmartDashboard::PutNumber("Auto/TestNumber", 5678.0);
    // ── 3. Compute dt for PID ─────────────────────────────────────────────
    units::second_t now = frc::Timer::GetFPGATimestamp();
    double dt = 0.0;

    if (m_firstPidLoop) {
        m_prevTime    = now;
        m_firstPidLoop = false;
        return; // skip first cycle so dt is valid on the next call
    }
    dt = (now - m_prevTime).value();
    m_prevTime = now;
    if (dt <= 1e-4 || dt > 0.1) dt = 0.02; // fallback for stalls / big gaps

    // ── 4. Heading is already current – IMUUpdate() ran in RobotPeriodic() ──
    // m_thetaRad is the bias-corrected radian heading maintained by IMUUpdate().
    // No second I2C read needed here.
    frc::SmartDashboard::PutNumber("IMU/Theta_rad",    m_thetaRad);
    frc::SmartDashboard::PutNumber("IMU/Bias_radps",   m_gyroZBiasRadps);

    // ── 5. Compute encoder-based pose (metres) ────────────────────────────
    // Wheel diameter 72 mm; encoder PPR from BOM:
    //   Drive motors  (0x80): 758.8 PPR effective
    //   Strafe motor  (0x81): 1425.1 PPR effective
    constexpr double kWheelCirc   = std::numbers::pi * 0.072;
    constexpr double kXMetPerPul  = kWheelCirc / 758.8;
    constexpr double kYMetPerPul  = kWheelCirc / 1425.1;

    int32_t e80_m1 = 0, e80_m2 = 0, e81_m1 = 0;
    uint8_t s80_m1,     s80_m2,     s81_m1;

    bool ok80_1 = RoboClawReadEncoderM1(kRoboClawAddr_Drive,  e80_m1, s80_m1);
    bool ok80_2 = RoboClawReadEncoderM2(kRoboClawAddr_Drive,  e80_m2, s80_m2);
    bool ok81_1 = RoboClawReadEncoderM1(kRoboClawAddr_Strafe, e81_m1, s81_m1);

    frc::SmartDashboard::PutBoolean("RC1 Encoder1 OK", ok80_1);
    frc::SmartDashboard::PutBoolean("RC1 Encoder2 OK", ok80_2);
    frc::SmartDashboard::PutBoolean("RC2 Encoder1 OK", ok81_1);
    if (ok80_1) frc::SmartDashboard::PutNumber("RC1 Encoder1", static_cast<double>(e80_m1));
    if (ok80_2) frc::SmartDashboard::PutNumber("RC1 Encoder2", static_cast<double>(e80_m2));
    if (ok81_1) frc::SmartDashboard::PutNumber("RC2 Encoder1", static_cast<double>(e81_m1));

    units::meter_t xl_pos = units::meter_t{e80_m2 * kXMetPerPul};
    units::meter_t xr_pos = units::meter_t{e80_m1 * kXMetPerPul};
    frc::Rotation2d gyroAngle{units::radian_t{m_thetaRad}};
    double rawStrafeY_m = e81_m1 * kYMetPerPul;

    frc::Pose2d estPose = UpdatePoseEstimator(now, xl_pos, xr_pos, gyroAngle);

    double x_pos = estPose.X().value();
    double y_pos = estPose.Y().value();
    double theta_pos = estPose.Rotation().Radians().value();

    frc::SmartDashboard::PutNumber("Pose/X_m", x_pos);
    frc::SmartDashboard::PutNumber("Pose/Y_m", y_pos);
    frc::SmartDashboard::PutNumber("Pose/Theta_rad", theta_pos);
    frc::SmartDashboard::PutNumber("Pose/Theta_rad IMU", m_thetaRad);

    // optional: keep raw strafe encoder visible for debugging
    frc::SmartDashboard::PutNumber("Pose/RawStrafeY_m", rawStrafeY_m);

    // ── 6. Read AprilTag ──────────────────────────────────────────────────
    bool hasTag = m_aprilTagReader.HasTarget();
    AprilTagData tag{};
    if (hasTag) {
        tag = m_aprilTagReader.GetPrimaryTag();
        if (tag.distance <= 0.0) hasTag = false; // reject invalid pose estimate
    }

    // ── 7. State machine ──────────────────────────────────────────────────
    switch (m_autoPhase)
    {
    case AutoPhase::TEST:
    {
        StopAllDrive();
        frc::SmartDashboard::PutString("Auto/Phase", "TEST");

        // ── Connection / heartbeat ───────────────────────────────────────
        bool visionConnected = m_aprilTagReader.IsConnected();
        frc::SmartDashboard::PutBoolean("Test/Vision Connected", visionConnected);
        frc::SmartDashboard::PutBoolean("Test/Has Target", hasTag);
        frc::SmartDashboard::PutNumber("Test/Tag Count", m_aprilTagReader.GetTagCount());

        // ── Primary tag raw data ─────────────────────────────────────────
        if (hasTag) {
            frc::SmartDashboard::PutNumber("Test/Primary ID", tag.id);
            frc::SmartDashboard::PutNumber("Test/Primary Pixel X", tag.x);
            frc::SmartDashboard::PutNumber("Test/Primary Pixel Y", tag.y);
            frc::SmartDashboard::PutNumber("Test/Primary Distance (m)", tag.distance);

            // Camera-relative pose (tilt-corrected by Pi)
            frc::SmartDashboard::PutNumber("Test/Pose TX (m)", tag.pose_tx);
            frc::SmartDashboard::PutNumber("Test/Pose TY (m)", tag.pose_ty);
            frc::SmartDashboard::PutNumber("Test/Pose TZ (m)", tag.pose_tz);

            // Detection quality
            frc::SmartDashboard::PutNumber("Test/Pose Error", tag.pose_err);
            frc::SmartDashboard::PutNumber("Test/Decision Margin", tag.decision_margin);

            // Capture timestamp (FPGA µs, -1 if not synced)
            frc::SmartDashboard::PutNumber("Test/Timestamp us", static_cast<double>(tag.timestamp_us));
        } else {
            frc::SmartDashboard::PutNumber("Test/Primary ID", -1);
            frc::SmartDashboard::PutNumber("Test/Primary Distance (m)", -1.0);
        }

        // ── Field-relative robot pose (computed by Pi) ───────────────────
        FieldPose fieldPose = m_aprilTagReader.GetFieldPose();
        frc::SmartDashboard::PutBoolean("Test/Field Pose Valid", fieldPose.valid);
        frc::SmartDashboard::PutNumber("Test/Field X (m)", fieldPose.x);
        frc::SmartDashboard::PutNumber("Test/Field Y (m)", fieldPose.y);
        frc::SmartDashboard::PutNumber("Test/Field Theta (rad)", fieldPose.theta);
        frc::SmartDashboard::PutNumber("Test/Field Theta (deg)", fieldPose.theta * 180.0 / std::numbers::pi);

        // ── Encoder odometry vs vision comparison ────────────────────────
        // These are already computed above as x_pos, y_pos, theta_pos
        frc::SmartDashboard::PutNumber("Test/Odom X (m)", x_pos);
        frc::SmartDashboard::PutNumber("Test/Odom Y (m)", y_pos);
        frc::SmartDashboard::PutNumber("Test/Odom Theta (rad)", theta_pos);

        // Difference: vision field pose minus encoder odometry
        if (fieldPose.valid) {
            frc::SmartDashboard::PutNumber("Test/Delta X (m)", fieldPose.x - x_pos);
            frc::SmartDashboard::PutNumber("Test/Delta Y (m)", fieldPose.y - y_pos);
            frc::SmartDashboard::PutNumber("Test/Delta Theta (rad)", fieldPose.theta - theta_pos);
        }

        // ── IMU heading ──────────────────────────────────────────────────
        frc::SmartDashboard::PutNumber("Test/IMU Theta (rad)", m_thetaRad);
        frc::SmartDashboard::PutNumber("Test/IMU Theta (deg)", m_thetaRad * 180.0 / std::numbers::pi);

        // ── All visible tags ─────────────────────────────────────────────
        auto allTags = m_aprilTagReader.GetAllTags();
        std::string tagList = "";
        for (size_t i = 0; i < allTags.size(); i++) {
            const auto& t = allTags[i];
            std::string prefix = "Test/Tag[" + std::to_string(i) + "]/";
            frc::SmartDashboard::PutNumber(prefix + "ID", t.id);
            frc::SmartDashboard::PutNumber(prefix + "Distance (m)", t.distance);
            frc::SmartDashboard::PutNumber(prefix + "Pose TX (m)", t.pose_tx);
            frc::SmartDashboard::PutNumber(prefix + "Pose TY (m)", t.pose_ty);
            frc::SmartDashboard::PutNumber(prefix + "Pose TZ (m)", t.pose_tz);
            if (i > 0) tagList += ", ";
            tagList += std::to_string(t.id);
        }
        frc::SmartDashboard::PutString("Test/Visible Tags", tagList);

        // ── Latency estimate ─────────────────────────────────────────────
        if (hasTag && tag.timestamp_us > 0) {
            auto fpgaNow = frc::Timer::GetFPGATimestamp();
            int64_t fpgaNow_us = static_cast<int64_t>(fpgaNow.value() * 1e6);
            double latency_ms = (fpgaNow_us - tag.timestamp_us) / 1000.0;
            frc::SmartDashboard::PutNumber("Test/Vision Latency (ms)", latency_ms);
        }

        break;
    }
    // ── DONE: hold position ───────────────────────────────────────────────
    case AutoPhase::DONE:
        StopAllDrive();
        frc::SmartDashboard::PutString("Auto/Phase", "Done");
        break;

    // ── TAG_SEARCH: default path done, waiting for AprilTag ID ───────────
    // Robot holds position. When a tag is seen, its ID selects the next path.
    // If no tag is found within kTagSearchTimeout_s, go to DONE.
    case AutoPhase::TAG_SEARCH: {
        StopAllDrive();
        double searchElapsed = m_timer.Get().value();
        frc::SmartDashboard::PutString("Auto/Phase", "Tag Search");
        frc::SmartDashboard::PutNumber("Auto/TagSearchTime_s", searchElapsed);

        int tagId = 0;
        bool tagFound = hasTag;
        if (tagFound) {
            tagId = m_aprilTagReader.GetPrimaryTag().id;
            // Tag found — load the corresponding path, reset pose, start running
            std::cout << "[Auto] Tag ID " << tagId << " detected — loading sub-path\n";
            m_setpoints = AutonomousPaths::GetPath(tagId);
            m_currentSetpointIndex = 0;
            m_autoComplete = m_setpoints.empty();
            ResetPidState();
            RoboClawResetAllEncoders();
            m_vertTicks  = 0;
            m_horizTicks = 0;
            m_waypointStartTime_s = m_timer.Get().value();
            m_autoPhase  = AutoPhase::APPROACH;
            frc::SmartDashboard::PutNumber("Auto/TagId", tagId);
        }
        else{
            //this is assuming the robot has the camera facing out the cave
            //we will drive straight until we can see the april tag
            //then when we see the april tag our new target will be the tag.distance - (comfortable distance away from tag to start bucket movement sequence)
            double x_target     = -30.0;
            double y_target     = -0.0;
            double theta_target = 0.0;
            double x_error     = x_target - x_pos;
            double y_error     = y_target - y_pos;
            double theta_error = WrapAngle(theta_target - theta_pos);
            x_integral     += x_error     * dt;
            y_integral     += y_error     * dt;
            theta_integral += theta_error * dt;
            double x_deriv     = (x_error     - x_prevError)     / dt;
            double y_deriv     = (y_error     - y_prevError)     / dt;
            double theta_deriv = (theta_error - theta_prevError) / dt;
            double abs_theta = std::abs(theta_error);
            double sched_kP  = 25.0;
            if (abs_theta > std::numbers::pi / 4)
                sched_kP = 80.0;
            else if (abs_theta > std::numbers::pi / 12)
                sched_kP = 40.0;

            double x_cmd     = x_kP     * x_error     + x_kI     * x_integral     + x_kD     * x_deriv;
            double y_cmd     = y_kP     * y_error     + y_kI     * y_integral     + y_kD     * y_deriv;
            double theta_cmd = sched_kP * theta_error + theta_kI * theta_integral + theta_kD * theta_deriv;
            if (std::abs(x_cmd)     > 127.0) { x_integral     -= x_error     * dt; x_cmd     = x_kP     * x_error     + x_kI     * x_integral     + x_kD     * x_deriv; }
            if (std::abs(y_cmd)     > 127.0) { y_integral     -= y_error     * dt; y_cmd     = y_kP     * y_error     + y_kI     * y_integral     + y_kD     * y_deriv; }
            if (std::abs(theta_cmd) >  80.0) { theta_integral -= theta_error * dt; theta_cmd = sched_kP * theta_error + theta_kI * theta_integral + theta_kD * theta_deriv; }
            x_prevError     = x_error;
            y_prevError     = y_error;
            theta_prevError = theta_error;
            constexpr double kXTol     = 0.002;
            constexpr double kYTol     = 0.002;
            constexpr double kThetaTol = 0.05; // ~3° — tight enough for accuracy, wide enough to settle
            if (std::abs(x_error) <= kXTol)
            {
                x_cmd = 0.0;
                x_integral = 0.0;
                x_prevError = 0.0;
                x_deriv = 0.0;
            }
            if (std::abs(y_error) <= kYTol)
            {
                y_cmd = 0.0;
                y_integral = 0.0;
                y_prevError = 0.0;
                y_deriv = 0.0;
            }
            if (std::abs(theta_error) <= kThetaTol)
            {
                theta_cmd = 0.0;
                theta_integral = 0.0;
                theta_prevError = 0.0;
                theta_deriv = 0.0;
            }

            bool x_done     = (std::abs(x_error)     <= kXTol);
            bool y_done     = (std::abs(y_error)     <= kYTol);
            bool theta_done = (std::abs(theta_error) <= kThetaTol);
            x_cmd     = std::clamp(x_cmd,     -127.0, 127.0);
            y_cmd     = std::clamp(y_cmd,     -127.0, 127.0);
            theta_cmd = std::clamp(theta_cmd,  -80.0,  80.0);

            // Minimum command (deadband kick) outside tolerance only
            if (theta_cmd > 0.0 && theta_cmd < 15.0) theta_cmd = 15.0;
            if (theta_cmd < 0.0 && theta_cmd > -15.0) theta_cmd = -15.0;
            if (y_cmd > 0.0 && y_cmd < 25.0) y_cmd = 25.0;
            if (y_cmd < 0.0 && y_cmd > -25.0) y_cmd = -25.0;

            // Differential mixing for drive wheels
            double xr_cmd = std::clamp(x_cmd + theta_cmd, -127.0, 127.0);
            double xl_cmd = std::clamp(x_cmd - theta_cmd, -127.0, 127.0);
            if (xr_cmd > 0.0 && xr_cmd < 25.0) xr_cmd = 25.0;
            if (xr_cmd < 0.0 && xr_cmd > -25.0) xr_cmd = -25.0;
            if (xl_cmd > 0.0 && xl_cmd < 25.0) xl_cmd = 25.0;
            if (xl_cmd < 0.0 && xl_cmd > -25.0) xl_cmd = -25.0;

            double spdr = std::abs(xr_cmd);
            double spdl = std::abs(xl_cmd);
            double spdy = std::abs(y_cmd);
             // Right drive (M1 on 0x80)
            if      (xr_cmd > 0.0) RoboClawM1Forward (kRoboClawAddr_Drive, static_cast<uint8_t>(spdr));
            else if (xr_cmd < 0.0) RoboClawM1Backward(kRoboClawAddr_Drive, static_cast<uint8_t>(spdr));
            else                   RoboClawM1Forward  (kRoboClawAddr_Drive, 0);

            // Left drive (M2 on 0x80)
            if      (xl_cmd > 0.0) RoboClawM2Forward (kRoboClawAddr_Drive, static_cast<uint8_t>(spdl));
            else if (xl_cmd < 0.0) RoboClawM2Backward(kRoboClawAddr_Drive, static_cast<uint8_t>(spdl));
            else                   RoboClawM2Forward  (kRoboClawAddr_Drive, 0);

            // Strafe (M1 on 0x81)
            if      (y_cmd > 0.0) RoboClawM1Forward (kRoboClawAddr_Strafe, static_cast<uint8_t>(spdy));
            else if (y_cmd < 0.0) RoboClawM1Backward(kRoboClawAddr_Strafe, static_cast<uint8_t>(spdy));
            else                  RoboClawM1Forward  (kRoboClawAddr_Strafe, 0);

        }

        break;
    }

    // ── CENTERING: strafe until AprilTag is centered in camera frame ─────
    // Non-blocking: each tick checks the pixel error and issues a single
    // DriveHorizontal command, then returns. No while loops.
    // When centered (or timed out), resets encoders + IMU and starts path.
    case AutoPhase::CENTERING: {
        frc::SmartDashboard::PutString("Auto/Phase", "Centering");

        // // Timeout safety: if we can't center in time, start the path anyway
        // if (m_timer.Get().value() >= kTagSearchTimeout_s) {
        //     StopAllDrive();
        //     RoboClawResetAllEncoders();
        //     m_thetaRad     = 0.0;
        //     m_yaw_rad      = 0.0;
        //     m_firstPidLoop = true;
        //     ResetPidState();
        //     m_autoPhase = AutoPhase::APPROACH;
        //     std::cout << "[Auto] Centering timed out — starting path from current position\n";
        //     break;
        // }
        //keep driving right until we see an april tag
        if(!hasTag){
            int8_t strafeSpeed = static_cast<int8_t>(kCenterSpeed);
            DriveHorizontal(strafeSpeed);
        }
        else{
            double pixelError = tag.x - kCameraCenter_px;
            frc::SmartDashboard::PutNumber("Auto/CenterError_px", pixelError);

            if (std::abs(pixelError) <= kCenterTolerance_px)
            {
                // Centered — lock lateral position, reset all odometry, start path
                StopAllDrive();
                RoboClawResetAllEncoders();
                m_thetaRad = 0.0;
                m_yaw_rad = 0.0;
                m_firstPidLoop = true;
                ResetPidState();
                frc::Rotation2d gyroAngle{units::radian_t{m_thetaRad}};
                ResetPoseEstimator(m_initialPose, 0_m, 0_m, gyroAngle);
                m_waypointStartTime_s = m_timer.Get().value();
                m_autoPhase = AutoPhase::APPROACH;
                std::cout << "[Auto] Centered (error=" << pixelError << "px) — starting path\n";
            }
            else
            {
                // Strafe toward tag center.
                // If pixelError > 0 (tag is RIGHT of center) → strafe right (+speed).
                // Flip sign here if the robot strafes the wrong way.
                int8_t strafeSpeed = (pixelError > 0) ? static_cast<int8_t>(kCenterSpeed)
                                                      : static_cast<int8_t>(-kCenterSpeed);
                DriveHorizontal(strafeSpeed);
            }
        }

        break;
    }

    // ── SEARCH: wait for a visible tag ───────────────────────────────────
    case AutoPhase::SEARCH:
        StopAllDrive();
        frc::SmartDashboard::PutString("Auto/Phase", "Search");

        if (hasTag) {
            // Seed the PID setpoint to the tag's current reported distance.
            // x_target = current x position + tag distance (drive toward it).
            // y_target = current y position (no lateral target yet).
            // theta_target = current heading (hold straight).
            // These will be updated live inside APPROACH every tick.
            m_waypointStartTime_s = m_timer.Get().value();
            m_autoPhase = AutoPhase::APPROACH;
            [[fallthrough]]; // start driving this same tick
        } else {
            break;
        }

    // ── APPROACH: PID-controlled drive toward tag ─────────────────────────
    case AutoPhase::APPROACH:
        // ── PID computation ───────────────────────────────────────────────
        // Setpoint: drive x forward by tag.distance, hold y, hold heading.
        // The tag distance is the live "how far until we stop" signal.
        {
            // Per-waypoint timeout: skip if stuck too long on one setpoint
            {
                double waypointElapsed = m_timer.Get().value() - m_waypointStartTime_s;
                if (waypointElapsed >= kWaypointTimeout_s)
                {
                    StopAllDrive();
                    std::cout << "[Auto] Waypoint " << m_currentSetpointIndex
                              << " timed out (" << waypointElapsed << "s) — skipping\n";
                    AdvanceToNextSetpoint();
                    break;
                }
            }
            // Use setpoints from the loaded list if they haven't all been
            // completed, otherwise fall back to a simple tag-distance target.
            // All waypoints done → stop and hold
            if (m_autoComplete || m_setpoints.empty()) {
                StopAllDrive();
                m_autoPhase = AutoPhase::DONE;
                frc::SmartDashboard::PutString("Auto/Phase", "Done (path complete)");
                break;
            }

            const auto& sp = m_setpoints[m_currentSetpointIndex];
            double x_target     = sp.x_trgt;
            double y_target     = sp.y_trgt;
            double theta_target = sp.theta_rad_trgt;

            //uncomment when we bring back april tag detection
            // } else {
            //     x_target     = x_pos + tag.distance;
            //     y_target     = y_pos;
            //     theta_target = m_thetaRad;
            // }

            double dx_field = x_target - x_pos;
            double dy_field = y_target - y_pos;

            double c = std::cos(theta_pos);
            double s = std::sin(theta_pos);

            double x_error =  c * dx_field + s * dy_field;   // robot forward/back
            double y_error = -s * dx_field + c * dy_field;   // robot lateral

            // Outer loop: lateral error creates a heading reference correction
            double theta_ref = theta_target + y_to_theta_kP * y_error;
            double theta_error = WrapAngle(theta_ref - theta_pos);

            // Integrate
            x_integral     += x_error     * dt;
            y_integral     += y_error     * dt;
            theta_integral += theta_error * dt;

            // Differentiate
            double x_deriv     = (x_error     - x_prevError)     / dt;
            double y_deriv     = (y_error     - y_prevError)     / dt;
            double theta_deriv = (theta_error - theta_prevError) / dt;

            // Gain scheduling for theta: larger P when heading error is large
            double abs_theta = std::abs(theta_error);
            double sched_kP  = 25.0;
            if (abs_theta > std::numbers::pi / 4)
                sched_kP = 80.0;
            else if (abs_theta > std::numbers::pi / 12)
                sched_kP = 40.0;

            double x_cmd     = x_kP     * x_error     + x_kI     * x_integral     + x_kD     * x_deriv;
            double y_cmd     = y_kP     * y_error     + y_kI     * y_integral     + y_kD     * y_deriv;
            double theta_cmd = sched_kP * theta_error + theta_kI * theta_integral + theta_kD * theta_deriv;

            // Anti-windup: only freeze the integral when the output is saturated.
            // If saturated, undo this tick's accumulation and recompute.
            if (std::abs(x_cmd)     > 127.0) { x_integral     -= x_error     * dt; x_cmd     = x_kP     * x_error     + x_kI     * x_integral     + x_kD     * x_deriv; }
            if (std::abs(y_cmd)     > 127.0) { y_integral     -= y_error     * dt; y_cmd     = y_kP     * y_error     + y_kI     * y_integral     + y_kD     * y_deriv; }
            if (std::abs(theta_cmd) >  80.0) { theta_integral -= theta_error * dt; theta_cmd = sched_kP * theta_error + theta_kI * theta_integral + theta_kD * theta_deriv; }

            y_cmd = 0.0;   
            y_integral = 0.0;
            y_prevError = 0.0;
            y_deriv = 0.0;

            // Save errors for next derivative calculation
            x_prevError     = x_error;
            y_prevError     = y_error;
            theta_prevError = theta_error;

            // Zero out axis when within tolerance (also prevents integral windup)
            constexpr double kXTol     = 0.002;
            constexpr double kYTol     = 0.002;
            constexpr double kThetaTol = 0.05; // ~3° — tight enough for accuracy, wide enough to settle
            if (std::abs(x_error) <= kXTol)
            {
                x_cmd = 0.0;
                x_integral = 0.0;
                x_prevError = 0.0;
                x_deriv = 0.0;
            }
            if (std::abs(y_error) <= kYTol)
            {
                y_cmd = 0.0;
                y_integral = 0.0;
                y_prevError = 0.0;
                y_deriv = 0.0;
            }
            if (std::abs(theta_error) <= kThetaTol)
            {
                theta_cmd = 0.0;
                theta_integral = 0.0;
                theta_prevError = 0.0;
                theta_deriv = 0.0;
            }

            bool x_done     = (std::abs(x_error)     <= kXTol);
            bool y_done = !m_enableYControl || (std::abs(y_error) <= kYTol);
            bool theta_done = (std::abs(theta_error) <= kThetaTol);

            if (x_done && y_done && theta_done) {
                RoboClawStopAll();
                ResetPidState();

                // ── Beacon deposit dwell (waypoint 1) ────────────────────────
                // Arm is already lowered from AutonomousInit. Hold position for
                // kServoDwell_s so the beacon settles, then raise arm and advance.
                if (m_currentSetpointIndex == 1) {
                    double now = m_timer.Get().value();
                    if (m_servoCommandTime.value() < 0) {
                        // First arrival — start dwell timer
                        m_servoCommandTime = units::second_t(now);
                        break;
                    }
                    if ((now - m_servoCommandTime.value()) < kServoDwell_s) {
                        break;  // still dwelling
                    }
                    // Done — raise arm, clear timer, fall through to advance
                    ArmRaise();
                    m_servoCommandTime = -1_s;
                    std::cout << "[Beacon] Deposit complete — arm raised\n";
                }

                // ── Ore deposit dwell (waypoint 9) ───────────────────────────
                // Non-blocking: start extend on first arrival, poll each tick,
                // retract when extend time elapses, advance when retract done.
                bool isDepositWaypoint = (m_currentSetpointIndex == 9);
                if (isDepositWaypoint) {
                    double now = m_timer.Get().value();
                    if (m_actuatorDwellStep == 0) {
                        // First arrival — start extending
                        ActuatorExtend(kActuatorSpeed);
                        m_actuatorDwellStart_s = now;
                        m_actuatorDwellStep = 1;
                        std::cout << "[Deposit] Extending actuator\n";
                        break;  // come back next tick
                    } else if (m_actuatorDwellStep == 1) {
                        // Waiting for full extension
                        if (now - m_actuatorDwellStart_s < kActuatorRunTime_s) {
                            break;  // still extending
                        }
                        ActuatorStop();
                        ActuatorRetract(kActuatorSpeed);
                        m_actuatorDwellStart_s = now;
                        m_actuatorDwellStep = 2;
                        std::cout << "[Deposit] Retracting actuator\n";
                        break;
                    } else if (m_actuatorDwellStep == 2) {
                        // Waiting for full retraction
                        if (now - m_actuatorDwellStart_s < kActuatorRunTime_s) {
                            break;  // still retracting
                        }
                        ActuatorStop();
                        m_actuatorDwellStep = 0;  // reset for next deposit
                        std::cout << "[Deposit] Done\n";
                        // fall through to advance
                    }
                }

                // ── Path / TAG_SEARCH handoff ─────────────────────────────────
                if (m_currentSetpointIndex == kTagHandoffWaypoint) {
                    m_timer.Reset();
                    m_timer.Start();
                    m_autoPhase = AutoPhase::TAG_SEARCH;
                } else {
                    AdvanceToNextSetpoint();
                }
                break;
            }

            // Saturate
            x_cmd     = std::clamp(x_cmd,     -127.0, 127.0);
            y_cmd     = std::clamp(y_cmd,     -127.0, 127.0);
            theta_cmd = std::clamp(theta_cmd,  -80.0,  80.0);

            // Minimum command (deadband kick) outside tolerance only
            if (theta_cmd > 0.0 && theta_cmd < 15.0) theta_cmd = 15.0;
            if (theta_cmd < 0.0 && theta_cmd > -15.0) theta_cmd = -15.0;
            if (y_cmd > 0.0 && y_cmd < 25.0) y_cmd = 25.0;
            if (y_cmd < 0.0 && y_cmd > -25.0) y_cmd = -25.0;

            if (!m_enableYControl) {
                y_cmd = 0.0;
                y_integral = 0.0;
                y_prevError = 0.0;
                y_deriv = 0.0;
            }

            // Differential mixing for drive wheels
            double xr_cmd = std::clamp(x_cmd + theta_cmd, -127.0, 127.0);
            double xl_cmd = std::clamp(x_cmd - theta_cmd, -127.0, 127.0);
            if (xr_cmd > 0.0 && xr_cmd < 25.0) xr_cmd = 25.0;
            if (xr_cmd < 0.0 && xr_cmd > -25.0) xr_cmd = -25.0;
            if (xl_cmd > 0.0 && xl_cmd < 25.0) xl_cmd = 25.0;
            if (xl_cmd < 0.0 && xl_cmd > -25.0) xl_cmd = -25.0;

            double spdr = std::abs(xr_cmd);
            double spdl = std::abs(xl_cmd);
            double spdy = std::abs(y_cmd);

            // Right drive (M1 on 0x80)
            if      (xr_cmd > 0.0) RoboClawM1Forward (kRoboClawAddr_Drive, static_cast<uint8_t>(spdr));
            else if (xr_cmd < 0.0) RoboClawM1Backward(kRoboClawAddr_Drive, static_cast<uint8_t>(spdr));
            else                   RoboClawM1Forward  (kRoboClawAddr_Drive, 0);

            // Left drive (M2 on 0x80)
            if      (xl_cmd > 0.0) RoboClawM2Forward (kRoboClawAddr_Drive, static_cast<uint8_t>(spdl));
            else if (xl_cmd < 0.0) RoboClawM2Backward(kRoboClawAddr_Drive, static_cast<uint8_t>(spdl));
            else                   RoboClawM2Forward  (kRoboClawAddr_Drive, 0);

            RoboClawM1Forward(kRoboClawAddr_Strafe, 0);

            // PID telemetry
            frc::SmartDashboard::PutNumber("PID/WaypointIndex",
                static_cast<double>(m_currentSetpointIndex));
            frc::SmartDashboard::PutNumber("PID/WaypointTotal",
                static_cast<double>(m_setpoints.size()));
            frc::SmartDashboard::PutNumber("PID/X_Target",    x_target);
            frc::SmartDashboard::PutNumber("PID/Y_Target",    y_target);
            frc::SmartDashboard::PutNumber("PID/Theta_Target",theta_target);
            frc::SmartDashboard::PutNumber("PID/X_Error",     x_error);
            frc::SmartDashboard::PutNumber("PID/Y_Error",     y_error);
            frc::SmartDashboard::PutNumber("PID/Theta_Error", theta_error);
            frc::SmartDashboard::PutNumber("PID/X_Cmd",       x_cmd);
            frc::SmartDashboard::PutNumber("PID/Y_Cmd",       y_cmd);
            frc::SmartDashboard::PutNumber("PID/Theta_Cmd",   theta_cmd);
            frc::SmartDashboard::PutNumber("PID/XR_Mixed",    xr_cmd);
            frc::SmartDashboard::PutNumber("PID/XL_Mixed",    xl_cmd);
        }

        frc::SmartDashboard::PutString("Auto/Phase", "Approach");
        frc::SmartDashboard::PutNumber("Auto/Tag_Distance_m", tag.distance);
        frc::SmartDashboard::PutNumber("Auto/Tag_X_px",       tag.x);
        break;
    }

    // ── 8. General dashboard ─────────────────────────────────────────────
    frc::SmartDashboard::PutNumber("Auto/ElapsedTime_s", m_timer.Get().value());
    frc::SmartDashboard::PutNumber("Auto/VertTicks",     m_vertTicks);
    frc::SmartDashboard::PutNumber("Auto/HorizTicks",    m_horizTicks);
}

// ============================================================================
//  Linear actuator helpers  (RoboClaw 0x81, M2)
// ============================================================================

// Extends the ore-deposit plate forward.
void Robot::ActuatorExtend(uint8_t speed) {
    RoboClawM2Forward(kRoboClawAddr_Strafe, speed);
}

// Retracts the ore-deposit plate back.
void Robot::ActuatorRetract(uint8_t speed) {
    RoboClawM2Backward(kRoboClawAddr_Strafe, speed);
}

// Stops the actuator in place.
void Robot::ActuatorStop() {
    RoboClawM2Forward(kRoboClawAddr_Strafe, 0);
}

// ============================================================================
//  Bucket / beacon / ore mechanisms
// ============================================================================

// GrabBucket – raise the arm to clear the bucket lip, then lower back to the
// init/clamped position to lock onto the bucket.
// The path is responsible for waiting between these two steps:
//   1. Call GrabBucket() → arm raises to ArmServoOpenPos (1200 μs)
//   2. Wait ~1 s for servo to travel
//   3. Call GrabBucket() again → arm lowers to ArmServoInitPos (500 μs)
void Robot::GrabBucket() {
    static bool raised = false;
    raised = !raised;
    int pw = raised ? ArmServoOpenPos : ArmServoInitPos;
    m_servoArm.SetPulseWidth(pw);
}

// ArmRaise – raise the arm to the beacon-drop angle.
// Call at autonomous start so the beacon clears the arena during early turns,
// and again just before the deposit waypoint.
// TODO: tune kArmServoBeaconPos (currently 1000 μs) for the right release angle.
void Robot::ArmRaise() {
    m_servoArm.SetPulseWidth(ArmServoOpenPos);
    std::cout << "[Mechanism] ArmRaise — arm up (" << ArmServoOpenPos << " μs)\n";
}

// ArmLower – return the arm to the init/resting position after deposit.
void Robot::ArmLower() {
    m_servoArm.SetPulseWidth(ArmServoInitPos);
    std::cout << "[Mechanism] ArmLower — arm down (" << ArmServoInitPos << " μs)\n";
}

// DepositOres – extend the linear actuator at full speed for kActuatorRunTime_s
// to push all ores out of the bucket.
// Non-blocking: starts the actuator and sets m_actuatorExtended = true.
// The path must call ActuatorStop() after kActuatorRunTime_s (5 s),
// then ActuatorRetract() + ActuatorStop() to home the plate.
void Robot::DepositOres() {
    ActuatorExtend(kActuatorSpeed);
    m_actuatorExtended = true;
    std::cout << "[Mechanism] DepositOres — actuator extending at speed " << (int)kActuatorSpeed << "\n";
}

// ============================================================================
//  TeleopInit / TeleopPeriodic
// ============================================================================

void Robot::TeleopInit() {
    StopAllDrive();
    std::cout << "[Teleop] TeleopInit — drive motors stopped\n";
}

void Robot::TeleopPeriodic() {}

// ============================================================================
//  Remaining mode stubs
// ============================================================================

void Robot::DisabledInit() {
    StopAllDrive();
    RoboClawDrain();
}

void Robot::DisabledPeriodic() {}


void Robot::TestInit() {
    m_testStep = 0;
    m_timer.Reset();
    m_timer.Start();
}


void Robot::TestPeriodic() {
    double t = m_timer.Get().value();

    switch (m_testStep)
    {
    case 0:
        ActuatorExtend(kActuatorSpeed);
        m_testStep = 1;
        break;
    case 1:
        if(t >= kActuatorRunTime_s){
            ActuatorStop();
            m_timer.Reset();
            m_timer.Start();
            m_testStep = 2;
        }
        break;
    case 2:
        ActuatorRetract(kActuatorSpeed);
        m_testStep = 3;
        break;
    case 3:
        if(t >= kActuatorRunTime_s){
            ActuatorStop();
            m_testStep = 4;
        }
        break;
    default:
        break;
    }
}


//test

void Robot::SimulationPeriodic() {}

// ============================================================================
//  Entry point
// ============================================================================

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif