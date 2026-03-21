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
    IMUWriteReg(0x6B, 0x00); // PWR_MGMT_1: wake up
    IMUWriteReg(0x1B, 0x00); // GYRO_CONFIG: ±250 dps full scale
    IMUWriteReg(0x1C, 0x00); // ACCEL_CONFIG: ±2 g full scale
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

// IMUDashboard() – push all raw and converted IMU data to SmartDashboard
void Robot::IMUDashboard() {
    uint8_t data[14] = {0};
    bool err = IMUReadRegs(0x3B, data, 14);
    frc::SmartDashboard::PutBoolean("IMU/ReadError", err);
    if (err) return;

    int16_t ax = ToInt16(data[0],  data[1]);
    int16_t ay = ToInt16(data[2],  data[3]);
    int16_t az = ToInt16(data[4],  data[5]);
    int16_t gx = ToInt16(data[8],  data[9]);
    int16_t gy = ToInt16(data[10], data[11]);
    int16_t gz = ToInt16(data[12], data[13]);

    frc::SmartDashboard::PutNumber("IMU/Accel_g_X",    ax / 16384.0);
    frc::SmartDashboard::PutNumber("IMU/Accel_g_Y",    ay / 16384.0);
    frc::SmartDashboard::PutNumber("IMU/Accel_g_Z",    az / 16384.0);
    frc::SmartDashboard::PutNumber("IMU/Gyro_radps_X", (gx / 131.0) * (std::numbers::pi / 180.0));
    frc::SmartDashboard::PutNumber("IMU/Gyro_radps_Y", (gy / 131.0) * (std::numbers::pi / 180.0));
    frc::SmartDashboard::PutNumber("IMU/Gyro_radps_Z", (gz / 131.0) * (std::numbers::pi / 180.0));
    frc::SmartDashboard::PutNumber("IMU/Yaw_rad",      m_yaw_rad);
    frc::SmartDashboard::PutNumber("IMU/Theta_rad",    m_thetaRad);
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
    m_setpoints = {
        {0.20, 0.0, 0.0},
        {0.20, 0.0, std::numbers::pi / 2.0},
        {0.40, 0.0, std::numbers::pi / 2.0}
    };
    m_currentSetpointIndex = 0;
    m_autoComplete = m_setpoints.empty();
}

void Robot::AdvanceToNextSetpoint() {
    ResetPidState();
    if (m_currentSetpointIndex + 1 < m_setpoints.size()) {
        m_currentSetpointIndex++;
    } else {
        m_autoComplete = true;
    }
}

void Robot::ResetPidState() {
    x_integral     = 0.0;
    y_integral     = 0.0;
    theta_integral = 0.0;

    x_prevError     = 0.0;
    y_prevError     = 0.0;
    theta_prevError = 0.0;
}

// ============================================================================
//  RobotPeriodic  – runs every packet regardless of mode
// ============================================================================

void Robot::RobotPeriodic() {
    // Single IMU I2C read per tick – updates m_thetaRad and m_yaw_rad
    IMUUpdate();

    // AprilTag dashboard (vision system health + tag data)
    m_aprilTagReader.UpdateDashboard();

    // Hall sensor → hatch door servo
    if (!m_hallDigital.Get()) {
        m_servoHall.SetPulseWidth(kHallServoOpenPos);
    } else {
        m_servoHall.SetPulseWidth(kHallServoInitPos);
    }

    // Dashboard: hall sensor
    frc::SmartDashboard::PutNumber ("Hall/Voltage_V", m_hallAnalog.GetVoltage());
    frc::SmartDashboard::PutNumber ("Hall/Raw",       m_hallAnalog.GetValue());
    frc::SmartDashboard::PutBoolean("Hall/Digital",   m_hallDigital.Get());

    // IMU dashboard (non-integrated values for debugging)
    IMUDashboard();
}

// ============================================================================
//  AutonomousInit
// ============================================================================

void Robot::AutonomousInit() {
    m_timer.Reset();
    m_timer.Start();

    // Reset all encoders to zero so dead-reckoning is relative to start pos
    RoboClawResetAllEncoders();
    RoboClawDrain();

    // Re-init IMU in case of a power cycle between runs
    IMUInit();

    // Zero all yaw integrators – must happen before CalibrateGyroZBias
    m_yaw_rad          = 0.0;
    m_thetaRad         = 0.0;
    m_lastGyroZ_radps  = 0.0;
    m_lastIMUTime      = frc::Timer::GetFPGATimestamp();

    // Calibrate gyro bias (robot must be stationary for ~1 second)
    CalibrateGyroZBias();

    // m_thetaRad is already zeroed above; reset the PID first-loop flag
    m_firstPidLoop = true;

    // Load the setpoint sequence for the PID approach phase
    LoadAutonomousSetpoints();

    // Reset approach state machine
    m_autoPhase = AutoPhase::SEARCH;
    m_taskDone  = false;

    // Clear NT completion flags so the Pi sees the reset
    m_taskDonePub.Set(false);
    m_sweepDonePub.Set(false);

    // Reset PID integrators
    ResetPidState();

    // Snapshot encoders then zero them (UpdateEncoders fills the members;
    // we then overwrite with 0 so sweep distances are relative)
    UpdateEncoders();
    m_vertTicks  = 0;
    m_horizTicks = 0;

    // Close hatch door
    m_servoHall.SetPulseWidth(kHallServoInitPos);

    // Safety stop
    RoboClawStopAll();

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
    // ── 1. Update sensors ──────────────────────────────────────────────────
    UpdateEncoders();

    // ── 2. Vision connection heartbeat ────────────────────────────────────
    // IsConnected() must be called every tick (it compares heartbeat counters).
    bool visionConnected = m_aprilTagReader.IsConnected();
    frc::SmartDashboard::PutBoolean("Vision/PiConnected", visionConnected);

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

    double xr_pos = e80_m1 * kXMetPerPul; // right drive wheel
    double xl_pos = e80_m2 * kXMetPerPul; // left  drive wheel
    double y_pos  = e81_m1 * kYMetPerPul; // strafe
    double x_pos  = (xr_pos + xl_pos) / 2.0;

    frc::SmartDashboard::PutNumber("Pose/X_m",     x_pos);
    frc::SmartDashboard::PutNumber("Pose/Y_m",     y_pos);
    frc::SmartDashboard::PutNumber("Pose/Theta_rad", m_thetaRad);

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
    // ── DONE: hold position ───────────────────────────────────────────────
    case AutoPhase::DONE:
        StopAllDrive();
        frc::SmartDashboard::PutString("Auto/Phase", "Done");
        break;

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
            m_autoPhase = AutoPhase::APPROACH;
            [[fallthrough]]; // start driving this same tick
        } else {
            break;
        }

    // ── APPROACH: PID-controlled drive toward tag ─────────────────────────
    case AutoPhase::APPROACH:
        // Safety: tag lost mid-approach → stop and re-search
        if (!hasTag) {
            StopAllDrive();
            m_autoPhase = AutoPhase::SEARCH;
            frc::SmartDashboard::PutString("Auto/Phase", "Search (tag lost)");
            break;
        }

        // Stop condition
        if (tag.distance <= kStopDistance_m) {
            StopAllDrive();
            m_taskDonePub.Set(true);
            m_sweepDonePub.Set(true); // legacy key for Pi compatibility
            m_taskDone  = true;
            m_autoPhase = AutoPhase::DONE;
            frc::SmartDashboard::PutString("Auto/Phase", "Done");
            break;
        }

        // ── PID computation ───────────────────────────────────────────────
        // Setpoint: drive x forward by tag.distance, hold y, hold heading.
        // The tag distance is the live "how far until we stop" signal.
        {
            // Use setpoints from the loaded list if they haven't all been
            // completed, otherwise fall back to a simple tag-distance target.
            double x_target, y_target, theta_target;
            if (!m_autoComplete && !m_setpoints.empty()) {
                const auto& sp = m_setpoints[m_currentSetpointIndex];
                x_target     = sp.x_trgt;
                y_target     = sp.y_trgt;
                theta_target = sp.theta_rad_trgt;
            } else {
                // Simple fallback: drive forward by tag distance, hold current y/theta
                x_target     = x_pos + tag.distance;
                y_target     = y_pos;
                theta_target = m_thetaRad;
            }

            double x_error     = x_target - x_pos;
            double y_error     = y_target - y_pos;
            double theta_error = WrapAngle(theta_target - m_thetaRad);

            // Integrate
            x_integral     += x_error     * dt;
            y_integral     += y_error     * dt;
            theta_integral += theta_error * dt;
            x_integral      = std::clamp(x_integral,     -10.0, 10.0);
            y_integral      = std::clamp(y_integral,     -10.0, 10.0);
            theta_integral  = std::clamp(theta_integral, -10.0, 10.0);

            // Differentiate
            double x_deriv     = (x_error     - x_prevError)     / dt;
            double y_deriv     = (y_error     - y_prevError)     / dt;
            double theta_deriv = (theta_error - theta_prevError) / dt;

            // Gain scheduling for theta: larger P when heading error is large
            double abs_theta = std::abs(theta_error);
            double sched_kP  = 40.0;
            if      (abs_theta > std::numbers::pi / 4)  sched_kP = 80.0;
            else if (abs_theta > std::numbers::pi / 12) sched_kP = 60.0;

            double x_cmd     = x_kP     * x_error     + x_kI     * x_integral     + x_kD     * x_deriv;
            double y_cmd     = y_kP     * y_error     + y_kI     * y_integral     + y_kD     * y_deriv;
            double theta_cmd = sched_kP * theta_error + theta_kI * theta_integral + theta_kD * theta_deriv;

            // Save errors for next derivative calculation
            x_prevError     = x_error;
            y_prevError     = y_error;
            theta_prevError = theta_error;

            // Zero out axis when within tolerance (also prevents integral windup)
            constexpr double kXTol     = 0.005;
            constexpr double kYTol     = 0.005;
            constexpr double kThetaTol = 0.02;
            if (std::abs(x_error)     <= kXTol)     { x_cmd     = 0.0; x_integral     = 0.0; }
            if (std::abs(y_error)     <= kYTol)      { y_cmd     = 0.0; y_integral     = 0.0; }
            if (std::abs(theta_error) <= kThetaTol) { theta_cmd = 0.0; theta_integral = 0.0; }

            bool x_done     = (std::abs(x_error)     <= kXTol);
            bool y_done     = (std::abs(y_error)     <= kYTol);
            bool theta_done = (std::abs(theta_error) <= kThetaTol);

            if (x_done && y_done && theta_done) {
                AdvanceToNextSetpoint();
                RoboClawStopAll();
                break;
            }

            // Saturate
            x_cmd     = std::clamp(x_cmd,     -127.0, 127.0);
            y_cmd     = std::clamp(y_cmd,     -127.0, 127.0);
            theta_cmd = std::clamp(theta_cmd,  -80.0,  80.0);

            // Minimum command (deadband kick) outside tolerance only
            if (theta_cmd > 0.0 && theta_cmd < 25.0) theta_cmd = 25.0;
            if (theta_cmd < 0.0 && theta_cmd > -25.0) theta_cmd = -25.0;
            if (y_cmd > 0.0 && y_cmd < 15.0) y_cmd = 15.0;
            if (y_cmd < 0.0 && y_cmd > -15.0) y_cmd = -15.0;

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

            // PID telemetry
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

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationPeriodic() {}

// ============================================================================
//  Entry point
// ============================================================================

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif