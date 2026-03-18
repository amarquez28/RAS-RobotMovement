// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// ============================================================================
//  Robot.cpp  –  RAS 2026  |  IEEE Region 5 Robotics Competition
// ============================================================================
//
//  Autonomous behaviour:
//    1. Vision system (Pi) detects start light → enables robot via NT.
//    2. Robot performs a boustrophedon ("lawnmower") sweep over 4 ft × 8 ft.
//    3. Intake runs continuously throughout the sweep.
//    4. IMU gyro-Z is integrated each tick to track heading; a small strafe
//       correction is applied when the robot drifts off its straight-line leg.
//    5. Encoder dead-reckoning (RoboClaw 0x80 M1 for vertical, 0x81 M1 for
//       horizontal) measures distance traveled per leg and strafe width.
//    6. AprilTag data from the Raspberry Pi is read every tick; if a tag is
//       spotted the sweep logs it and finishes the current strip before acting.
//
//  Teleop:
//    Currently a safety stop (all motors off).  Extend as needed.
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

// ── Constructor ──────────────────────────────────────────────────────────────

Robot::Robot() : frc::TimesliceRobot{5_ms, 10_ms} {
    // LiveWindow causes drastic overruns in robot periodic functions
    frc::LiveWindow::DisableAllTelemetry();

    // Timeslice scheduling: 5 ms robot periodic + 2 ms + 2 ms = 9 ms / 10 ms
    Schedule([=] {}, 2_ms);
    Schedule([=] {}, 2_ms);

    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

    // Power and enable all four servo channels
    m_servo0.SetPowered(true);  m_servo0.SetEnabled(true);
    m_servo1.SetPowered(true);  m_servo1.SetEnabled(true);
    m_servo2.SetPowered(true);  m_servo2.SetEnabled(true);
    m_servo3.SetPowered(true);  m_servo3.SetEnabled(true);

    // Initialise IMU
    IMUInit();

    // Clean RoboClaw buffers and stop all motors on boot
    m_roboclaw.Reset();
    RoboClawStopAll();
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
    uint8_t pkt[3]  = {addr, cmd, value};
    uint16_t crc    = RoboClawCRC16(pkt, 3);
    uint8_t out[5]  = {
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
// Returns a signed int32 (RoboClaw raw value reinterpreted as signed).
bool Robot::RoboClawReadEncoder(uint8_t addr, uint8_t cmd,
                                 int32_t& count, uint8_t& status) {
    RoboClawDrain();

    uint8_t req[2]  = {addr, cmd};
    uint16_t reqCrc = RoboClawCRC16(req, 2);
    uint8_t out[4]  = {
        addr, cmd,
        static_cast<uint8_t>((reqCrc >> 8) & 0xFF),
        static_cast<uint8_t>(reqCrc & 0xFF)
    };
    m_roboclaw.Write(reinterpret_cast<const char*>(out), 4);

    // Response: 4-byte count  +  1-byte status  +  2-byte CRC  = 7 bytes
    uint8_t resp[7];
    if (!ReadExact(m_roboclaw, resp, 7, 20_ms)) return false;

    // Parse count (big-endian), reinterpret as signed
    uint32_t raw = (uint32_t(resp[0]) << 24) | (uint32_t(resp[1]) << 16) |
                   (uint32_t(resp[2]) <<  8) | (uint32_t(resp[3]) <<  0);
    count  = static_cast<int32_t>(raw);
    status = resp[4];

    // Verify CRC covers [addr, cmd, resp[0..4]]
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
//  High-level drive helpers  (used by Robot.h public API and SweepController)
// ============================================================================

// DriveVertical: positive speed = forward (intake direction)
// Drives RoboClaw 0x80 M1 (left) and M2 (right) together.
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

// DriveHorizontal: positive speed = strafe right
// Drives RoboClaw 0x81 M1 only.
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

// ============================================================================
//  IMU (MPU-6050) helpers
// ============================================================================

bool Robot::IMUWriteReg(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    return m_imu.WriteBulk(buf, 2); // false = success in WPILib I2C
}

bool Robot::IMUReadRegs(uint8_t startReg, uint8_t* out, int len) {
    if (m_imu.WriteBulk(&startReg, 1)) return true; // error
    return m_imu.ReadOnly(len, out);                 // true = error
}

void Robot::IMUInit() {
    IMUWriteReg(0x6B, 0x00);  // PWR_MGMT_1: wake up
    IMUWriteReg(0x1B, 0x00);  // GYRO_CONFIG: ±250 dps full scale
    IMUWriteReg(0x1C, 0x00);  // ACCEL_CONFIG: ±2 g full scale
    m_lastIMUTime = frc::Timer::GetFPGATimestamp();
}

// IMUUpdate() – integrate gyro Z to track yaw heading.
// Call this every RobotPeriodic() tick so the timing stays consistent.
void Robot::IMUUpdate() {
    uint8_t data[14] = {0};
    if (IMUReadRegs(0x3B, data, 14)) return; // read error → skip

    // Gyro Z is bytes [12, 13] in the 14-byte burst starting at 0x3B
    int16_t gz_raw = static_cast<int16_t>((data[12] << 8) | data[13]);
    double  gz_dps = gz_raw / 131.0; // ±250 dps → 131 LSB/(°/s)

    auto now = frc::Timer::GetFPGATimestamp();
    double dt = (now - m_lastIMUTime).value(); // seconds
    m_lastIMUTime = now;

    // Clamp dt to avoid huge integration jumps on first call or stalls
    if (dt > 0.05) dt = 0.05;

    // Trapezoidal integration (average of previous and current sample)
    m_yaw_deg += ((m_lastGyroZ_dps + gz_dps) / 2.0) * dt;
    m_lastGyroZ_dps = gz_dps;
}

// IMUDashboard() – push all raw and converted IMU data to SmartDashboard
void Robot::IMUDashboard() {
    uint8_t data[14] = {0};
    bool err = IMUReadRegs(0x3B, data, 14);
    frc::SmartDashboard::PutBoolean("IMU/ReadError", err);
    if (err) return;

    int16_t ax = static_cast<int16_t>((data[0]  << 8) | data[1]);
    int16_t ay = static_cast<int16_t>((data[2]  << 8) | data[3]);
    int16_t az = static_cast<int16_t>((data[4]  << 8) | data[5]);
    int16_t gx = static_cast<int16_t>((data[8]  << 8) | data[9]);
    int16_t gy = static_cast<int16_t>((data[10] << 8) | data[11]);
    int16_t gz = static_cast<int16_t>((data[12] << 8) | data[13]);

    frc::SmartDashboard::PutNumber("IMU/Accel_g_X",   ax / 16384.0);
    frc::SmartDashboard::PutNumber("IMU/Accel_g_Y",   ay / 16384.0);
    frc::SmartDashboard::PutNumber("IMU/Accel_g_Z",   az / 16384.0);
    frc::SmartDashboard::PutNumber("IMU/Gyro_dps_X",  gx / 131.0);
    frc::SmartDashboard::PutNumber("IMU/Gyro_dps_Y",  gy / 131.0);
    frc::SmartDashboard::PutNumber("IMU/Gyro_dps_Z",  gz / 131.0);
    frc::SmartDashboard::PutNumber("IMU/Yaw_deg",     m_yaw_deg);
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
//  RobotPeriodic  – runs every packet regardless of mode
// ============================================================================

void Robot::RobotPeriodic() {
    // Integrate IMU every tick for consistent dt
    IMUUpdate();

    // AprilTag dashboard (vision system health + tag data)
    m_aprilTagReader.UpdateDashboard();

    // Hall sensor → hatch door servo
    if (!m_hallDigital.Get()) {
        m_servo0.SetPulseWidth(kHallServoOpenPos);
    } else {
        m_servo0.SetPulseWidth(kHallServoInitPos);
    }

    // Dashboard: servo + hall sensor
    frc::SmartDashboard::PutNumber ("Servo0/PulseWidth", m_servo0.GetPulseWidth());
    frc::SmartDashboard::PutNumber ("Hall/Voltage_V",    m_hallAnalog.GetVoltage());
    frc::SmartDashboard::PutNumber ("Hall/Raw",          m_hallAnalog.GetValue());
    frc::SmartDashboard::PutBoolean("Hall/Digital",      m_hallDigital.Get());

    // IMU dashboard (non-integrated values for debugging)
    IMUDashboard();
}

// ============================================================================
//  AutonomousInit
// ============================================================================

void Robot::AutonomousInit() {
    m_timer.Reset();
    m_timer.Start();
    RoboClawDrain();

    // Reset heading integrator at the start of every auto run
    m_yaw_deg       = 0.0;
    m_lastGyroZ_dps = 0.0;
    m_lastIMUTime   = frc::Timer::GetFPGATimestamp();

    // Re-init IMU in case of power cycle
    IMUInit();

    // Snapshot encoders BEFORE we start moving so the sweep gets
    // relative distances (not absolute counts from power-on)
    UpdateEncoders();
    m_vertTicks  = 0;
    m_horizTicks = 0;

    // Prepare sweep state machine
    m_sweep.Reset();
    m_sweepStarted = false;

    // Close hatch door
    m_servo0.SetPulseWidth(kHallServoInitPos);

    // Safety stop all drive motors
    RoboClawStopAll();

    std::cout << "[Auto] AutonomousInit complete\n";
}

// ============================================================================
//  AutonomousPeriodic
// ============================================================================

void Robot::AutonomousPeriodic() {
    // ── 1. Update sensors ─────────────────────────────────────────────────
    UpdateEncoders();

    // Read the most recent AprilTag (if any)
    std::optional<AprilTagData> tag = std::nullopt;
    if (m_aprilTagReader.HasTarget()) {
        tag = m_aprilTagReader.GetPrimaryTag();
    }

    // ── 2. Log vision connection status ───────────────────────────────────
    // NOTE: IsConnected() compares heartbeat values between calls.
    // Moved here (not inside a conditional after a return) so it always runs.
    bool visionConnected = m_aprilTagReader.IsConnected();
    frc::SmartDashboard::PutBoolean("Vision/PiConnected", visionConnected);
    if (visionConnected) {
        std::cout << "Vision system connected\n";
    }

    // ── 3. Start sweep on first periodic tick ─────────────────────────────
    // The Pi enables the robot via the DS protocol, so by the time
    // AutonomousPeriodic() runs we are already enabled.
    if (!m_sweepStarted && m_encodersValid) {
        m_sweep.Start(m_vertTicks, m_horizTicks, m_yaw_deg);
        m_sweepStarted = true;
        std::cout << "[Auto] Sweep started\n";
    }

    if (!m_sweepStarted) {
        // Encoders not yet valid — stay stopped
        StopAllDrive();
        return;
    }

    // ── 4. Run sweep state machine ────────────────────────────────────────
    SweepController::DriveCommand cmd =
        m_sweep.Update(m_vertTicks, m_horizTicks, m_yaw_deg, tag);

    // ── 5. Execute drive command ──────────────────────────────────────────
    DriveVertical  (cmd.vertical);
    DriveHorizontal(cmd.horizontal);

    // ── 6. Handle sweep completion ────────────────────────────────────────
    if (cmd.done) {
        StopAllDrive();

        // If we spotted a tag during the sweep, report it
        if (m_sweep.TagWasSeen()) {
            std::cout << "[Auto] Sweep complete. AprilTag ID "
                      << m_sweep.LastTagId()
                      << " was detected during sweep.\n";
        } else {
            std::cout << "[Auto] Sweep complete. No AprilTags detected.\n";
        }
    }

    // ── 7. Dashboard summary ──────────────────────────────────────────────
    frc::SmartDashboard::PutNumber("Auto/ElapsedTime_s", m_timer.Get().value());
    frc::SmartDashboard::PutNumber("Auto/Yaw_deg",       m_yaw_deg);
    frc::SmartDashboard::PutNumber("Auto/VertTicks",     m_vertTicks);
    frc::SmartDashboard::PutNumber("Auto/HorizTicks",    m_horizTicks);
}

// ============================================================================
//  TeleopInit / TeleopPeriodic
//  BUG FIX: Previously empty — motors could be left running from auto.
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