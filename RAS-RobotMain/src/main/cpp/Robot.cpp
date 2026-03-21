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
#include <numbers>

// ── Constructor ──────────────────────────────────────────────────────────────

Robot::Robot() : frc::TimesliceRobot{5_ms, 10_ms} {
    // LiveWindow causes drastic overruns in robot periodic functions
    frc::LiveWindow::DisableAllTelemetry();

    // Timeslice scheduling: 5 ms robot periodic + 2 ms + 2 ms = 9 ms / 10 ms
    Schedule([=] {}, 2_ms);
    Schedule([=] {}, 2_ms);

    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

    // Power and enable all four servo channels
    m_servoBrush.SetPowered(true);  m_servoBrush.SetEnabled(true);
    m_servoRelease.SetPowered(true);  m_servoRelease.SetEnabled(true);
    m_servoHall.SetPowered(true);  m_servoHall.SetEnabled(true);
    m_servoArm.SetPowered(true);  m_servoArm.SetEnabled(true);

    // Initialise IMU
    IMUInit();

    // Clean RoboClaw buffers and stop all motors on boot
    m_roboclaw.Reset();
    RoboClawStopAll();

    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("Vision");
    m_sweepDonePub = table->GetBooleanTopic("sweep_done").Publish();
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

//Roboclaw encoder reset for testing
void Robot::RoboClawResetEncoder(uint8_t addr) {
  uint8_t pkt[2] = {addr, 20}; // command 20 = Reset Encoders
  uint16_t crc = RoboClawCRC16(pkt, 2);

  uint8_t out[4] = {
    addr,
    20,
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
//  DriveTankSteered  –  differential steering during tag approach
// ============================================================================
//
//  Because the wheels cannot strafe while moving, we steer by giving the two
//  drive motors slightly different speeds (tank/differential drive).
//
//  Parameters:
//    baseSpeed  : signed speed for both wheels before correction.
//                 Will be NEGATIVE during tag approach (robot backs toward tag).
//    pixelError : tag.x − kCameraCenter_px
//                 Positive = tag is right of the camera's optical center.
//                 Negative = tag is left.
//
//  Why the sign works out automatically for the rear-facing camera:
//    When we drive backward (baseSpeed < 0):
//      pixelError > 0  →  tag is right in the image
//                      →  robot's rear end is drifting LEFT
//                      →  we need to swing the rear RIGHT
//                      →  slow the LEFT motor (M1), keep RIGHT (M2)
//      The correction term is (base * normError).
//      base = negative, normError = positive → correction is negative.
//      leftSpeed  = base + (negative correction) → |leftSpeed| decreases ✓
//      rightSpeed = base - (negative correction) → |rightSpeed| increases ✓
//    The opposite holds when pixelError < 0. No explicit sign flip needed.
//
//  kSteerFactor (0.0–1.0): fraction of base speed applied as differential.
//    0.3 means the inner wheel slows by 30% of base at maximum pixel error.
//    Start low and tune up if the robot is not correcting fast enough.
//
void Robot::DriveTankSteered(int8_t baseSpeed, double pixelError){
    // Normalise error  to [-1, 1] clamped at one tolerance width
    double normError = pixelError / kCenterTolerance_px;
    if(normError > 1.0) normError = 1.0;
    if(normError < -1.0) normError = -1.0;

    double base = static_cast<double>(baseSpeed);
    double leftSpeed = base + base * normError * kSteerFactor;
    double rightSpeed = base - base * normError * kSteerFactor;

    //clamp to valid roboclaw range [-127, 127]
    auto clamp127 = [](double v) -> int8_t{
        if(v > 127.0) v = 127.0;
        if(v < -127.0) v = -127.0;
        return static_cast<int8_t>(v);
    };
    int8_t l = clamp127(leftSpeed);
    int8_t r = clamp127(rightSpeed);

    frc::SmartDashboard::PutNumber("Drive/LeftSpeed",  l);
    frc::SmartDashboard::PutNumber("Drive/RightSpeed", r);

    //M1 = left motor, M2 = right motor on roboclaw 0x80
    if(l >= 0){
        RoboClawM1Forward(kRoboClawAddr_Drive, static_cast<uint8_t>(l));
    }
    else{
        RoboClawM1Backward(kRoboClawAddr_Drive, static_cast<uint8_t>(-l));
    }
    if(r >= 0){
        RoboClawM2Forward(kRoboClawAddr_Drive, static_cast<uint8_t>(r));
    }
    else{
        RoboClawM2Backward(kRoboClawAddr_Drive, static_cast<uint8_t>(-r));
    }
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

    //TODO: convert this into radians since thats what we are using
    // Trapezoidal integration (average of previous and current sample)
    //m_yaw_deg += ((m_lastGyroZ_dps + gz_dps) / 2.0) * dt;
    // m_lastGyroZ_dps = gz_dps;
}
//TOD0: Merge the two functions ↑ & ↓
static bool ReadIMU(double& gz_radps_out) {
  uint8_t data[14] = {0};
  bool readErr = ReadRegs(0x3B, data, 14);
  if (readErr) return false;

  int16_t gz = ToInt16(data[12], data[13]);

  double gz_dps = gz / 131.0;
  double gz_radps = gz_dps * std::numbers::pi / 180.0;

  gz_radps_out = gz_radps;
  return true;
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
    // frc::SmartDashboard::PutNumber("IMU/Yaw_deg",     m_yaw_deg);
}

// Convert two bytes (big-endian) to signed 16-bit
static int16_t ToInt16(uint8_t hi, uint8_t lo) {
  return static_cast<int16_t>((hi << 8) | lo);
}

double Robot::WrapAngle(double angle) {
  while (angle > std::numbers::pi) {
    angle -= 2.0 * std::numbers::pi;
  }
  while (angle < -std::numbers::pi) {
    angle += 2.0 * std::numbers::pi;
  }
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
  x_integral = 0.0;
  y_integral = 0.0;
  theta_integral = 0.0;

  x_prevError = 0.0;
  y_prevError = 0.0;
  theta_prevError = 0.0;
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
        m_servoBrush.SetPulseWidth(kHallServoOpenPos);
    } else {
        m_servoBrush.SetPulseWidth(kHallServoInitPos);
    }

    // Dashboard: servo + hall sensor
    frc::SmartDashboard::PutNumber ("Servo0/PulseWidth", m_servoBrush.GetPulseWidth());
    frc::SmartDashboard::PutNumber ("Hall/Voltage_V",    m_hallAnalog.GetVoltage());
    frc::SmartDashboard::PutNumber ("Hall/Raw",          m_hallAnalog.GetValue());
    frc::SmartDashboard::PutBoolean("Hall/Digital",      m_hallDigital.Get());

    // IMU dashboard (non-integrated values for debugging)
    IMUDashboard();
}

void Robot::CalibrateGyroZBias() {
  constexpr int kSamples = 500;

  double sum = 0.0;
  int count = 0;

  for (int i = 0; i < kSamples; i++) {
    double gz_radps = 0.0;

    if (ReadIMU(gz_radps)) {
      sum += gz_radps;
      count++;
    }

    frc::Wait(0.002_s);
  }

  if (count > 0) {
    m_gyroZBiasRadps = sum / count;
  }

  frc::SmartDashboard::PutNumber(
    "IMU_Calibrated_GyroZBias_radps", m_gyroZBiasRadps);
}

// ============================================================================
//  AutonomousInit
// ============================================================================

void Robot::AutonomousInit() {
    m_timer.Reset();
    m_timer.Start();
    RoboClawResetAllEncoders();
    RoboClawDrain();
    CalibrateGyroZBias();
    LoadAutonomousSetpoints();

    // TODO: again, use radians not degrees
    // Reset heading integrator at the start of every auto run
    // m_yaw_deg       = 0.0;
    // m_lastGyroZ_dps = 0.0;
    m_lastIMUTime   = frc::Timer::GetFPGATimestamp();

    // Re-init IMU in case of power cycle
    IMUInit();

    //reset approach state
    m_autoPhase = AutoPhase::SEARCH;
    m_taskDone = false;

    //clear nt completion flags so the pi sees the reset
    m_taskDonePub.Set(false);
    m_sweepDonePub.Set(false);

    // Snapshot encoders BEFORE we start moving so the sweep gets
    // relative distances (not absolute counts from power-on)
    UpdateEncoders();
    m_vertTicks  = 0;
    m_horizTicks = 0;

    // Close hatch door
    m_servoBrush.SetPulseWidth(kHallServoInitPos);

    // Safety stop all drive motors
    RoboClawStopAll();

    std::cout << "[Auto] AutonomousInit complete\n";
}

// ============================================================================
//  AutonomousPeriodic
//
//  State machine:
//
//   SEARCH ──(tag detected)──► APPROACH ──(distance ≤ 0.5 m)──► DONE
//
//  In APPROACH, every tick:
//    • DriveVertical(kApproachSpeed)  – move forward
//    • Compute pixel error: tagX − kCameraCenter_px
//        - If error > +kCenterTolerance_px → strafe right  (+kCenterSpeed)
//        - If error < -kCenterTolerance_px → strafe left   (-kCenterSpeed)
//        - Otherwise → strafe 0  (already centered)
//
//  If we lose the tag mid-approach (HasTarget() goes false), we stop and
//  fall back to SEARCH so we don't blindly drive off course.
//
// ============================================================================

void Robot::AutonomousPeriodic() {
    // ── Update sensors ─────────────────────────────────────────────────
    UpdateEncoders();

    // ── Log vision connection status ───────────────────────────────────
    // NOTE: IsConnected() compares heartbeat values between calls.
    // Moved here (not inside a conditional after a return) so it always runs.
    bool visionConnected = m_aprilTagReader.IsConnected();
    frc::SmartDashboard::PutBoolean("Vision/PiConnected", visionConnected);


    units::second_t now = frc::Timer::GetFPGATimestamp();
    double dt = 0.0;
    if (m_firstPidLoop)
    {
        m_prevTime = now;
        m_firstPidLoop = false;
        return; // skip one cycle so dt is valid next time
    }
    dt = (now - m_prevTime).value();
    m_prevTime = now;
    if (dt <= 1e-4 || dt > 0.1)
    {
        dt = 0.02; // fallback
    }

    // IMU Values
    double gz_radps;
    bool ok = ReadIMU(gz_radps);
    double omega_z = gz_radps - m_gyroZBiasRadps;
    m_thetaRad += omega_z * dt;
    m_thetaRad = WrapAngle(m_thetaRad);
    frc::SmartDashboard::PutBoolean("IMU_Read OK", ok);
    frc::SmartDashboard::PutNumber("IMU_GyroZ_radps", gz_radps);
    frc::SmartDashboard::PutNumber("IMU_GyroZ_Bias_radps", m_gyroZBiasRadps);
    frc::SmartDashboard::PutNumber("IMU_OmegaZ_radps", omega_z);
    frc::SmartDashboard::PutNumber("IMU_Theta_rad", m_thetaRad);

    // Encoder values, first we check if we are receiving all bytes then we handle the information into the dashboard
    int32_t e80_m1, e80_m2, e81_m1;
    uint8_t s80_m1, s80_m2, s81_m1;

    bool ok80_1 = RoboClawReadEncoderM1(kRoboClawAddr_Drive, e80_m1, s80_m1);
    bool ok80_2 = RoboClawReadEncoderM2(kRoboClawAddr_Drive, e80_m2, s80_m2);
    bool ok81_1 = RoboClawReadEncoderM1(kRoboClawAddr_Strafe, e81_m1, s81_m1);

    frc::SmartDashboard::PutBoolean("RC1 Encoder1 OK", ok80_1);
    frc::SmartDashboard::PutBoolean("RC1 Encoder2 OK", ok80_2);
    frc::SmartDashboard::PutBoolean("RC2 Encoder1 OK", ok81_1);

    if (ok80_1)
        frc::SmartDashboard::PutNumber("RC1 Encoder1", (double)e80_m1);
    if (ok80_2)
        frc::SmartDashboard::PutNumber("RC1 Encoder2", (double)e80_m2);
    if (ok81_1)
        frc::SmartDashboard::PutNumber("RC2 Encoder1", (double)e81_m1);

    // Encoder unit conversions
    double wd = 0.072;
    double wcirc = std::numbers::pi * wd;
    double x_mperpul = wcirc / 758.8;
    double y_mperpul = wcirc / 1425.1;

    // Convert pulses to actual meters traveled
    double xr_m_position = e80_m1 * x_mperpul;
    double xl_m_position = e80_m2 * x_mperpul;
    double y_m_position = e81_m1 * y_mperpul;
    // Pose estimation
    double x_m_position = (xr_m_position + xl_m_position) / 2.0;
    // IMU estimation (get Yaw in radians not degrees)
    double theta_position = m_thetaRad; // IMU data will load this variable

    // ── DONE: task already complete, just hold still ───────────────────────
    if(m_taskDone){
        StopAllDrive();
        return;
    }
    /* OR we use setpoints */
    // Setpoints
    // if (m_autoComplete || m_setpoints.empty())
    // {
    //     RoboClawStopAll();
    //     frc::SmartDashboard::PutString("Auto Status", "Complete");
    //     return;
    // }

    // Read the most recent AprilTag (if any)
    bool hasTag = m_aprilTagReader.HasTarget();
    AprilTagData tag{};
    if(hasTag){
        tag = m_aprilTagReader.GetPrimaryTag();
        // Reject stale / invalid pose readings (Pi publishes -1 when pose
        // estimation fails even though tag_detected is still true)
        if(tag.distance <= 0.0){
            hasTag = false;
        }
    }

    double x_target_meters = target.x_trgt; //Target would be tag?
    double y_target_meters = target.y_trgt;
    double theta_target_rads = target.theta_rad_trgt;

    // Errors
    double x_error_meters = x_target_meters - x_m_position;
    double y_error_meters = y_target_meters - y_m_position;
    double theta_error = WrapAngle(theta_target_rads - theta_position);

    // Integral
    x_integral += x_error_meters * dt;
    y_integral += y_error_meters * dt;
    theta_integral += theta_error * dt;
    // Anti-windup clamp
    x_integral = std::clamp(x_integral, -10.0, 10.0);
    y_integral = std::clamp(y_integral, -10.0, 10.0);
    theta_integral = std::clamp(theta_integral, -10.0, 10.0);

    // Derivative
    double x_derivative = (x_error_meters - x_prevError) / dt;
    double y_derivative = (y_error_meters - y_prevError) / dt;
    double theta_derivative = (theta_error - theta_prevError) / dt;

    // Gain Scheduling
    double abs_theta_error = std::abs(theta_error);
    double scheduled_theta_kP = 40.0;
    if (abs_theta_error > std::numbers::pi / 4)
    {
        scheduled_theta_kP = 80.0;
    }
    else if (abs_theta_error > std::numbers::pi / 12)
    {
        scheduled_theta_kP = 60.0;
    }

    // PID controller
    double x_controller = x_kP * x_error_meters + x_kI * x_integral + x_kD * x_derivative;
    double y_controller = y_kP * y_error_meters + y_kI * y_integral + y_kD * y_derivative;
    double theta_controller = scheduled_theta_kP * theta_error + theta_kI * theta_integral + theta_kD * theta_derivative;

    // Save error for next loop
    x_prevError = x_error_meters;
    y_prevError = y_error_meters;
    theta_prevError = theta_error;

    // Tolerance
    double x_tolerance = 0.005;
    double y_tolerance = 0.005;
    double theta_tolerance = 0.02;

    if (std::abs(x_error_meters) <= x_tolerance)
    {
        x_controller = 0.0;
        x_integral = 0.0;
    }
    if (std::abs(y_error_meters) <= y_tolerance)
    {
        y_controller = 0.0;
        y_integral = 0.0;
    }
    if (std::abs(theta_error) <= theta_tolerance)
    {
        theta_controller = 0.0;
        theta_integral = 0.0;
    }

    // At target logic
    bool x_at_target = std::abs(x_error_meters) <= x_tolerance;
    bool y_at_target = std::abs(y_error_meters) <= y_tolerance;
    bool theta_at_target = std::abs(theta_error) <= theta_tolerance;

    if (x_at_target && y_at_target && theta_at_target)
    {
        AdvanceToNextSetpoint();
        RoboClawStopAll();
        return;
    }

    // Saturation
    x_controller = std::clamp(x_controller, -127.0, 127.0);
    y_controller = std::clamp(y_controller, -127.0, 127.0);
    theta_controller = std::clamp(theta_controller, -80.0, 80.0);

    // Minimum command only outside tolerance
    if (theta_controller > 0.0 && theta_controller < 25.0)
        theta_controller = 25.0;
    if (theta_controller < 0.0 && theta_controller > -25.0)
        theta_controller = -25.0;
    if (y_controller > 0.0 && y_controller < 15.0)
        y_controller = 15.0;
    if (y_controller < 0.0 && y_controller > -15.0)
        y_controller = -15.0;

    // Motor Mixing
    double xr_controller = x_controller + theta_controller;
    double xl_controller = x_controller - theta_controller;

    // Clamp mixed wheel commands
    xr_controller = std::clamp(xr_controller, -127.0, 127.0);
    xl_controller = std::clamp(xl_controller, -127.0, 127.0);

    // Minimum tolerance for mixed motors
    if (xr_controller > 0.0 && xr_controller < 25.0)
        xr_controller = 25.0;
    if (xr_controller < 0.0 && xr_controller > -25.0)
        xr_controller = -25.0;
    if (xl_controller > 0.0 && xl_controller < 25.0)
        xl_controller = 25.0;
    if (xl_controller < 0.0 && xl_controller > -25.0)
        xl_controller = -25.0;

    // Command magnitudes
    double spdr = std::abs(xr_controller);
    double spdl = std::abs(xl_controller);
    double spdy = std::abs(y_controller);

    // Command motors separately
    if (xr_controller > 0.0)
    {
        RoboClawM1Forward(kRoboClawAddr_Drive, spdr);
    }
    else if (xr_controller < 0.0)
    {
        RoboClawM1Backward(kRoboClawAddr_Drive, spdr);
    }
    else
    {
        RoboClawM1Forward(kRoboClawAddr_Drive, 0);
    }

    if (xl_controller > 0.0)
    {
        RoboClawM2Forward(kRoboClawAddr_Drive, spdl);
    }
    else if (xl_controller < 0.0)
    {
        RoboClawM2Backward(kRoboClawAddr_Drive, spdl);
    }
    else
    {
        RoboClawM2Forward(kRoboClawAddr_Drive, 0);
    }

    if (y_controller > 0.0)
    {
        RoboClawM1Forward(kRoboClawAddr_Strafe, spdy);
    }
    else if (y_controller < 0.0)
    {
        RoboClawM1Backward(kRoboClawAddr_Strafe, spdy);
    }
    else
    {
        RoboClawM1Forward(kRoboClawAddr_Strafe, 0);
    }

    switch (m_autoPhase)
    {
    case AutoPhase::SEARCH:
        StopAllDrive();

        if(hasTag){
            m_autoPhase = AutoPhase::APPROACH;
            // Fall through intentionallly so we start driving this same tick rather than waiting one more 5 ms cycle
            [[fallthrough]];
        }
        else{
            frc::SmartDashboard::PutString("Auto/phase", "search");
            break;
        }
    // ── APPROACH: drive toward the tag ────────────────────────────────────
    case AutoPhase::APPROACH:
        //safety: if we lost the tag, stop and go back to searching
        if(!hasTag){
            StopAllDrive();
            m_autoPhase = AutoPhase::SEARCH;
            break;
        }
        
        //check stop condition so we dont overshoot
        if(tag.distance <= kStopDistance_m){
            StopAllDrive();
            //notify sir pi we have arrived
            m_taskDonePub.Set(true);
            m_sweepDonePub.Set(true); //also set legacy key
            m_taskDone = true;

            m_autoPhase = AutoPhase::DONE;
            break;
        }
        //still approaching - drive forward
        DriveVertical(static_cast<int8_t>(-kApproachSpeed));

         // ── Differential steering toward tag ──────────────────────────────
        //
        // pixelError > 0: tag is RIGHT of camera centre
        //   → robot rear is drifting LEFT relative to tag
        //   → slow left motor to swing rear right
        //
        // pixelError < 0: tag is LEFT of camera centre
        //   → robot rear is drifting RIGHT relative to tag
        //   → slow right motor to swing rear left
        //
        // DriveTankSteered() takes care of the sign arithmetic. We pass
        // -kApproachSpeed because the robot must drive BACKWARD to move
        // the camera (rear-facing) toward the tag.
        {
            double pixelError = tag.x - kCameraCenter_px;
            DriveTankSteered(-static_cast<int8_t>(kApproachSpeed),pixelError);

            frc::SmartDashboard::PutNumber("auto/tag_distance_m", tag.distance);
            frc::SmartDashboard::PutNumber("auto/tag_x_px", tag.x);
            frc::SmartDashboard::PutNumber("auto/tag_pixelError", pixelError);
        }

        frc::SmartDashboard::PutString("Auto/Phase", "Approach");
        break;

    case AutoPhase::DONE:
        StopAllDrive();
        frc::SmartDashboard::PutString("auto/phase", "Done");
        break;

    }
    // ── 7. Dashboard summary ──────────────────────────────────────────────
    frc::SmartDashboard::PutNumber("Auto/ElapsedTime_s", m_timer.Get().value());
    // frc::SmartDashboard::PutNumber("Auto/Yaw_deg",       m_yaw_deg);
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