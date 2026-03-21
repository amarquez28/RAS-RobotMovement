// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimesliceRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/PWM.h>
#include <frc/DigitalOutput.h>
#include <frc/Timer.h>
#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include "rev/ServoHub.h"
#include <frc/AnalogInput.h>
#include <frc/DigitalInput.h>
#include <frc/SerialPort.h>
#include <frc/I2C.h>
#include "AprilTagReader.h"
#include "SweepController.h"

// ── Autonomous phase ─────────────────────────────────────────────────────────
// SEARCH  : No tag visible yet – robot holds position and waits.
// APPROACH: Tag detected – robot drives straight toward it until ≤ 0.5 m away.
// DONE    : Within stopping distance – motors stopped, Pi notified.
enum class AutoPhase{
    SEARCH,
    APPROACH,
    DONE
};

class Robot : public frc::TimesliceRobot {
 public:
  Robot();
  void RobotPeriodic()      override;
  void AutonomousInit()     override;
  void AutonomousPeriodic() override;
  void TeleopInit()         override;
  void TeleopPeriodic()     override;
  void DisabledInit()       override;
  void DisabledPeriodic()   override;
  void TestInit()           override;
  void TestPeriodic()       override;
  void SimulationPeriodic() override;

  // ── high-level drive helpers ────────────────────────────────────────────
  // speed: -127 (full back) to +127 (full forward)
  void DriveVertical(int8_t speed);
  // speed: -127 (full left) to +127 (full right)
  void DriveHorizontal(int8_t speed);
  void StopAllDrive();

  // Differential steering: drives M1 and M2 at different speeds to turn
  // while moving. Used during tag approach instead of strafe.
  // baseSpeed: signed (negative = backing toward rear camera / tag)
  // pixelError: tag.x - kCameraCenter_px (positive = tag right of centre)
  void DriveTankSteered(int8_t baseSpeed, double pixelError);

 private:
  // ── SmartDashboard / auto chooser ───────────────────────────────────────
  frc::SendableChooser<std::string> m_chooser;
  std::string m_autoSelected;
  frc::Timer  m_timer;

  // ── RoboClaw serial ─────────────────────────────────────────────────────
  // MXP UART at 38400 baud, shared by both controllers
  frc::SerialPort m_roboclaw{38400, frc::SerialPort::Port::kMXP};

  // RoboClaw addresses
  // 0x80 → RC1: M1 = Left drive motor,  M2 = Right drive motor  (vertical)
  // 0x81 → RC2: M1 = Strafe motor                               (horizontal)
  static constexpr uint8_t kRoboClawAddr_Drive  = 0x80;
  static constexpr uint8_t kRoboClawAddr_Strafe = 0x81;

  // RoboClaw low-level API
  uint16_t RoboClawCRC16(const uint8_t* data, int len);
  void     RoboClawDrain();
  void     RoboClawSend3(uint8_t addr, uint8_t cmd, uint8_t value);

  // Per-motor direction commands (speed 0–127)
  void RoboClawM1Forward (uint8_t addr, uint8_t speed);
  void RoboClawM1Backward(uint8_t addr, uint8_t speed);
  void RoboClawM2Forward (uint8_t addr, uint8_t speed);
  void RoboClawM2Backward(uint8_t addr, uint8_t speed);
  void RoboClawStop(uint8_t addr);
  void RoboClawStopAll();

  // Encoder read commands – all counts are signed int32
  bool RoboClawReadEncoder  (uint8_t addr, uint8_t cmd, int32_t& count, uint8_t& status);
  bool RoboClawReadEncoderM1(uint8_t addr, int32_t& count, uint8_t& status);
  bool RoboClawReadEncoderM2(uint8_t addr, int32_t& count, uint8_t& status);

  // ── Hall sensor ─────────────────────────────────────────────────────────
  frc::AnalogInput  m_hallAnalog{0};   // AI0
  frc::DigitalInput m_hallDigital{9};  // DIO 9

  // ── REV ServoHub ────────────────────────────────────────────────────────
  static constexpr int kServoHubId = 20;
  rev::servohub::ServoHub m_servoHub{kServoHubId};

  rev::servohub::ServoChannel& m_servo0{
      m_servoHub.GetServoChannel(rev::servohub::ServoChannel::ChannelId::kChannelId0)};
  rev::servohub::ServoChannel& m_servo1{
      m_servoHub.GetServoChannel(rev::servohub::ServoChannel::ChannelId::kChannelId1)};
  rev::servohub::ServoChannel& m_servo2{
      m_servoHub.GetServoChannel(rev::servohub::ServoChannel::ChannelId::kChannelId2)};
  rev::servohub::ServoChannel& m_servo3{
      m_servoHub.GetServoChannel(rev::servohub::ServoChannel::ChannelId::kChannelId3)};

  // Servo pulse widths (microseconds)
  static constexpr int kHallServoInitPos = 500;   // Closed
  static constexpr int kHallServoOpenPos = 1500;  // Open

  // ── IMU (MPU-6050 on I²C onboard port, addr 0x68) ──────────────────────
  frc::I2C m_imu{frc::I2C::Port::kOnboard, 0x68};

  bool  IMUWriteReg (uint8_t reg, uint8_t data);
  bool  IMUReadRegs (uint8_t startReg, uint8_t* out, int len);
  void  IMUInit     ();
  void  IMUUpdate   ();   // Integrates gyro Z → m_yaw_deg
  void  IMUDashboard();   // Pushes raw + converted values to SmartDashboard

  // Integrated yaw (degrees). Positive = counter-clockwise when viewed from top.
  double          m_yaw_deg       = 0.0;
  double          m_lastGyroZ_dps = 0.0;
  units::second_t m_lastIMUTime{0};

  // ── Encoder snapshot used by sweep ─────────────────────────────────────
  // RoboClaw 0x80 M1 (left drive)  → vertical odometer
  // RoboClaw 0x81 M1 (strafe motor) → horizontal odometer
  // Both are signed: positive = forward / right respectively.
  int32_t m_vertTicks    = 0;
  int32_t m_horizTicks   = 0;
  bool    m_encodersValid = false;

  void UpdateEncoders();   // Reads both odometry encoders every periodic tick

  // ── Vision / AprilTag ───────────────────────────────────────────────────
  AprilTagReader m_aprilTagReader;

  // ── Sweep state machine ─────────────────────────────────────────────────
  SweepController m_sweep;
  bool            m_sweepStarted = false;

  // ── Tag-approach autonomous phase ───────────────────────────────────────
  // The robot waits for a tag, then drives straight toward it until
  
  AutoPhase m_autoPhase = AutoPhase::SEARCH;
  bool m_taskDone = false;

  //stopping distance from the tag (meters)
  static constexpr double kStopDistance_m = 0.2;

  //forward drive speed while approaching (0-127)
  static constexpr uint8_t kApproachSpeed = 45;

  //lateral correction speed for centering on the tag (0-127)
  //applied as a strafe blend when the tag is off-center in the camera frame
  static constexpr uint8_t kCenterSpeed = 20;

// ── Camera centering constants ────────────────────────────────────────
  //
  // kCameraCenter_px: the pixel column (tag.x) that appears when the tag is
  // directly in front of the camera lens, i.e. "straight ahead of the camera".
  //
  // Because the camera is offset to the RIGHT of the robot centreline, this
  // value will NOT be 740 (half of 1480). You must measure it:
  //   1. Align a tag directly in front of the camera lens.
  //   2. Read "Vision/Primary X" from SmartDashboard.
  //   3. Average 5–10 readings and set that value here.
  //
  // Starting estimate: 740 (true geometric centre of a 1480 px wide frame).
  // Adjust after measurement. A rightward camera offset typically produces a
  // value LESS than 740 (the tag appears left of the frame centre when
  // the robot is pointed straight at it).
  static constexpr double kCameraCenter_px = 650.0;
 
  // How far off-center (pixels) before we apply a lateral correction.
  static constexpr double kCenterTolerance_px = 40.0;

  // Steering gain: fraction of base speed applied as differential at max error.
  // At pixelError == kCenterTolerance_px, inner wheel slows by kSteerFactor * baseSpeed.
  // Start at 0.3 and tune upward if the robot fails to track a laterally
  // offset tag, or downward if it oscillates.
  //   If the robot isn't correcting enough, increase it. If it oscillates side to side, decrease it.
  static constexpr double kSteerFactor = 0.3;
 
  // ── NetworkTables publishers ─────────────────────────────────────────────
  // Vision/task_done   → Pi reads this to know the robot has arrived.
  // Vision/sweep_done  → Legacy sweep-completion signal (kept for compatibility).
  nt::BooleanPublisher m_taskDonePub;
  nt::BooleanPublisher m_sweepDonePub;
};