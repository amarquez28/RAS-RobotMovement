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
  //Encoder data commands
  bool RoboClawReadEncoder(uint8_t addr, uint8_t cmd, int32_t& count, uint8_t& status);
  bool RoboClawReadEncoderM1(uint8_t addr, int32_t& count, uint8_t& status);
  bool RoboClawReadEncoderM2(uint8_t addr, int32_t& count, uint8_t& status);

  // ── Hall sensor ─────────────────────────────────────────────────────────
  frc::AnalogInput m_hallAnalog{0};   // AI0
  frc::DigitalInput m_hallDigital{9}; // DIO 9

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

  // Servo pulse widths
  static constexpr int kHallServoInitPos = 500;   // Closed
  static constexpr int kHallServoOpenPos = 1500;  // Open

  // ── IMU (MPU-6050 on I²C onboard port, addr 0x68) ──────────────────────
  frc::I2C m_imu{frc::I2C::Port::kOnboard, 0x68};

  bool  IMUWriteReg   (uint8_t reg, uint8_t data);
  bool  IMUReadRegs   (uint8_t startReg, uint8_t* out, int len);
  void  IMUInit       ();
  void  IMUUpdate     ();          // Integrates gyro Z → m_yaw_deg
  void  IMUDashboard  ();          // Pushes raw + converted values

  // Integrated yaw (degrees).  Positive = counter-clockwise when viewed from top.
  double m_yaw_deg         = 0.0;
  double m_lastGyroZ_dps   = 0.0;
  units::second_t m_lastIMUTime{0};

  // ── Encoder snapshot used by sweep ─────────────────────────────────────
  // We use RoboClaw 0x80 M1 (left drive) as the vertical odometer
  // and RoboClaw 0x81 M1 (strafe motor) as the horizontal odometer.
  int32_t m_vertTicks  = 0;   // signed: positive = forward
  int32_t m_horizTicks = 0;   // signed: positive = right

  // Raw 32-bit unsigned reads from RoboClaw (encoder wraps at 0xFFFFFFFF)
  uint32_t m_vertRaw  = 0;
  uint32_t m_horizRaw = 0;
  bool     m_encodersValid = false;

  // Convert unsigned RoboClaw encoder value to a signed relative count.
  // RoboClaw encoders start at 0x00000000 and wrap; treat the midpoint
  // (0x80000000) as the sign boundary.
  static int32_t ToSigned(uint32_t raw) {
      return static_cast<int32_t>(raw);
  }

  void UpdateEncoders();   // reads both encoders every periodic tick

  // ── Vision / AprilTag ───────────────────────────────────────────────────
  AprilTagReader m_aprilTagReader;

  // ── Sweep state machine ─────────────────────────────────────────────────
  SweepController m_sweep;
  bool            m_sweepStarted = false;
};