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
#include <frc/Encoder.h> //library for encoders
#include <frc/controller/PIDController.h> //library for PID
#include "rev/ServoHub.h" //Servo hub lib
#include <frc/AnalogInput.h> //hall sensor test
#include <frc/DigitalInput.h> //hall sensor test
#include <frc/SerialPort.h> //UART communication with Roboclaw
#include "AprilTagReader.h"

class Robot : public frc::TimesliceRobot {
 public:
  Robot();
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationPeriodic() override;

  void move(double speed);
  void stopMotor();

 private:
  frc::SendableChooser<std::string> m_chooser;
  std::string m_autoSelected;
  frc::Timer m_timer;

  //Roboclaw Serial Port and address for them to share the same serial address
  frc::SerialPort m_roboclaw{38400, frc::SerialPort::Port::kMXP};
  static constexpr uint8_t kRoboClawAddr1 = 0x80;
  static constexpr uint8_t kRoboClawAddr2 = 0x81;

  uint16_t RoboClawCRC16(const uint8_t* data, int len); 
  void RoboClawDrain(); //Drain buffer so solb bytes are not stuck in the stream
  void RoboClawSend3(uint8_t addr, uint8_t cmd, uint8_t value); //Send packets
  //Movement commands
  void RoboClawM1Forward(uint8_t addr, uint8_t speed);
  void RoboClawM1Backward(uint8_t addr, uint8_t speed);
  void RoboClawM2Forward(uint8_t addr, uint8_t speed);
  void RoboClawM2Backward(uint8_t addr, uint8_t speed);
  void RoboClawStop(uint8_t addr);
  void RoboClawStopAll();
  //Encoder data commands
  bool RoboClawReadEncoder(uint8_t addr, uint8_t cmd, uint32_t& count, uint8_t& status);
  bool RoboClawReadEncoderM1(uint8_t addr, uint32_t& count, uint8_t& status);
  bool RoboClawReadEncoderM2(uint8_t addr, uint32_t& count, uint8_t& status);

  //hall sensor inputs
  frc::AnalogInput m_hallAnalog{0};   // AI0
  frc::DigitalInput m_hallDigital{9}; // DIO 8

  //servohub id and channel configuration
  static constexpr int kServoHubId = 20;
  rev::servohub::ServoHub m_servoHub{kServoHubId};

  rev::servohub::ServoChannel& m_servo0{m_servoHub.GetServoChannel(rev::servohub::ServoChannel::ChannelId::kChannelId0)};

  rev::servohub::ServoChannel& m_servo1{m_servoHub.GetServoChannel(rev::servohub::ServoChannel::ChannelId::kChannelId1)};

  rev::servohub::ServoChannel& m_servo2{m_servoHub.GetServoChannel(rev::servohub::ServoChannel::ChannelId::kChannelId2)};

  rev::servohub::ServoChannel& m_servo3{m_servoHub.GetServoChannel(rev::servohub::ServoChannel::ChannelId::kChannelId3)};

  AprilTagReader m_aprilTagReader;
};