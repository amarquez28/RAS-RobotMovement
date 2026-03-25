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
#include <units/time.h>
#include <frc/Encoder.h> //library for encoders
#include <frc/controller/PIDController.h> //library for PID
#include "rev/ServoHub.h" //Servo hub lib
#include <frc/AnalogInput.h> //hall sensor test
#include <frc/DigitalInput.h> //hall sensor test
#include <frc/SerialPort.h> //UART communication with Roboclaw
#include "AprilTagReader.h"
#include <vector>

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
  bool RoboClawReadEncoder(uint8_t addr, uint8_t cmd, int32_t& count, uint8_t& status);
  bool RoboClawReadEncoderM1(uint8_t addr, int32_t& count, uint8_t& status);
  bool RoboClawReadEncoderM2(uint8_t addr, int32_t& count, uint8_t& status);
  //Encoder reset
  void RoboClawResetEncoder(uint8_t addr);
  void RoboClawResetAllEncoders();
  //Wrap Angle for Theta controller
  double WrapAngle(double angle);
  //IMU Calibration before starting the program
  void CalibrateGyroZBias();
  void UpdateIMUTheta();
  //Setpoint Helpers
  void LoadAutonomousSetpoints();
  void AdvanceToNextSetpoint();
  void ResetPidState();

  //hall sensor inputs
  frc::AnalogInput m_hallAnalog{0};   // AI0
  frc::DigitalInput m_hallDigital{9}; // DIO 8

  //servohub id and channel configuration
  static constexpr int kServoHubId = 20;
  rev::servohub::ServoHub m_servoHub{kServoHubId};
  rev::servohub::ServoChannel& m_servoBrush{m_servoHub.GetServoChannel(rev::servohub::ServoChannel::ChannelId::kChannelId0)}; //Brush
  rev::servohub::ServoChannel& m_servoRelease{m_servoHub.GetServoChannel(rev::servohub::ServoChannel::ChannelId::kChannelId4)}; //Release Orbs Door
  rev::servohub::ServoChannel& m_servoHall{m_servoHub.GetServoChannel(rev::servohub::ServoChannel::ChannelId::kChannelId2)}; //Hall Sensor
  rev::servohub::ServoChannel& m_servoArm{m_servoHub.GetServoChannel(rev::servohub::ServoChannel::ChannelId::kChannelId5)}; //Arm
  //PID Tuning
  // Gains
  double x_kP = 105.0;
  double x_kI = 19.0;
  double x_kD = 0.8;
  double y_kP = 200.0;
  double y_kI = 50.0;
  double y_kD = 0.0;
  double theta_kI = 0.0;
  double theta_kD = 0.0;
  //Forward/Backward wheel PID state
  double x_integral = 0.0;
  double x_prevError = 0.0;
  //Strafe wheel PID state
  double y_integral = 0.0;
  double y_prevError = 0.0;
  //Theta (IMU) PID State
  double theta_integral = 0.0;
  double theta_prevError = 0.0;
  double m_thetaRad = 0.0;
  double m_gyroZBiasRadps = 0.0;

  // Time tracking
  units::second_t m_prevTime = 0_s;
  bool m_firstPidLoop = true;

  //Setpoint declaration
  struct Setpoint {
    double x_trgt;
    double y_trgt;
    double theta_rad_trgt;
  };

  std::vector<Setpoint> m_setpoints;
  size_t m_currentSetpointIndex = 0;
  bool m_autoComplete = false; 

  AprilTagReader m_aprilTagReader;
};