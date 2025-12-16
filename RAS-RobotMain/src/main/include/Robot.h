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
  frc::PWM m_pwm{0};
  frc::DigitalOutput m_in1{0};
  frc::DigitalOutput m_in2{1};
  frc::DigitalOutput m_in3{2};
  frc::DigitalOutput m_in4{3};
  frc::Encoder m_encoder{4,5}; //Encoders use two Digital Input ports (channels). Let's assume 4 and 5.
  frc::PIDController m_pid{1.0, 0.0, 0.0}; //P = 1.0 means if we are 1 meter away, go at speed 1.0 (full speed).
};
