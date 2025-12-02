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

  void moveFwd(double speed);
  void moveBwd(double speed);
  void stopMotor();

 private:
  frc::SendableChooser<std::string> m_chooser;
  // const std::string kAutoNameDefault = "Default";
  // const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  frc::Timer m_timer;
  frc::PWM m_pwm{0};
  frc::DigitalOutput m_in1{0};
  frc::DigitalOutput m_in2{1};
  frc::DigitalOutput m_in3{2};
  frc::DigitalOutput m_in4{3};

};
