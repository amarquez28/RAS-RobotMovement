// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/print.h>

// Run robot periodic() functions for 5 ms, and run controllers every 10 ms
Robot::Robot() : frc::TimesliceRobot{5_ms, 10_ms} {
  // LiveWindow causes drastic overruns in robot periodic functions that will
  // interfere with controllers
  frc::LiveWindow::DisableAllTelemetry();

  // Runs for 2 ms after robot periodic functions
  Schedule([=] {}, 2_ms);

  // Runs for 2 ms after first controller function
  Schedule([=] {}, 2_ms);

  // Total usage:
  // 5 ms (robot) + 2 ms (controller 1) + 2 ms (controller 2) = 9 ms
  // 9 ms / 10 ms -> 90% allocated

  // m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  // m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */

void Robot::moveFwd(double speed){
  if(speed > 1.0){
    speed = 1.0;
  }

  if(speed < 0.0){
    speed = 0.0;
  }

  m_in1.Set(true);
  m_in2.Set(false);
  m_pwm.SetSpeed(speed);

}

void Robot::stopMotor(){
  m_in1.Set(false);
  m_in2.Set(false);
  m_pwm.SetSpeed(0);
}

void Robot::AutonomousInit() {
  m_timer.Reset();
  m_timer.Start();
}

void Robot::moveBwd(double speed){
  if(speed > 1.0){
    speed = 1.0;
  }
  if(speed < 0.0){
    speed = 0.0;
  }
  m_in1.Set(false);
  m_in2.Set(true);
  m_pwm.SetSpeed(speed);
}

void Robot::AutonomousPeriodic() {
  units::time::second_t time = m_timer.Get();

  if(time < 5_s){
    moveFwd(1);
  }
  else if(time < 7_s){
    stopMotor();
  }
  else if(time < 12_s){
    moveBwd(1);
  }
  else{
    stopMotor();
  }

}

//TODO: add use the PhysicsSim classes (like DifferentialDrivetrainSim or SwerveDriveSim), 
//read the motor voltages, pass them to the simulator object, and then update your odometry with the new calculated position.
void Robot::SimulationPeriodic(){

}


void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
