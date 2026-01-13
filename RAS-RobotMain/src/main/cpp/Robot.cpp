// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/print.h>

int pulse = 500;     // center position
int dir = 1;
int counter = 0;
bool start = false;

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

  //distance per pulse
  //must be calculted for specific robot
  //formula (wheel circumfrence (M))/ (Encoder pulses per revolution)
  // Example: 6 inch wheel (0.1524m dia) -> Circumference = 0.478m
  // Our encoder resolution is 751.88
  // Our circumfrence is 0.226195
  // If encoder has 360 ticks per rev: 0.478 / 360 = 0.00132
  m_encoder.SetDistancePerPulse(0.0003008392); // Replace with our calculated value

  //PID Tolerance
  //we are done if we are within 0.05 meters of target
  m_pid.SetTolerance(0.05);

  m_servo0.SetPowered(true);
  m_servo1.SetPowered(true);
  m_servo3.SetPowered(true);

  m_servo0.SetEnabled(true);
  m_servo1.SetEnabled(true);
  m_servo3.SetEnabled(true);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("Encoder Dist", m_encoder.GetDistance());
  frc::SmartDashboard::PutNumber("Encoder Rate", m_encoder.GetRate());

  //hall sensor dashboard values
  frc::SmartDashboard::PutNumber("Hall Voltage (V)", m_hallAnalog.GetVoltage());
  frc::SmartDashboard::PutNumber("Hall Raw (0-4095)", m_hallAnalog.GetValue());
  frc::SmartDashboard::PutBoolean("Hall Digital (DIO8)", m_hallDigital.Get());
}

// void Robot::moveFwd(double speed){
//   if(speed > 1.0){
//     speed = 1.0;
//   }

//   if(speed < 0.0){
//     speed = 0.0;
//   }

//   m_in1.Set(true);
//   m_in2.Set(false);
//   m_pwm.SetSpeed(speed);

// }

void Robot::stopMotor(){
  m_in1.Set(false);
  m_in2.Set(false);
  m_pwm.SetSpeed(0);
}

void Robot::AutonomousInit() {
  m_timer.Reset();
  m_timer.Start();
  m_encoder.Reset();
  m_pid.Reset();
  //set our goal: move forward 1.0 meter
  m_pid.SetSetpoint(1.0);

  // Force known start position
  m_servo0.SetPulseWidth(pulse);
  m_servo1.SetPulseWidth(pulse);
  m_servo3.SetPulseWidth(pulse);

}

// void Robot::moveBwd(double speed){
//   if(speed > 1.0){
//     speed = 1.0;
//   }
//   if(speed < 0.0){
//     speed = 0.0;
//   }
//   m_in1.Set(false);
//   m_in2.Set(true);
//   m_pwm.SetSpeed(speed);
// }

void Robot::move(double speed){
  speed = std::clamp(speed, -1.0, 1.0);
  if(speed > 0){
    m_in1.Set(true);
    m_in2.Set(false);
  }
  else if(speed < 0){
    m_in1.Set(false);
    m_in2.Set(true);
  }
  else{
    m_in1.Set(false);
    m_in2.Set(false);
  }
  m_pwm.SetSpeed(std::abs(speed));
}

void Robot::AutonomousPeriodic() {

  //get data: read the current distance from encoder
  double currentDistance = m_encoder.GetDistance();

  //calculate: ask pid controller what speed we need to reach the setpoint 
  //this returns a value between -1.0(full reverse) and 1.0 (full forward)
  double outputSpeed = m_pid.Calculate(currentDistance);
  outputSpeed = std::clamp(outputSpeed, -1.0, 1.0);

  //actuate: send the data to motor functions

  //check if we are at the distance(within tolerance)
  if(m_pid.AtSetpoint()){
    stopMotor();
  }
  else{
    move(outputSpeed);
  }

  if (!start) {
    counter++;

    if (counter >= 25) {  // ~25 loops ≈ 0.5s
      start = true;
      counter = 0;
    }

    return;  // do NOT move yet
  }

  // --- Normal slow 180° sweep ---
  counter++;

  if (counter >= 10) {
    counter = 0;

    pulse += dir * 20;

    if (pulse >= 2000) dir = -1;
    if (pulse <= 1000) dir =  1;

    m_servo0.SetPulseWidth(pulse);
    m_servo1.SetPulseWidth(pulse);
    m_servo3.SetPulseWidth(pulse);
    frc::SmartDashboard::PutNumber("Pulse: ", pulse);
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
