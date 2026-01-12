// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/print.h>
#include <frc/AnalogInput.h>

// Run robot periodic() functions for 5 ms, and run controllers every 10 ms
Robot::Robot() : frc::TimesliceRobot{5_ms, 10_ms} {

  NetworkTable *table;
  AHRS *ahrs;
  LiveWindow *lw;
  int autoLoopCounter;

  table(NULL),
  ahrs(NULL),
  lw(NULL),
  autoLoopCounter(0)

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

  table = NetworkTable::GetTable("datatable");
  lw = LiveWindow::GetInstance();
  try {
    /* Communicate w/navX-MXP via the MXP SPI Bus.                                       */
    /* Alternatively:  I2C::Port::kMXP, SerialPort::Port::kMXP or SerialPort::Port::kUSB */
    /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.   */
    ahrs = new AHRS(SPI::Port::kMXP);
  } 
  catch (std::exception ex ) {
    std::string err_string = "Error instantiating navX-MXP:  ";
    err_string += ex.what();
    DriverStation::ReportError(err_string.c_str());
  }
  if ( ahrs ) {
    LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", ahrs);
  }

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

  autoLoopCounter = 0;
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
  if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
  {
    autoLoopCounter++;
  }

  //navX-MXP IMU data
  frc::SmartDashboard::PutBoolean( "IMU_Connected",        ahrs->IsConnected());
  frc::SmartDashboard::PutNumber(  "IMU_Yaw",              ahrs->GetYaw());
  frc::SmartDashboard::PutNumber(  "IMU_Pitch",            ahrs->GetPitch());
  frc::SmartDashboard::PutNumber(  "IMU_Roll",             ahrs->GetRoll());
  frc::SmartDashboard::PutNumber(  "IMU_CompassHeading",   ahrs->GetCompassHeading());
  frc::SmartDashboard::PutNumber(  "IMU_Update_Count",     ahrs->GetUpdateCount());
  frc::SmartDashboard::PutNumber(  "IMU_Byte_Count",       ahrs->GetByteCount());

  //Hall sensor test - detect magnet
  double hallVoltage = m_hallSensor.GetVoltage(); 
  const double magnetThreshold = 2.5; //example threshold voltage
  if (hallVoltage > magnetThreshold) {
    // Magnet detected
    frc::SmartDashboard::PutString("Magnet Status", "Detected");
    frc::SmartDashboard::PutString(hallVoltage);
  } else {
    // No magnet detected
    frc::SmartDashboard::PutString("Magnet Status", "Not Detected");
  }

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
