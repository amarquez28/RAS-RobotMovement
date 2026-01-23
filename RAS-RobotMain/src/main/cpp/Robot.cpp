// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/print.h>
#include <frc/I2C.h>
#include <cstdint>
#include <algorithm>  // for std::clamp, required for compilation

int pulse = 500;     // center position
int dir = 1;
int counter = 0;
bool start = false;

static void InitIMU();

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

  InitIMU();

  // RoboClaw: clean buffers & stop motors on boot
  m_roboclaw.Reset();
  RoboClawStop();
}

//Roboclaw Helper Functions 

//CRC verification
uint16_t Robot::RoboClawCRC16(const uint8_t* data, int len) {
  uint16_t crc = 0;
  for (int i = 0; i < len; i++) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

//Packet Creation
void Robot::RoboClawSend3(uint8_t cmd, uint8_t value) {
  uint8_t pkt[3] = {kRoboClawAddr, cmd, value};
  uint16_t crc = RoboClawCRC16(pkt, 3);

  uint8_t out[5] = {
      kRoboClawAddr,
      cmd,
      value,
      static_cast<uint8_t>((crc >> 8) & 0xFF),
      static_cast<uint8_t>(crc & 0xFF)
  };

  m_roboclaw.Write(reinterpret_cast<const char*>(out), 5);
}

//Motor command wrappers
void Robot::RoboClawM1Forward(uint8_t speed)  { RoboClawSend3(0, speed); } // cmd 0
void Robot::RoboClawM1Backward(uint8_t speed) { RoboClawSend3(1, speed); } // cmd 1
void Robot::RoboClawM2Forward(uint8_t speed)  { RoboClawSend3(4, speed); } // cmd 4
void Robot::RoboClawM2Backward(uint8_t speed) { RoboClawSend3(5, speed); } // cmd 5

void Robot::RoboClawStop() {
  RoboClawM1Forward(0);
  RoboClawM2Forward(0);
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

static frc::I2C imu{frc::I2C::Port::kOnboard, 0x68};

// Write 1 byte to a register
static bool WriteReg(uint8_t reg, uint8_t data) {
  uint8_t buf[2] = {reg, data};
  return imu.WriteBulk(buf, 2); // false = success, true = error
}

// Read N bytes starting at a register
static bool ReadRegs(uint8_t startReg, uint8_t* out, int len) {
  bool writeErr = imu.WriteBulk(&startReg, 1);
  if (writeErr) return true;    // true = error
  return imu.ReadOnly(len, out); // true = error
}

// Convert two bytes (big-endian) to signed 16-bit
static int16_t ToInt16(uint8_t hi, uint8_t lo) {
  return static_cast<int16_t>((hi << 8) | lo);
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

static void InitIMU() {
  // Wake up: PWR_MGMT_1 (0x6B) = 0x00
  WriteReg(0x6B, 0x00);

  // Gyro full-scale: ±250 dps (GYRO_CONFIG 0x1B = 0x00)
  WriteReg(0x1B, 0x00);

  // Accel full-scale: ±2g (ACCEL_CONFIG 0x1C = 0x00)
  WriteReg(0x1C, 0x00);
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

static void UpdateIMUDashboard() {
  // --- WHO_AM_I ---
  fmt::print("Accessed\n");
  uint8_t who = 0;
  bool whoErr = ReadRegs(0x75, &who, 1); // WHO_AM_I = 0x75

  frc::SmartDashboard::PutBoolean("IMU_WHO_AM_I Error", whoErr);
  frc::SmartDashboard::PutNumber("IMU_WHO_AM_I (hex)", who); // show as number (0-255)

  // --- Accel + Gyro burst read ---
  // ACCEL_XOUT_H starts at 0x3B (14 bytes)
  uint8_t data[14] = {0};
  bool readErr = ReadRegs(0x3B, data, 14);

  frc::SmartDashboard::PutBoolean("IMU_Read Error", readErr);
  if (readErr) return;

  int16_t ax = ToInt16(data[0],  data[1]);
  int16_t ay = ToInt16(data[2],  data[3]);
  int16_t az = ToInt16(data[4],  data[5]);

  int16_t gx = ToInt16(data[8],  data[9]);
  int16_t gy = ToInt16(data[10], data[11]);
  int16_t gz = ToInt16(data[12], data[13]);

  // Raw values
  frc::SmartDashboard::PutNumber("IMU_Accel Raw X", ax);
  frc::SmartDashboard::PutNumber("IMU_Accel Raw Y", ay);
  frc::SmartDashboard::PutNumber("IMU_Accel Raw Z", az);

  frc::SmartDashboard::PutNumber("IMU_Gyro Raw X", gx);
  frc::SmartDashboard::PutNumber("IMU_Gyro Raw Y", gy);
  frc::SmartDashboard::PutNumber("IMU_Gyro Raw Z", gz);

  // Convert to physical units (for ±2g and ±250 dps)
  // ±2g: 16384 LSB/g
  // ±250 dps: 131 LSB/(deg/s)
  double ax_g = ax / 16384.0;
  double ay_g = ay / 16384.0;
  double az_g = az / 16384.0;

  double gx_dps = gx / 131.0;
  double gy_dps = gy / 131.0;
  double gz_dps = gz / 131.0;

  frc::SmartDashboard::PutNumber("IMU_Accel g X", ax_g);
  frc::SmartDashboard::PutNumber("IMU_Accel g Y", ay_g);
  frc::SmartDashboard::PutNumber("IMU_Accel g Z", az_g);

  frc::SmartDashboard::PutNumber("IMU_Gyro dps X", gx_dps);
  frc::SmartDashboard::PutNumber("IMU_Gyro dps Y", gy_dps);
  frc::SmartDashboard::PutNumber("IMU_Gyro dps Z", gz_dps);
}

void Robot::AutonomousPeriodic() {

  //get data: read the current distance from encoder
  //double currentDistance = m_encoder.GetDistance();

  //calculate: ask pid controller what speed we need to reach the setpoint 
  //this returns a value between -1.0(full reverse) and 1.0 (full forward)
  //double outputSpeed = m_pid.Calculate(currentDistance);
  //outputSpeed = std::clamp(outputSpeed, -1.0, 1.0);

  //actuate: send the data to motor functions

  //check if we are at the distance(within tolerance)
  /*if(m_pid.AtSetpoint()){
    stopMotor();
  }
  else{
    move(outputSpeed);
  } */
/*
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
    UpdateIMUDashboard();
    pulse += dir * 10;

    if (pulse >= 2000) dir = -1;
    if (pulse <= 1000) dir =  1;

    m_servo0.SetPulseWidth(pulse);
    m_servo1.SetPulseWidth(pulse);
    m_servo3.SetPulseWidth(pulse);
    frc::SmartDashboard::PutNumber("Pulse: ", pulse);
  } */

  double t = m_timer.Get().value();
  uint8_t spd = 60;  // 0-127

  if (t < 10.0) {
    RoboClawM1Forward(spd);
    RoboClawM2Forward(spd);
    return;
  } else if (t < 20.0) {
    RoboClawM1Backward(spd);
    RoboClawM2Backward(spd);
    return;
  } else {
    RoboClawStop();
    return;
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
