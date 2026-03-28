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
#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include "rev/ServoHub.h"
#include <frc/AnalogInput.h>
#include <frc/DigitalInput.h>
#include <frc/SerialPort.h> //UART communication with Roboclaw
#include <frc/I2C.h>
#include "AprilTagReader.h"
#include <vector>
#include <AutonomousPaths.h>

//Pose estimation required libraries
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/estimator/DifferentialDrivePoseEstimator.h>
#include <units/length.h>
#include <units/angle.h>

// ── Autonomous phase ─────────────────────────────────────────────────────────
//             Resets encoders + IMU when done, then transitions to APPROACH.
// SEARCH    : No tag visible yet – robot holds position and waits.
// APPROACH  : Running a waypoint path via PID dead-reckoning.
// TAG_SEARCH: Default path complete – robot holds position and reads AprilTag
//             ID to select which sub-path to run next.
// DONE      : All paths complete – motors stopped, Pi notified.
enum class AutoPhase{
    CENTERING,
    SEARCH,
    APPROACH,
    TAG_SEARCH,
    DONE,
    TEST
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
  void StopAllDrive();

  // Differential steering: drives M1 and M2 at different speeds to turn
  // while moving. Used during tag approach for steering.
  // baseSpeed: signed (negative = backing toward rear camera / tag)
  // pixelError: tag.x - kCameraCenter_px (positive = tag right of centre)
  void DriveTankSteered(int8_t baseSpeed, double pixelError);

 private:
   units::second_t m_autoStartTime{0_s};

  // ── SmartDashboard / auto chooser ───────────────────────────────────────
  frc::SendableChooser<std::string> m_chooser;
  std::string m_autoSelected;
  frc::Timer  m_timer;

  // ── RoboClaw serial ─────────────────────────────────────────────────────
  // MXP UART at 38400 baud, shared by both controllers
  frc::SerialPort m_roboclaw{38400, frc::SerialPort::Port::kMXP};

  // RoboClaw addresses
  // 0x80 → RC1: M1 = Right drive motor, M2 = Left drive motor
  static constexpr uint8_t kRoboClawAddr_Drive  = 0x80;
  // 0x81 → RC2: M2 = Linear actuator only (M1/strafe removed)
  static constexpr uint8_t kRoboClawAddr_Actuator = 0x81;

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
  //Encoder reset
  void RoboClawResetEncoder(uint8_t addr);
  void RoboClawResetAllEncoders();
  //Wrap Angle for Theta controller
  double WrapAngle(double angle);
  //IMU Calibration before starting the program
  void ResetIMUState();
  void CalibrateGyroZBias();
  //Setpoint Helpers
  void LoadAutonomousSetpoints();
  void AdvanceToNextSetpoint();
  void ResetPidState();


  //Pose Estimation helpers
  static constexpr units::meter_t kTrackWidth = 0.215_m;  //Real value, from center of the left wheel to center of the right wheel
  frc::DifferentialDriveKinematics m_driveKinematics{kTrackWidth};

  //Pose initial point, first setpoint
  frc::Pose2d m_initialPose{0.78_m, 0.48_m, frc::Rotation2d{0_rad}}; //change values, wait for Justice
  frc::Pose2d m_initialPose_BACKUP{0.81_m, 0.16_m, frc::Rotation2d{units::radian_t{-std::numbers::pi / 2.0}}};

  //Estimator declaration
  frc::DifferentialDrivePoseEstimator field_poseEstimator{
    m_driveKinematics,
    frc::Rotation2d{0_rad},
    0_m, //Left Wheel
    0_m, //Right Wheel
    frc::Pose2d{}
  };

  // Pose estimator helpers
  void ResetPoseEstimator(const frc::Pose2d& pose,
                          units::meter_t leftDist,
                          units::meter_t rightDist,
                          frc::Rotation2d gyroAngle);

  frc::Pose2d UpdatePoseEstimator(units::second_t timestamp,
                                  units::meter_t leftDist,
                                  units::meter_t rightDist,
                                  frc::Rotation2d gyroAngle);

  void AddVisionToPoseEstimator(const frc::Pose2d& visionPose,
                                     units::second_t timestamp,
                                     const wpi::array<double, 3>& stdDevs) {
    field_poseEstimator.AddVisionMeasurement(visionPose, timestamp, stdDevs);
}

  // ── Hall sensor ─────────────────────────────────────────────────────────
  frc::AnalogInput  m_hallAnalog{0};   // AI0
  frc::DigitalInput m_hallDigital{9};  // DIO 9

  // ── REV ServoHub ────────────────────────────────────────────────────────
  static constexpr int kServoHubId = 20;
  rev::servohub::ServoHub m_servoHub{kServoHubId};

  rev::servohub::ServoChannel& m_servoBrush{m_servoHub.GetServoChannel(rev::servohub::ServoChannel::ChannelId::kChannelId0)};//brush
  rev::servohub::ServoChannel &m_servoRelease{m_servoHub.GetServoChannel(rev::servohub::ServoChannel::ChannelId::kChannelId4)};//release orbs door
  rev::servohub::ServoChannel &m_servoHall{m_servoHub.GetServoChannel(rev::servohub::ServoChannel::ChannelId::kChannelId2)};//hall sensor
  rev::servohub::ServoChannel &m_servoArm{m_servoHub.GetServoChannel(rev::servohub::ServoChannel::ChannelId::kChannelId5)};//arm

  units::second_t m_servoCommandTime{-1_s};
  static constexpr double kServoDwell_s = 1.0;

  // Centering sub-states:
  // 0 = drive forward until tag found
  // 1 = center on tag pixel
  // 2 = arm raised, driving -y toward tag (approach to 13cm)
  // 3 = dwell — beacon dropping
  // 4 = done — lower arm, transition to APPROACH
  int m_centerStep = 0;
  double m_centerDwellStart_s = 0.0;
  static constexpr double kBeaconApproachDist_m = 0.13;  // stop 13cm from tag
  static constexpr double kBeaconDwell_s = 1.5;          // time to drop beacon

  bool firstloop;
  // PID Tuning gains
  double x_kP = 140.0;
  double x_kI = 25.0;
  double x_kD = 0.5;
  double y_kP = 0.0;
  double y_kI = 0.0;
  double y_kD = 0.00;
  double y_to_theta_kP = 3.0;
  double theta_kI = 5.0;
  double theta_kD = 0.3;
  // Forward/Backward wheel PID state
  double x_integral = 0.0;
  double x_prevError = 0.0;
  double x_deriv = 0.0;
  // Strafe wheel PID state
  double y_integral = 0.0;
  double y_prevError = 0.0;
  double y_deriv = 0.0;
  // Theta PID state
  double theta_integral  = 0.0;
  double theta_prevError = 0.0;
  double theta_deriv = 0.0;
  // Gyro bias (rad/s) – measured during CalibrateGyroZBias() in AutonomousInit
  double m_gyroZBiasRadps = 0.0;

  wpi::array<double, 3> GetVisionStdDevsFromConfidence(double confidence);

  // Time tracking for PID dt
  units::second_t m_prevTime   = 0_s;
  bool m_firstPidLoop = true;

  std::vector<Setpoint> m_setpoints;
  size_t m_currentSetpointIndex = 0;
  bool m_autoComplete = false;

  // Index of the last waypoint in Path_Default before handing off to
  // TAG_SEARCH. After arriving here the robot stops, reads the AprilTag ID,
  // and loads the corresponding sub-path. Matches the end of the bucket grab
  // sequence in Path_Default (waypoint [40] per CLAUDE.md waypoint table).
  // Waypoint index where robot stops and reads an AprilTag to select a sub-path.
  // Set to 999 to disable (path runs to completion without handoff).
  static constexpr size_t kTagHandoffWaypoint = 999;

  // How long (seconds) to spend on one waypoint before skipping to the next.
  static constexpr double kWaypointTimeout_s = 8.0;
  double m_waypointStartTime_s = 0.0;  // m_timer snapshot when current waypoint began

  // Servo pulse widths (microseconds)
  static constexpr int kHallServoInitPos    = 500;   // Closed
  static constexpr int kHallServoOpenPos    = 1500;  // Open
  static constexpr int BrushServoInitPos    = 500;
  static constexpr int BrushServoOpenPos    = 1500;
  static constexpr int ArmServoInitPos      = 1240;
  static constexpr int ArmServoOpenPos      = 1900;
  static constexpr int ReleaseServoInitPos  = 500;
  static constexpr int ReleaseServoOpenPos  = 1500;
  bool m_armRaised = false;
  bool m_armDropped = false;

  // ── IMU (MPU-6050 on I²C onboard port, addr 0x68) ──────────────────────
  frc::I2C m_imu{frc::I2C::Port::kOnboard, 0x68};

  bool  IMUWriteReg  (uint8_t reg, uint8_t data);
  bool  IMUReadRegs  (uint8_t startReg, uint8_t* out, int len);
  void  IMUInit      ();
  void  IMUUpdate    ();   // Single I2C read per tick; integrates gyro-Z → m_thetaRad
  void  IMUDashboard ();   // Pushes raw + converted values to SmartDashboard

  // Integrated yaw in radians. Updated by IMUUpdate() every RobotPeriodic() tick.
  // m_thetaRad is the bias-corrected heading used by the PID controller.
  // m_yaw_rad  is the un-corrected heading used by sweep / dashboard.
  double          m_thetaRad         = 0.0;  // bias-corrected, PID heading
  double          m_yaw_rad          = 0.0;  // raw integrated yaw (dashboard/sweep)
  double          m_lastGyroZ_radps  = 0.0;  // previous sample for trapezoidal integration
  units::second_t m_lastIMUTime{0};

  // ── Encoder snapshot used by sweep ─────────────────────────────────────
  // RoboClaw 0x80 M1 (left drive) → vertical odometer
  // Signed: positive = forward.
  int32_t m_vertTicks    = 0;
  bool    m_encodersValid = false;

  void UpdateEncoders();   // Reads drive odometry encoder every periodic tick

  // ── Vision / AprilTag ───────────────────────────────────────────────────
  AprilTagReader m_aprilTagReader;

  // ── Tag-approach autonomous phase ───────────────────────────────────────
  // The robot waits for a tag, then drives straight toward it until
  
  AutoPhase m_autoPhase = AutoPhase::SEARCH;
  bool m_taskDone = false;

  //stopping distance from the tag (meters)
  static constexpr double kStopDistance_m = 0.2;

  //forward drive speed while approaching (0-127)
  static constexpr uint8_t kApproachSpeed = 45;

  //lateral correction speed for centering on the tag (0-127)
  //applied as a steering blend when the tag is off-center in the camera frame
  static constexpr uint8_t kCenterSpeed = 80;

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
  static constexpr double kCameraCenter_px = 528.0;
 
  // How far off-center (pixels) before we apply a lateral correction.
  static constexpr double kCenterTolerance_px = 40.0;

  // Steering gain: fraction of base speed applied as differential at max error.
  // At pixelError == kCenterTolerance_px, inner wheel slows by kSteerFactor * baseSpeed.
  // Start at 0.3 and tune upward if the robot fails to track a laterally
  // offset tag, or downward if it oscillates.
  //   If the robot isn't correcting enough, increase it. If it oscillates side to side, decrease it.
  static constexpr double kSteerFactor = 0.1;
 
  // ── NetworkTables publishers ─────────────────────────────────────────────
  // Vision/task_done   → Pi reads this to know the robot has arrived.
  nt::BooleanPublisher m_taskDonePub;

  // ── Linear actuator (RoboClaw 0x81, M2) ─────────────────────────────────
  // Electrical confirmed: plugged into the M2 terminals on the same controller
  // on 0x81 M2. Extend pushes the ore deposit plate forward.
  void ActuatorExtend (uint8_t speed);
  void ActuatorRetract(uint8_t speed);
  void ActuatorStop   ();

  static constexpr uint8_t kActuatorSpeed     = 126;   // TODO: tune – 0-127
  static constexpr double  kActuatorRunTime_s = 5.0;  // TODO: tune – seconds to full extend/retract
  bool m_actuatorExtended = false;

  // Actuator dwell state — used during ore deposit waypoints.
  // 0 = idle, 1 = extending, 2 = retracting
  int    m_actuatorDwellStep = 0;
  double m_actuatorDwellStart_s = 0.0;

  // ── Bucket / beacon mechanisms ───────────────────────────────────────────
  // Only m_servoArm is used in the non-sorting system.
  void GrabBucket   ();  // Raise arm then clamp onto collection bucket
  void ArmRaise     ();  // Raise arm to beacon-drop angle (clears arena during turns)
  void ArmLower     ();  // Lower arm back to init position after beacon is deposited
  void DepositOres  ();  // Extend actuator at full speed to push all ores out

  // ── Test routines ────────────────────────────────────────────────────────
  // Call from TestInit()/TestPeriodic() to verify hardware before a match.
  void TestServo   ();  // Sweeps arm servo: grab → wait → release
  void TestActuator();  // Extends actuator → waits → retracts

  int m_testStep = 0;   // Step counter used by TestPeriodic() sequencer
};