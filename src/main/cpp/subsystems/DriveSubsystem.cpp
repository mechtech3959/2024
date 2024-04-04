// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"
#include "LimeLight.h"

#include <frc/DriverStation.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <memory>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/util/ReplanningConfig.h>

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : m_left1{kLeftFrontMotorID, rev::CANSparkMax::MotorType::kBrushed},
      m_left2{kLeftRearMotorID, rev::CANSparkMax::MotorType::kBrushed},
      m_right1{kRightFrontMotorID, rev::CANSparkMax::MotorType::kBrushed},
      m_right2{kRightRearMotorID, rev::CANSparkMax::MotorType::kBrushed},
      m_leftEncoder{kLeftEncoderID}, m_rightEncoder{kRightEncoderID},
      m_gyro{32, GeneralConstants::kCanBus},
      m_odometry{m_gyro.GetRotation2d(), units::meter_t{0}, units::meter_t{0}},
      m_kinematics{kTrackwidth}, m_leftFeedforward{kSLeft, kVLeft, kALeft},
      m_rightFeedforward{kSRight, kVRight, kARight} {
  wpi::SendableRegistry::AddChild(&m_drive, &m_left1);
  wpi::SendableRegistry::AddChild(&m_drive, &m_right1);

  m_left1.RestoreFactoryDefaults();
  m_left2.RestoreFactoryDefaults();
  m_right1.RestoreFactoryDefaults();
  m_right2.RestoreFactoryDefaults();
  m_left2.Follow(m_left1);
  m_right2.Follow(m_right1);
  // We need to invert one side of the drivetrain so that positive voltages
  // result in both sides moving forward. Depending on how your robot's
  // gearbox is constructed, you might have to invert the left side instead.
  m_left1.SetInverted(true);

  // Set the distance per pulse for the encoders
  // m_leftEncoder.SetDistancePerPulse(kEncoderDistancePerPulse.value());
  // m_rightEncoder.SetDistancePerPulse(kEncoderDistancePerPulse.value());

  ResetEncoders();

  // Configure the AutoBuilder last
  pathplanner::AutoBuilder::configureRamsete(
      [this]() { return GetPose(); }, // Robot pose supplier
      [this](frc::Pose2d pose) {
        ResetOdometry(pose);
      }, // Method to reset odometry (will be called if your auto has a starting
         // pose)
      [this]() {
        return GetChassisSpeeds();
      }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      [this](frc::ChassisSpeeds speeds) {
        DriveChassisSpeeds(speeds);
      }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      pathplanner::ReplanningConfig(), // Default path replanning config. See
                                       // the API for the options here
      []() {
        // Boolean supplier that controls when the path will be mirrored for the
        // red alliance This will flip the path being followed to the red side
        // of the field. THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        auto alliance = frc::DriverStation::GetAlliance();
        if (alliance) {
          return alliance.value() == frc::DriverStation::Alliance::kRed;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
  );
}
void DriveSubsystem::visionUpdate() {
  LimeLight Greenie{"limelight-greenie"};
  LimeLight Bakshot{"limelight-bakshot"};
  Greenie.updateTracking();
  if (Greenie.m_LimelightHasTarget)
    ResetOdometry(Greenie.GetRobotPose());
}
void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(
      m_gyro.GetRotation2d(),
      units::meter_t{m_leftEncoder.GetPosition().GetValueAsDouble() /
                     kRotationsPerMeter},
      units::meter_t{-m_rightEncoder.GetPosition().GetValueAsDouble() /
                     kRotationsPerMeter});
  frc::SmartDashboard::PutNumber(
      "LeftyEnc", m_leftEncoder.GetPosition().GetValueAsDouble());
  frc::SmartDashboard::PutNumber(
      "RightyEnc", -m_rightEncoder.GetPosition().GetValueAsDouble());
  visionUpdate();
  auto X = m_odometry.GetPose().X().;
  auto Y = m_odometry.GetPose().Y();
 // frc::SmartDashboard::Put("X Pos:", X);
 // frc::SmartDashboard::PutData("Y Pos:", Y);
}

void DriveSubsystem::ArcadeDrive(double fwd, double rot) {
  m_drive.ArcadeDrive(fwd, rot, true);
}

void DriveSubsystem::TankDriveVolts(units::volt_t left, units::volt_t right) {
  m_left1.SetVoltage(left);
  m_right1.SetVoltage(right);
  m_drive.Feed();
}

void DriveSubsystem::ResetEncoders() {
  m_leftEncoder.SetPosition(0_tr);
  m_rightEncoder.SetPosition(0_tr);
}

double DriveSubsystem::GetAverageEncoderDistance() {
  return (m_leftEncoder.GetPosition().GetValueAsDouble() +
          -m_rightEncoder.GetPosition().GetValueAsDouble()) /
         2.0 / kRotationsPerMeter;
}

ctre::phoenix6::hardware::CANcoder &DriveSubsystem::GetLeftEncoder() {
  return m_leftEncoder;
}

ctre::phoenix6::hardware::CANcoder &DriveSubsystem::GetRightEncoder() {
  return m_rightEncoder;
}

void DriveSubsystem::SetMaxOutput(double maxOutput) {
  m_drive.SetMaxOutput(maxOutput);
}

units::degree_t DriveSubsystem::GetHeading() const {
  return m_gyro.GetRotation2d().Degrees();
}

double DriveSubsystem::GetTurnRate() { return -m_gyro.GetRate(); }

frc::Pose2d DriveSubsystem::GetPose() { return m_odometry.GetPose(); }

frc::DifferentialDriveWheelSpeeds DriveSubsystem::GetWheelSpeeds() {
  return {
      units::meters_per_second_t{
          m_leftEncoder.GetVelocity().GetValueAsDouble() / kRotationsPerMeter},
      units::meters_per_second_t{
          -m_rightEncoder.GetVelocity().GetValueAsDouble() /
          kRotationsPerMeter}};
}

frc::ChassisSpeeds DriveSubsystem::GetChassisSpeeds() {
  return m_kinematics.ToChassisSpeeds(GetWheelSpeeds());
}

void DriveSubsystem::DriveChassisSpeeds(frc::ChassisSpeeds speeds) {
  auto [left, right] = m_kinematics.ToWheelSpeeds(speeds);

  TankDriveVolts(m_leftFeedforward.Calculate(left),
                 m_rightFeedforward.Calculate(right));
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(
      m_gyro.GetRotation2d(),
      units::meter_t{m_leftEncoder.GetPosition().GetValueAsDouble() /
                     kRotationsPerMeter},
      units::meter_t{-m_rightEncoder.GetPosition().GetValueAsDouble() /
                     kRotationsPerMeter},
      pose);
}
