#include "subsystems/DriveSubsystem.h"
#include "Constants.h"
#include "LimeLight.h"

#include <chrono>
#include <frc/DriverStation.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <thread>

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : m_left1{kLeftFrontMotorID, rev::CANSparkMax::MotorType::kBrushed},
      m_left2{kLeftRearMotorID, rev::CANSparkMax::MotorType::kBrushed},
      m_right1{kRightFrontMotorID, rev::CANSparkMax::MotorType::kBrushed},
      m_right2{kRightRearMotorID, rev::CANSparkMax::MotorType::kBrushed},
      m_leftEncoder{kLeftEncoderID}, m_rightEncoder{kRightEncoderID},
      m_gyro{32, GeneralConstants::kCanBus},
      m_odometry{m_gyro.GetRotation2d(), units::meter_t{0}, units::meter_t{0}},
      m_kinematics{kTrackWidth}, m_leftFeedforward{kSLeft, kVLeft, kALeft},
      m_rightFeedforward{kSRight, kVRight, kARight} {
  wpi::SendableRegistry::AddChild(&m_drive, &m_left1);
  wpi::SendableRegistry::AddChild(&m_drive, &m_right1);

  // Restore all drive motor configurations to factory defaults
  m_left1.RestoreFactoryDefaults();
  m_left2.RestoreFactoryDefaults();
  m_right1.RestoreFactoryDefaults();
  m_right2.RestoreFactoryDefaults();
  // Sleep for one second so we don't overload the CAN bus when setting up
  // followers
  // If we don't do this follower setup will randomly fail and half of the
  // drivetrain will only run on one motor
  std::this_thread::sleep_for(std::chrono::seconds(1));
  m_left2.Follow(m_left1);
  m_right2.Follow(m_right1);
  // Invert the left side so positive voltages move both sides in the same
  // direction
  m_left1.SetInverted(true);

  m_leftEncoder.SetPosition(0_tr);
  m_rightEncoder.SetPosition(0_tr);

  // Configure the AutoBuilder last
  pathplanner::AutoBuilder::configureRamsete(
      [this]() { return GetPose(); },
      [this](frc::Pose2d pose) { ResetOdometry(pose); },
      [this]() { return m_kinematics.ToChassisSpeeds(GetWheelSpeeds()); },
      [this](frc::ChassisSpeeds speeds) {
        auto [left, right] = m_kinematics.ToWheelSpeeds(speeds);

        TankDriveVolts(m_leftFeedforward.Calculate(left),
                       m_rightFeedforward.Calculate(right));
      },
      pathplanner::ReplanningConfig(),
      []() {
        return frc::DriverStation::GetAlliance().value() ==
               frc::DriverStation::Alliance::kRed;
      },
      this);
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
  // auto X = m_odometry.GetPose().X();
  // auto Y = m_odometry.GetPose().Y();
  //  frc::SmartDashboard::Put("X Pos:", X);
  //  frc::SmartDashboard::PutData("Y Pos:", Y);
}

void DriveSubsystem::ArcadeDrive(double fwd, double rot) {
  m_drive.ArcadeDrive(fwd, rot, true);
}

void DriveSubsystem::TankDriveVolts(units::volt_t left, units::volt_t right) {
  m_left1.SetVoltage(left);
  m_right1.SetVoltage(-right);
  m_drive.Feed();
}

ctre::phoenix6::hardware::Pigeon2 &DriveSubsystem::GetPigeon() {
  return m_gyro;
}

void DriveSubsystem::SetMaxOutput(double maxOutput) {
  m_drive.SetMaxOutput(maxOutput);
}

frc::Pose2d DriveSubsystem::GetPose() { return m_odometry.GetPose(); }

frc::DifferentialDriveWheelSpeeds DriveSubsystem::GetWheelSpeeds() {
  return {
      units::meters_per_second_t{
          m_leftEncoder.GetVelocity().GetValueAsDouble() / kRotationsPerMeter},
      units::meters_per_second_t{
          -m_rightEncoder.GetVelocity().GetValueAsDouble() /
          kRotationsPerMeter}};
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
