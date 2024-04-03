// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : /*m_left1{2, rev::CANSparkMax::MotorType::kBrushed},
      m_left2{3, rev::CANSparkMax::MotorType::kBrushed},
      m_right1{4, rev::CANSparkMax::MotorType::kBrushed},
      m_right2{5, rev::CANSparkMax::MotorType::kBrushed},*/
      m_leftEncoder{30}, m_rightEncoder{31}, m_gyro{32},
      m_odometry{m_gyro.GetRotation2d(), units::meter_t{0}, units::meter_t{0}} {
  wpi::SendableRegistry::AddChild(&m_drive, &m_left1);
  wpi::SendableRegistry::AddChild(&m_drive, &m_right1);

  m_left2.Follow(m_left1);
  m_right2.Follow(m_right1);

  // We need to invert one side of the drivetrain so that positive voltages
  // result in both sides moving forward. Depending on how your robot's
  // gearbox is constructed, you might have to invert the left side instead.
  m_right1.SetInverted(true);

  // Set the distance per pulse for the encoders
  // m_leftEncoder.SetDistancePerPulse(kEncoderDistancePerPulse.value());
  // m_rightEncoder.SetDistancePerPulse(kEncoderDistancePerPulse.value());

  ResetEncoders();
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(
      m_gyro.GetRotation2d(),
      units::meter_t{m_leftEncoder.GetPosition().GetValueAsDouble()},
      units::meter_t{m_rightEncoder.GetPosition().GetValueAsDouble()});
}

void DriveSubsystem::ArcadeDrive(double fwd, double rot) {
  // m_drive.ArcadeDrive(fwd, rot, true);
  m_left1.Set(fwd);
  m_right1.Set(rot);
  m_drive.Feed();
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
          m_rightEncoder.GetPosition().GetValueAsDouble()) /
         2.0;
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
  return {units::meters_per_second_t{
              m_leftEncoder.GetVelocity().GetValueAsDouble()},
          units::meters_per_second_t{
              m_rightEncoder.GetVelocity().GetValueAsDouble()}};
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(
      m_gyro.GetRotation2d(),
      units::meter_t{m_leftEncoder.GetPosition().GetValueAsDouble()},
      units::meter_t{m_rightEncoder.GetPosition().GetValueAsDouble()}, pose);
}
