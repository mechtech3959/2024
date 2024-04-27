#include "subsystems/ShooterSubsystem.h"
#include "Constants.h"

using namespace ShooterConstants;

ShooterSubsystem::ShooterSubsystem()
    : m_leftShooterMotor{kLeftShooterMotorID, GeneralConstants::kCanBus},
      m_rightShooterMotor{kRightShooterMotorID, GeneralConstants::kCanBus} {
  m_leftShooterMotor.GetConfigurator().Apply(
      ctre::phoenix6::configs::TalonFXConfiguration{});
  m_rightShooterMotor.GetConfigurator().Apply(
      ctre::phoenix6::configs::TalonFXConfiguration{});
  m_leftShooterMotor.SetInverted(true);
  m_rightShooterMotor.SetControl(ctre::phoenix6::controls::Follower{
      m_leftShooterMotor.GetDeviceID(), true});
}

void ShooterSubsystem::ShootAmp() { m_leftShooterMotor.Set(kAmpShootSpeed); }

void ShooterSubsystem::ShootSpeaker() {
  m_leftShooterMotor.Set(kSpeakerShootSpeed);
}

void ShooterSubsystem::Reverse() { m_leftShooterMotor.Set(-1); }

void ShooterSubsystem::Stop() { m_leftShooterMotor.Set(0); }

// void ShooterSubsystem::Shoot(double speed) { m_leftShooterMotor.Set(speed); }
