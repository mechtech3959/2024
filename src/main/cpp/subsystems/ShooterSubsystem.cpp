#include "subsystems/ShooterSubsystem.h"

using namespace ShooterConstants;

ShooterSubsystem::ShooterSubsystem()
    : m_leftShooterMotor{kLeftShooterMotorID, kShooterCanBus},
      m_rightShooterMotor{kRightShooterMotorID, kShooterCanBus} {
  m_leftShooterMotor.GetConfigurator().Apply(
      ctre::phoenix6::configs::TalonFXConfiguration{});
  m_rightShooterMotor.GetConfigurator().Apply(
      ctre::phoenix6::configs::TalonFXConfiguration{});
  m_rightShooterMotor.SetInverted(true);
  m_rightShooterMotor.SetControl(ctre::phoenix6::controls::Follower{
      m_leftShooterMotor.GetDeviceID(), true});
}

void ShooterSubsystem::ShootAmp() { m_leftShooterMotor.Set(kAmpShootSpeed); }

void ShooterSubsystem::ShootSpeaker() {
  m_leftShooterMotor.Set(kSpeakerShootSpeed);
}

void ShooterSubsystem::Shoot(double speed) { m_leftShooterMotor.Set(speed); }
